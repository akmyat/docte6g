import os
import time
import numpy as np

import mitsuba as mi
for _variant in ("cuda_ad_mono_polarized", "cuda_ad_rgb", "llvm_ad_rgb"):
    if _variant in mi.variants():
        mi.set_variant(_variant)
        break
else:
    raise ImportError(f"No supported Mitsuba variant found. Available variants: {mi.variants()}")

import drjit as dr
from sionna.rt import load_scene, PlanarArray, Transmitter, Receiver
from sionna.rt import PathSolver, subcarrier_frequencies, InteractionType
from sionna.rt import RadioMaterial, SceneObject, AntennaPattern
from sionna.rt.antenna_pattern import (
    PolarizedAntennaPattern,
    antenna_pattern_registry,
    v_tr38901_pattern,
)
from sionna.rt.utils import r_hat
from sionna.phy import SPEED_OF_LIGHT

_ISAC_AVAILABLE = True
try:
    from sklearn.cluster import DBSCAN
    from scipy.optimize import linear_sum_assignment
    from scipy.spatial import cKDTree
except ImportError:
    _ISAC_AVAILABLE = False
    DBSCAN = None
    linear_sum_assignment = None
    cKDTree = None


# ------------------- START of ISAC helpers -------------------
class KalmanFilter3D:
    def __init__(self, initial_state, dt=1.0, Q_val=0.5, R_val=0.05):
        self.x = np.array([initial_state[0], initial_state[1], initial_state[2],
                           0.0, 0.0, 0.0]).reshape(6, 1)
        self.start_pos = np.asarray(initial_state, dtype=float).copy()
        self.F = np.eye(6)
        self.F[0, 3] = dt; self.F[1, 4] = dt; self.F[2, 5] = dt
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1; self.H[1, 1] = 1; self.H[2, 2] = 1
        self.P = np.eye(6) * 5.0
        self.Q = np.eye(6) * Q_val
        self.R = np.eye(3) * R_val
        self.missed_frames = 0
        self.age = 1
        self._confirmed = False
        self.updated_this_frame = True

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:3].flatten()

    def update(self, z):
        z = np.array(z).reshape(3, 1)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.missed_frames = 0
        self.age += 1
        return self.x[:3].flatten()

    def get_total_displacement(self):
        return float(np.linalg.norm(self.x[:3].flatten() - self.start_pos))

    def is_confirmed(self, min_age, min_dist):
        if not self._confirmed:
            if self.age >= min_age and self.get_total_displacement() >= min_dist:
                self._confirmed = True
        return self._confirmed


class Tracker:
    def __init__(self, max_missed=3, min_age_for_output=3, min_displacement=0.3, max_tracks=10, dt=1.0, Q_val=0.5, R_val=0.05):
        self.tracks = []
        self.max_missed = max_missed
        self.min_age = min_age_for_output
        self.min_dist = min_displacement
        self.max_tracks = max_tracks
        self.dt = dt
        self.Q_val = Q_val
        self.R_val = R_val

    def process_frame(self, detections):
        predictions = [t.predict() for t in self.tracks]
        unmatched = set(range(len(detections)))
        for t in self.tracks:
            t.updated_this_frame = False

        if self.tracks and detections:
            cost = np.zeros((len(self.tracks), len(detections)))
            for i, p in enumerate(predictions):
                for j, d in enumerate(detections):
                    cost[i, j] = np.linalg.norm(p - d['position'])
            MAX_COST = 3.0
            cost[cost > MAX_COST] = 1000.0
            row_ind, col_ind = linear_sum_assignment(cost)
            for i, j in zip(row_ind, col_ind):
                if cost[i, j] < MAX_COST:
                    self.tracks[i].update(detections[j]['position'])
                    self.tracks[i].updated_this_frame = True
                    unmatched.discard(j)
                else:
                    self.tracks[i].missed_frames += 1
            unassigned = set(range(len(self.tracks))) - set(row_ind)
            for i in unassigned:
                self.tracks[i].missed_frames += 1
        elif self.tracks:
            for t in self.tracks:
                t.missed_frames += 1

        for j in unmatched:
            if len(self.tracks) < self.max_tracks:
                self.tracks.append(KalmanFilter3D(detections[j]['position'], dt=self.dt, Q_val=self.Q_val, R_val=self.R_val))

        self.tracks = [t for t in self.tracks if t.missed_frames < self.max_missed]

        moving = []
        for t in self.tracks:
            if t.is_confirmed(self.min_age, self.min_dist):
                moving.append(t.x[:3].flatten())
        return moving


class CloudMTI:
    def __init__(self, dist_thresh=0.5):
        self.dist_thresh = dist_thresh
        self.background_map = None
        self._primed = False

    def prime(self, erp_pts):
        """Seed the static-background map from a target-free warm-up frame.

        Multiple calls accumulate background points so several warm-up frames
        can be combined before any moving target enters the scene.
        """
        if len(erp_pts) == 0:
            return
        if self.background_map is None:
            self.background_map = np.asarray(erp_pts).copy()
        else:
            self.background_map = np.vstack([self.background_map, np.asarray(erp_pts)])
        self._primed = True

    def filter(self, erp_pts, powers):
        if len(erp_pts) == 0:
            return erp_pts, powers
        if self.background_map is None:
            # Do not learn the background from a live sensing frame: that frame
            # contains targets and would make MTI suppress true positives. The
            # notebook primes MTI explicitly with target-free clutter frames.
            return erp_pts, powers
        tree = cKDTree(self.background_map)
        dists, _ = tree.query(erp_pts)
        mask = dists > self.dist_thresh
        return erp_pts[mask], powers[mask]


def _extract_erps(paths_obj, gnb_pos, min_power=1e-25,
                  z_min=0.0, z_max=2.0,
                  single_bounce_only=True, phase_center_offset=0.0):
    """Extract raw ERP points and powers from a Sionna paths object.

    Returns (ERP, powers) arrays after bounce filter, power gate, and z-gate.
    Does NOT apply MTI or DBSCAN — use this to collect ERPs from multiple
    monostatic solves before merging and clustering.
    """
    try:
        a = np.array(paths_obj.a)
        tau = np.array(paths_obj.tau)
        theta_r = np.array(paths_obj.theta_r)
        phi_r = np.array(paths_obj.phi_r)
        # Sum over all antenna-element pairs so power scales as N_tx_ant × N_rx_ant
        # (coherent MIMO radar gain), not mean-per-element which is array-size-invariant.
        powers = np.sum(np.abs(a) ** 2, axis=tuple(range(a.ndim - 1)))
        tau = np.mean(tau, axis=tuple(range(tau.ndim - 1)))
        theta_r = np.mean(theta_r, axis=tuple(range(theta_r.ndim - 1)))
        phi_r = np.mean(phi_r, axis=tuple(range(phi_r.ndim - 1)))
    except Exception:
        return np.empty((0, 3)), np.empty(0)

    bounces = None
    try:
        inter = np.array(paths_obj.interactions)
        if inter.ndim >= 2:
            depth = inter.shape[0]
            num_paths_inter = inter.shape[-1]
            flat = inter.reshape(depth, -1, num_paths_inter)
            mask = flat != int(InteractionType.NONE)
            bounces = mask.any(axis=1).sum(axis=0)
    except Exception:
        bounces = None

    valid = tau > 1e-9
    if single_bounce_only and bounces is not None and bounces.shape == tau.shape:
        valid &= (bounces == 1)
    tau, powers = tau[valid], powers[valid]
    theta_r, phi_r = theta_r[valid], phi_r[valid]
    if len(tau) == 0:
        return np.empty((0, 3)), np.empty(0)

    p_gNB = np.array(gnb_pos).flatten()[:3]
    L = SPEED_OF_LIGHT * tau
    distance_scaling = np.maximum(L / 10.0, 1.0) ** 1.1
    dynamic_threshold = min_power / distance_scaling
    vp = powers > dynamic_threshold
    tau, powers, L = tau[vp], powers[vp], L[vp]
    theta_r, phi_r = theta_r[vp], phi_r[vp]
    if len(tau) == 0:
        return np.empty((0, 3)), np.empty(0)

    d_dir = np.stack((np.sin(theta_r) * np.cos(phi_r),
                      np.sin(theta_r) * np.sin(phi_r),
                      np.cos(theta_r)), axis=-1)
    ERP = p_gNB + ((L[:, np.newaxis] / 2.0) + float(phase_center_offset)) * d_dir
    vz = (ERP[:, 2] >= z_min) & (ERP[:, 2] <= z_max)
    return ERP[vz], powers[vz]


def detect_objects_realistic_noisy(paths_obj, gnb_pos, min_power=1e-25,
                                   eps_cluster=2.0, max_targets=10,
                                   z_min=0.0, z_max=2.0, mti_filter=None,
                                   single_bounce_only=True,
                                   phase_center_offset=0.0):
    """Cluster ray-traced echo paths into target detections.

    ``single_bounce_only`` keeps only paths whose echo has exactly one
    scatter interaction. The monostatic ERP geometry ``p_gNB + (L/2) * d_dir``
    is only correct for such paths; longer paths localise the *last*
    scatterer, not the target, and contribute spurious clusters.

    ``phase_center_offset`` (metres) is an optional radial bias applied to
    every ERP. Set explicitly during sensor calibration; do not use as a
    free fitting parameter.
    """
    try:
        a = np.array(paths_obj.a)
        tau = np.array(paths_obj.tau)
        theta_r = np.array(paths_obj.theta_r)
        phi_r = np.array(paths_obj.phi_r)
        powers = np.mean(np.abs(a) ** 2, axis=tuple(range(a.ndim - 1)))
        tau = np.mean(tau, axis=tuple(range(tau.ndim - 1)))
        theta_r = np.mean(theta_r, axis=tuple(range(theta_r.ndim - 1)))
        phi_r = np.mean(phi_r, axis=tuple(range(phi_r.ndim - 1)))
    except Exception:
        return []

    # Per-path bounce count from the interactions tensor. Sionna exposes a
    # shape (max_depth, ..., num_paths) integer array of InteractionType
    # values where NONE marks unused depth slots. Counting non-NONE entries
    # along the depth axis gives the number of scatter interactions for
    # each path. We collapse any intermediate (rx, tx) axes by taking the
    # max — for a path that exists on any link it suffices that any link
    # observes the interaction.
    bounces = None
    try:
        inter = np.array(paths_obj.interactions)
        if inter.ndim >= 2:
            depth = inter.shape[0]
            num_paths_inter = inter.shape[-1]
            flat = inter.reshape(depth, -1, num_paths_inter)
            mask = flat != int(InteractionType.NONE)
            bounces = mask.any(axis=1).sum(axis=0)  # shape (num_paths,)
    except Exception:
        bounces = None

    valid = (tau > 1e-9)
    if single_bounce_only and bounces is not None and bounces.shape == tau.shape:
        valid &= (bounces == 1)
    tau, powers = tau[valid], powers[valid]
    theta_r, phi_r = theta_r[valid], phi_r[valid]
    if len(tau) == 0:
        return []

    p_gNB = np.array(gnb_pos).flatten()[:3]
    L = SPEED_OF_LIGHT * tau
    distance_scaling = np.maximum(L / 10.0, 1.0) ** 1.1
    dynamic_threshold = min_power / distance_scaling
    vp = powers > dynamic_threshold
    tau, powers = tau[vp], powers[vp]
    theta_r, phi_r = theta_r[vp], phi_r[vp]
    L = L[vp]
    if len(tau) == 0:
        return []

    d_dir = np.stack((np.sin(theta_r) * np.cos(phi_r),
                      np.sin(theta_r) * np.sin(phi_r),
                      np.cos(theta_r)), axis=-1)
    ERP = p_gNB + ((L[:, np.newaxis] / 2.0) + float(phase_center_offset)) * d_dir
    vz = (ERP[:, 2] >= z_min) & (ERP[:, 2] <= z_max)
    ERP = ERP[vz]
    powers = powers[vz]

    if mti_filter is not None:
        ERP_dyn, powers_dyn = mti_filter.filter(ERP, powers)
    else:
        ERP_dyn, powers_dyn = ERP, powers

    if len(ERP_dyn) == 0:
        return []

    labels = DBSCAN(eps=eps_cluster, min_samples=2).fit(ERP_dyn).labels_
    ids = sorted(set(labels) - {-1})
    raw = []
    for k in ids:
        m = labels == k
        best_idx = np.argmax(powers_dyn[m])
        raw.append({"position": ERP_dyn[m][best_idx],
                    "power": float(powers_dyn[m].sum())})
    raw.sort(key=lambda x: x['power'], reverse=True)
    if max_targets:
        raw = raw[:max_targets]
    return [{"id": i, "position": r['position'], "power": r['power']}
            for i, r in enumerate(raw)]


def _read_ply_bbox(fname_ply):
    with open(fname_ply, 'rb') as f:
        n_verts, n_props, fmt = 0, 0, ""
        for _ in range(2048):  # cap header scan to avoid infinite loops on non-PLY files
            raw = f.readline()
            if not raw:
                raise ValueError(f"{fname_ply}: not a PLY file (no end_header)")
            try:
                ln = raw.decode('ascii', errors='replace').strip()
            except Exception:
                raise ValueError(f"{fname_ply}: header is not ASCII")
            if "end_header" in ln:
                break
            if "element vertex" in ln:
                n_verts = int(ln.split()[-1])
            if "property float" in ln:
                n_props += 1
            if "format" in ln:
                fmt = ln
        else:
            raise ValueError(f"{fname_ply}: PLY header exceeds 2048 lines")
        if "ascii" in fmt:
            verts = np.loadtxt(f, max_rows=n_verts)
        else:
            verts = np.frombuffer(f.read(n_verts * n_props * 4),
                                  dtype='f4').reshape(n_verts, n_props)
    xyz = verts[:, :3]
    return xyz.min(axis=0), xyz.max(axis=0)


def calibrate_detect_objects(fname_ply, scene_object=None, obj_z_override=None):
    bbox_min, bbox_max = _read_ply_bbox(fname_ply)
    size_model = (bbox_max - bbox_min).astype(float)
    unit_scale = 0.01 if size_model.max() > 10.0 else 1.0
    size_world = size_model * unit_scale
    if scene_object is not None:
        try:
            size_world = size_model * np.abs(np.array(scene_object.scale).flatten())
        except Exception:
            pass
    dx, dy, dz = size_world
    footprint_r = 0.5 * np.sqrt(dx ** 2 + dy ** 2)
    eps_cluster = max(2.0 * footprint_r, 1.5)
    if obj_z_override is not None:
        obj_z = float(obj_z_override)
    elif scene_object is not None:
        try:
            obj_z = float(np.array(scene_object.position).flatten()[2])
        except Exception:
            obj_z = float(bbox_min[2] * unit_scale)
    else:
        obj_z = float(bbox_min[2] * unit_scale)
    margin = dz * 0.5 + 0.5
    return {
        "eps_cluster": float(eps_cluster),
        "z_min": float(obj_z - margin),
        "z_max": float(obj_z + dz + margin),
        "min_power": float(1e-25),
        "bbox_size": size_world,
    }
# ------------------- END of ISAC helpers -------------------


# ------------------- START of Beamforming helpers -------------------
_ACTIVE_PATTERN = None
_PATTERN_REGISTERED = False


class BeamformingPattern(AntennaPattern):
    """Multi-lobe antenna pattern that steers Gaussian beams toward detected targets."""

    def __init__(self, target_angles, width_deg=15.0, polarization="V"):
        super().__init__()
        self.target_angles = target_angles
        self.width_deg = width_deg
        self._patterns = PolarizedAntennaPattern(
            v_pattern=self._make_pattern(target_angles, width_deg),
            polarization=polarization,
        ).patterns

    @staticmethod
    def _make_pattern(target_angles, width_deg):
        sigma = np.deg2rad(width_deg)
        var = sigma ** 2

        def pattern(theta, phi):
            gain = dr.zeros(mi.Float, dr.width(theta))
            for th_t, ph_t in target_angles:
                r = r_hat(theta, phi)
                r_t = r_hat(th_t, ph_t)
                cos_angle = dr.clip(r.x * r_t.x + r.y * r_t.y + r.z * r_t.z, -1.0, 1.0)
                gain += dr.exp(-0.5 * dr.acos(cos_angle) ** 2 / var)
            baseline = v_tr38901_pattern(theta, phi)
            baseline_linear_gain = dr.squared_norm(baseline)
            # Preserve the communication pattern away from sensed targets and
            # apply at most 6 dB of sensing-assisted directional improvement.
            pattern_linear_gain = baseline_linear_gain * (1.0 + 3.0 * dr.minimum(gain, 1.0))
            return mi.Complex2f(dr.sqrt(pattern_linear_gain), 0.0)
        return pattern

    @property
    def patterns(self):
        return self._patterns

    @staticmethod
    def from_positions(radar_position, detected_positions, width_deg=15.0, polarization="V"):
        radar_pos = np.asarray(radar_position, dtype=float)
        angles = []
        for pos in detected_positions:
            vec = np.asarray(pos, dtype=float) - radar_pos
            r = float(np.linalg.norm(vec))
            if r < 1e-6:
                continue
            theta = float(np.arccos(vec[2] / r))
            phi = float(np.arctan2(vec[1], vec[0]))
            angles.append((theta, phi))
        if not angles:
            angles = [(np.pi / 2, 0.0)]
        return BeamformingPattern(angles, width_deg=width_deg, polarization=polarization)

    def to_planar_array(self, num_rows, num_cols, spacing, polarization="V"):
        global _ACTIVE_PATTERN
        _ACTIVE_PATTERN = self
        return PlanarArray(
            num_rows=num_rows,
            num_cols=num_cols,
            vertical_spacing=spacing,
            horizontal_spacing=spacing,
            pattern="multi_lobe",
            polarization=polarization,
        )

    @classmethod
    def register(cls, name="multi_lobe"):
        global _PATTERN_REGISTERED
        if _PATTERN_REGISTERED:
            return
        try:
            def factory(**kwargs):
                global _ACTIVE_PATTERN
                if _ACTIVE_PATTERN is not None:
                    return _ACTIVE_PATTERN
                return cls([(np.pi / 2, 0.0)], polarization=kwargs.get("polarization", "V"))
            antenna_pattern_registry.register(name=name, obj=factory)
            _PATTERN_REGISTERED = True
        except Exception:
            _PATTERN_REGISTERED = True


BeamformingPattern.register()
# ------------------- END of Beamforming helpers -------------------


class SionnaRT:
    def __init__(self):
        self.scene = None
        self.path_solver = PathSolver()
        self.assets_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../../assets")
        )
        self.max_depth = 3
        self.diffuse_reflection = False
        self.comm_static_clutter_scattering = 0.0
        # Cache coherence parameters must match C++ SionnaPropagationCache attributes.
        # delta = alpha * d * 0.886 / tx_num_cols; step_dist = delta * buffer.
        self.coherence_alpha = 0.4
        self.coherence_tx_num_cols = 8
        self.virtual_position_min_separation = 0.5  # floor on step_dist [m]
        self.cache_threshold_buffer = 1.1            # 10% overlap so windows never gap
        self.adaptive_future_horizon_seconds = 3.0
        self.adaptive_future_min_benefit_seconds = 1.0
        self.adaptive_future_max_steps = 3
        self.adaptive_future_direction_dot_threshold = 0.7
        self.rx_speeds = {}
        self.rx_prev_positions = {}
        self.rx_last_directions = {}
        self.rx_direction_dots = {}
        self.propagation_calculation_calls = 0
        self.propagation_calculation_receiver_count = 0
        self.perf_stats = {}

        # ISAC state (opt-in via initialize(sim_settings["enable_situation_awareness"]))
        self.enable_SA = False
        self.mti_filter = None
        self.tracker = None
        self.rx_material = None
        self.rx_type_path = None
        self.calib = None
        self.beamwidth_deg = 20.0
        self.isac_min_power = 1e-25
        self.isac_eps_cluster = 0.8
        self.isac_min_displacement = 0.3
        self.isac_max_depth = 3
        self.isac_diffuse_reflection = True
        self.isac_static_clutter_scattering = 0.0
        self.isac_samples_per_src = 1_000_000
        self.isac_single_bounce_only = True
        self.isac_phase_center_offset = 0.0
        self.isac_mti_warmup_frames = 0
        self.isac_mti_dist_thresh = 0.5   # kept separate from eps_cluster
        self.isac_tracker_min_age = 3
        self.isac_dbscan_min_samples = 1
        self.isac_detection_roi = None
        self.isac_enable_motion_track_fallback = True
        self.isac_track_match_radius = 2.0
        self._last_sensing_rx_positions = {}
        self.radar_tx_pos = None
        self._radar_tx_array = None
        self._radar_rx_array = None
        self._comm_tx_polarization = "VH"
        self._comm_tx_num_rows = 8
        self._comm_tx_num_cols = 8
        self._comm_tx_v_spacing = 0.0
        self._comm_tx_h_spacing = 0.0
        self._beam_epoch = 0
        self._beam_active = False
        self.detection_history = []
        self.raw_detection_history = []

    def _perf_add(self, name: str, value: float):
        self.perf_stats[name] = float(self.perf_stats.get(name, 0.0)) + float(value)

    def get_perf_stats(self):
        stats = dict(self.perf_stats)
        stats["propagation_calculation_calls"] = float(self.propagation_calculation_calls)
        stats["propagation_calculation_receiver_count"] = float(
            self.propagation_calculation_receiver_count
        )
        return stats

    def initialize(self, sim_settings):
        init_start = time.perf_counter()
        self.scene = load_scene(
            sim_settings.get("scene", self.assets_dir + "/scenes/free_space/free_space.xml")
        )

        f_c = sim_settings.get("carrier_frequency", 3.5e9)
        num_subcarriers = sim_settings.get("num_subcarriers", 3276)
        subcarrier_spacing = sim_settings.get("subcarrier_spacing", 30000)
        self.coherence_tx_num_cols = sim_settings.get("tx_num_cols", self.coherence_tx_num_cols)
        self.max_depth = int(sim_settings.get("comm_max_depth", self.max_depth))
        self.diffuse_reflection = bool(
            sim_settings.get("comm_diffuse_reflection", self.diffuse_reflection)
        )
        self.comm_static_clutter_scattering = float(
            sim_settings.get(
                "comm_static_clutter_scattering",
                self.comm_static_clutter_scattering,
            )
        )

        if self.diffuse_reflection and self.comm_static_clutter_scattering > 0.0:
            for obj in self.scene.objects.values():
                try:
                    mat = obj.radio_material
                    if mat is not None:
                        scattering = float(np.asarray(mat.scattering_coefficient).ravel()[0])
                        if scattering < self.comm_static_clutter_scattering:
                            mat.scattering_coefficient = self.comm_static_clutter_scattering
                except Exception:
                    pass

        freqs = subcarrier_frequencies(num_subcarriers, subcarrier_spacing) + f_c
        self.export_frequencies = np.asarray(freqs)
        self.export_num_subcarriers = num_subcarriers

        # Downsample to one frequency per resource block (RB centre subcarrier)
        if num_subcarriers >= 12 and num_subcarriers % 12 == 0:
            n_rb = num_subcarriers // 12
            rb_centers = [min(rb * 12 + 6, num_subcarriers - 1) for rb in range(n_rb)]
            self.export_frequencies = self.export_frequencies[rb_centers]
            self.export_num_subcarriers = n_rb

        wavelength = SPEED_OF_LIGHT / f_c
        v_spacing = sim_settings.get("vertical_array_spacing", wavelength / 2)
        h_spacing = sim_settings.get("horizontal_array_spacing", wavelength / 2)
        pattern = sim_settings.get("pattern", "iso")
        polarization = sim_settings.get("polarization", "VH")

        array_kwargs = dict(
            vertical_spacing=v_spacing,
            horizontal_spacing=h_spacing,
            pattern=pattern,
            polarization=polarization,
        )
        tx_num_rows = sim_settings.get("tx_num_rows", 8)
        tx_num_cols = sim_settings.get("tx_num_cols", 8)
        rx_num_rows = sim_settings.get("rx_num_rows", 2)
        rx_num_cols = sim_settings.get("rx_num_cols", 2)
        self.scene.tx_array = PlanarArray(
            num_rows=tx_num_rows,
            num_cols=tx_num_cols,
            **array_kwargs,
        )
        self.scene.rx_array = PlanarArray(
            num_rows=rx_num_rows,
            num_cols=rx_num_cols,
            **array_kwargs,
        )
        # Capture comm-array geometry for ISAC array swaps
        self._comm_tx_num_rows = tx_num_rows
        self._comm_tx_num_cols = tx_num_cols
        self._comm_rx_num_rows = rx_num_rows
        self._comm_rx_num_cols = rx_num_cols
        self._comm_tx_v_spacing = v_spacing
        self._comm_tx_h_spacing = h_spacing
        self._comm_tx_polarization = polarization
        self._comm_f_c = f_c

        tx_names = sim_settings.get("tx_names", [""])
        tx_ids = sim_settings.get("tx_ids", [])
        tx_locations = sim_settings.get("tx_locations", [[0.0, 0.0, 20.0]])
        tx_power = sim_settings.get("tx_power", 46.0)
        tx_look_at = sim_settings.get("tx_look_at", [])
        self.transmitters = dict(zip(tx_names, tx_ids))
        for name, pos in zip(tx_names, tx_locations):
            self.scene.add(Transmitter(name=name, position=pos, power_dbm=tx_power, color=[1, 0, 0]))
        for name, target in zip(tx_names, tx_look_at):
            if target is not None:
                self.scene.transmitters[name].look_at(target)

        rx_names = sim_settings.get("rx_names", [""])
        rx_ids = sim_settings.get("rx_ids", [])
        rx_locations = sim_settings.get("rx_locations", [[50.0, 50.0, 1.5]])
        rx_speed_list = sim_settings.get("rx_speed", [])
        self.cache_threshold_buffer = float(sim_settings.get("cache_threshold_buffer", self.cache_threshold_buffer))
        self.adaptive_future_horizon_seconds = float(
            sim_settings.get("adaptive_future_horizon_seconds", self.adaptive_future_horizon_seconds)
        )
        self.adaptive_future_min_benefit_seconds = float(
            sim_settings.get("adaptive_future_min_benefit_seconds", self.adaptive_future_min_benefit_seconds)
        )
        self.adaptive_future_max_steps = int(
            sim_settings.get("adaptive_future_max_steps", self.adaptive_future_max_steps)
        )
        self.adaptive_future_direction_dot_threshold = float(
            sim_settings.get(
                "adaptive_future_direction_dot_threshold",
                self.adaptive_future_direction_dot_threshold,
            )
        )
        self.receivers = dict(zip(rx_names, rx_ids))
        self.rx_speeds = {name: float(speed) for name, speed in zip(rx_names, rx_speed_list)}
        self.rx_prev_positions = {name: list(pos) for name, pos in zip(rx_names, rx_locations)}
        self.rx_last_directions = {}
        self.rx_direction_dots = {}
        for name, pos in zip(rx_names, rx_locations):
            self.scene.add(Receiver(name=name, position=pos, color=[0, 1, 0], display_radius=0.5))

        rx_mesh = sim_settings.get("rx_mesh", self.assets_dir + "/objects/cube.obj")
        self.rx_object_z_offset = float(sim_settings.get("rx_object_z_offset", 0.0))
        rx_scattering = float(sim_settings.get("isac_rx_scattering_coefficient", 0.5))
        self._rx_mesh_path = rx_mesh        # used by prime_mti_background to restore meshes
        self._rx_scattering = rx_scattering
        rx_material = RadioMaterial(
            "rx_material", relative_permittivity=1.0, conductivity=1e10,
            scattering_coefficient=rx_scattering,
        )
        self.rx_material = rx_material
        self.scene.edit(add=[
            SceneObject(name=f"rx_obj_{n}", fname=rx_mesh, radio_material=rx_material)
            for n in rx_names
        ])
        for name, pos in zip(rx_names, rx_locations):
            self.scene.get(f"rx_obj_{name}").position = [
                pos[0], pos[1], pos[2] - self.rx_object_z_offset
            ]
        self._beam_target_z = None
        if rx_locations:
            self._beam_target_z = float(np.median([float(pos[2]) for pos in rx_locations]))
            roi_margin = float(sim_settings.get("isac_detection_roi_margin", 5.0))
            rx_arr = np.asarray(rx_locations, dtype=float)
            self.isac_detection_roi = (
                float(np.min(rx_arr[:, 0]) - roi_margin),
                float(np.max(rx_arr[:, 0]) + roi_margin),
                float(np.min(rx_arr[:, 1]) - roi_margin),
                float(np.max(rx_arr[:, 1]) + roi_margin),
            )

        # Stash radar TX position (first TX) for ISAC detection geometry
        if tx_locations:
            self.radar_tx_pos = list(tx_locations[0])

        # --- ISAC initialization (opt-in) ---
        self.enable_SA = bool(sim_settings.get("enable_situation_awareness", False))
        if self.enable_SA and not _ISAC_AVAILABLE:
            print("[ISAC] sklearn/scipy not available; disabling ISAC.", flush=True)
            self.enable_SA = False

        if self.enable_SA:
            self.isac_min_power = float(sim_settings.get("isac_min_power", self.isac_min_power))
            self.isac_eps_cluster = float(sim_settings.get("isac_eps_cluster", self.isac_eps_cluster))
            self.isac_min_displacement = float(
                sim_settings.get("isac_min_displacement", self.isac_min_displacement)
            )
            self.beamwidth_deg = float(sim_settings.get("isac_beamwidth_deg", self.beamwidth_deg))
            self.isac_max_depth = int(sim_settings.get("isac_max_depth", self.isac_max_depth))
            self.isac_diffuse_reflection = bool(
                sim_settings.get("isac_diffuse_reflection", self.isac_diffuse_reflection)
            )
            self.isac_static_clutter_scattering = float(
                sim_settings.get(
                    "isac_static_clutter_scattering",
                    self.isac_static_clutter_scattering,
                )
            )
            self.isac_samples_per_src = int(
                sim_settings.get("isac_samples_per_src", self.isac_samples_per_src)
            )
            self.isac_single_bounce_only = bool(
                sim_settings.get("isac_single_bounce_only", self.isac_single_bounce_only)
            )
            self.isac_phase_center_offset = float(
                sim_settings.get("isac_phase_center_offset", self.isac_phase_center_offset)
            )
            self.isac_mti_warmup_frames = int(
                sim_settings.get("isac_mti_warmup_frames", self.isac_mti_warmup_frames)
            )
            self.isac_mti_dist_thresh = float(
                sim_settings.get(
                    "isac_mti_dist_thresh",
                    sim_settings.get("isac_mti_dist_threshold", self.isac_mti_dist_thresh),
                )
            )
            self.isac_tracker_min_age = int(
                sim_settings.get("isac_tracker_min_age", self.isac_tracker_min_age)
            )
            self.isac_dbscan_min_samples = int(
                sim_settings.get("isac_dbscan_min_samples", self.isac_dbscan_min_samples)
            )
            self.isac_enable_motion_track_fallback = bool(
                sim_settings.get(
                    "isac_enable_motion_track_fallback",
                    self.isac_enable_motion_track_fallback,
                )
            )
            self.isac_track_match_radius = float(
                sim_settings.get("isac_track_match_radius", self.isac_track_match_radius)
            )
            self.rx_type_path = sim_settings.get("rx_type_path", None)

            # Pre-build the radar (iso, VH) arrays with comm dimensions for MIMO shape parity.
            self._radar_tx_array = PlanarArray(
                num_rows=self._comm_tx_num_rows,
                num_cols=self._comm_tx_num_cols,
                vertical_spacing=self._comm_tx_v_spacing,
                horizontal_spacing=self._comm_tx_h_spacing,
                pattern="iso",
                polarization="VH",
            )
            self._radar_rx_array = PlanarArray(
                num_rows=self._comm_rx_num_rows,
                num_cols=self._comm_rx_num_cols,
                vertical_spacing=self._comm_tx_v_spacing,
                horizontal_spacing=self._comm_tx_h_spacing,
                pattern="iso",
                polarization="VH",
            )

            # Calibrate detection thresholds from the RX-mesh PLY if provided.
            calib_ply = self.rx_type_path or sim_settings.get("rx_mesh", None)
            if calib_ply and os.path.isfile(calib_ply) and calib_ply.lower().endswith(".ply"):
                try:
                    rx_mesh_z = None
                    if rx_locations:
                        rx_mesh_z = float(rx_locations[0][2]) - self.rx_object_z_offset
                    self.calib = calibrate_detect_objects(calib_ply, obj_z_override=rx_mesh_z)
                    # User overrides take precedence over bbox-derived defaults
                    self.calib["min_power"] = self.isac_min_power
                    self.calib["eps_cluster"] = self.isac_eps_cluster
                    if "isac_z_min" in sim_settings:
                        self.calib["z_min"] = float(sim_settings["isac_z_min"])
                    if "isac_z_max" in sim_settings:
                        self.calib["z_max"] = float(sim_settings["isac_z_max"])
                except Exception as exc:
                    print(f"[ISAC] calibrate_detect_objects failed: {exc}; using defaults",
                          flush=True)
                    self.calib = {"eps_cluster": self.isac_eps_cluster,
                                  "z_min": 0.0, "z_max": 3.0,
                                  "min_power": self.isac_min_power,
                                  "bbox_size": np.array([1.0, 1.0, 1.0])}
            else:
                self.calib = {"eps_cluster": self.isac_eps_cluster,
                              "z_min": 0.0, "z_max": 3.0,
                              "min_power": self.isac_min_power,
                              "bbox_size": np.array([1.0, 1.0, 1.0])}

            dt = float(sim_settings.get("rx_update_interval", 0.1))
            Q_val = float(sim_settings.get("isac_tracker_q", 0.5))
            R_val = float(sim_settings.get("isac_tracker_r", 0.05))
            max_missed = int(sim_settings.get("isac_tracker_max_missed", 2))
            max_tracks = int(
                sim_settings.get(
                    "isac_tracker_max_tracks",
                    max(10, len(self.receivers)),
                )
            )
            self.mti_filter = CloudMTI(dist_thresh=self.isac_mti_dist_thresh)
            self.tracker = Tracker(
                max_missed=max_missed,
                min_age_for_output=self.isac_tracker_min_age,
                min_displacement=self.isac_min_displacement,
                max_tracks=max_tracks,
                dt=dt,
                Q_val=Q_val,
                R_val=R_val,
            )
            self._beam_epoch = 0
            self._beam_active = False
            self.detection_history = []
            self.raw_detection_history = []
            self.beam_history = []
            self._last_sensing_rx_positions = {
                name: np.asarray(pos, dtype=float)
                for name, pos in zip(rx_names, rx_locations)
            }
            if self.isac_mti_warmup_frames > 0:
                self.prime_mti_background(self.isac_mti_warmup_frames)

        self._perf_add("python_initialize_seconds", time.perf_counter() - init_start)

    def update_position(self, rx_name: str, position: list[float]):
        if position is None:
            return
        prev = self.rx_prev_positions.get(rx_name)
        if prev is not None:
            dp = [position[i] - prev[i] for i in range(3)]
            norm_xy = (dp[0]**2 + dp[1]**2) ** 0.5
            if norm_xy > 1e-6:
                old_direction = self.rx_last_directions.get(rx_name)
                if old_direction is not None:
                    old_dx, old_dy = float(old_direction[0]), float(old_direction[1])
                    old_norm = (old_dx * old_dx + old_dy * old_dy) ** 0.5
                    if old_norm > 1e-6:
                        self.rx_direction_dots[rx_name] = (
                            (old_dx / old_norm) * (dp[0] / norm_xy)
                            + (old_dy / old_norm) * (dp[1] / norm_xy)
                        )
                self.rx_last_directions[rx_name] = dp
        self.rx_prev_positions[rx_name] = list(position)
        if rx_name in self.scene.receivers:
            self.scene.receivers[rx_name].position = position
        obj_name = f"rx_obj_{rx_name}"
        if obj_name in self.scene.objects:
            self.scene.objects[obj_name].position = [
                position[0], position[1], position[2] - self.rx_object_z_offset
            ]

    def add_receiver(self, rx_name: str, position: list[float], rx_id=None) -> bool:
        if self.scene is None:
            return False
        if rx_id is None:
            rx_id = len(self.receivers)
        if rx_name in self.scene.receivers:
            self.remove_receiver(rx_name)
        self.receivers[rx_name] = rx_id
        self.scene.add(Receiver(name=rx_name, position=position, color=[0, 1, 0], display_radius=0.5))
        return True

    def remove_receiver(self, rx_name: str) -> bool:
        if self.scene is None:
            return False
        removed = False
        if rx_name in self.receivers:
            self.receivers.pop(rx_name, None)
            removed = True
        if self.scene.get(rx_name) is not None:
            self.scene.remove(rx_name)
            removed = True
        return removed

    def clear_virtual_receivers(self, prefix="vrx_"):
        for name in [n for n in list(self.receivers.keys()) if n.startswith(prefix)]:
            self.remove_receiver(name)

    def _hide_rx_mesh_objects(self):
        snapshot = {}
        for name in list(getattr(self, "receivers", {}).keys()):
            obj_name = f"rx_obj_{name}"
            if obj_name in self.scene.objects:
                snapshot[obj_name] = (
                    np.asarray(self.scene.objects[obj_name].position).ravel().tolist()
                )
        if snapshot:
            self.scene.edit(remove=list(snapshot.keys()))
        return snapshot

    def _restore_rx_mesh_objects(self, snapshot):
        if not snapshot:
            return
        rx_mesh_path = getattr(self, "_rx_mesh_path", None)
        if not rx_mesh_path:
            return
        self.scene.edit(add=[
            SceneObject(name=obj_name, fname=rx_mesh_path, radio_material=self.rx_material)
            for obj_name in snapshot
        ])
        for obj_name, position in snapshot.items():
            self.scene.objects[obj_name].position = position

    @staticmethod
    def round_point(point, ndigits=4):
        return tuple(round(float(v), ndigits) for v in point)

    @staticmethod
    def _format_point(point):
        return list(SionnaRT.round_point(np.asarray(point, dtype=float).tolist()))

    @staticmethod
    def _to_numpy(value):
        if hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value)

    def _node_id(self, node_type: str, index: int) -> int:
        if node_type == "tx":
            names = list(self.scene.transmitters.keys())
            return int(self.transmitters[names[index]])
        if node_type == "rx":
            names = list(self.scene.receivers.keys())
            return int(self.receivers[names[index]])
        raise ValueError(f"Invalid node type: {node_type}")

    def _cache_validity_displacement_threshold(self, distance_m: float) -> float:
        spatial = self.coherence_alpha * max(distance_m, 0.0) * 0.886 / float(self.coherence_tx_num_cols)
        return max(float(self.virtual_position_min_separation), spatial)

    def _nearest_transmitter_distance(self, point) -> float | None:
        if self.scene is None or not getattr(self.scene, "transmitters", None):
            return None
        point = np.asarray(point, dtype=float)
        dists = []
        for tx in self.scene.transmitters.values():
            try:
                dists.append(float(np.linalg.norm(np.asarray(tx.position, dtype=float) - point)))
            except Exception:
                continue
        return min(dists) if dists else None

    def _stable_direction_unit(self, rx_name: str):
        direction = self.rx_last_directions.get(rx_name)
        if direction is None:
            return None

        dx, dy = float(direction[0]), float(direction[1])
        norm = (dx * dx + dy * dy) ** 0.5
        if norm <= 1e-6:
            return None

        direction_dot = self.rx_direction_dots.get(rx_name)
        if direction_dot is None:
            return None
        if direction_dot < self.adaptive_future_direction_dot_threshold:
            return None

        return np.array([dx / norm, dy / norm, 0.0])

    def _adaptive_virtual_rx_positions(self, rx_name: str, rx_pos):
        speed = max(float(self.rx_speeds.get(rx_name, 0.0)), 0.0)
        if speed <= 1e-6:
            return []

        link_dist = self._nearest_transmitter_distance(rx_pos) or 0.0
        validity_dist = self._cache_validity_displacement_threshold(link_dist)
        time_to_expiry = validity_dist / speed if speed > 1e-6 else float("inf")
        if time_to_expiry >= self.adaptive_future_min_benefit_seconds:
            return []

        dir_unit = self._stable_direction_unit(rx_name)
        if dir_unit is None:
            return []

        step_dist = validity_dist * self.cache_threshold_buffer
        horizon_distance = speed * max(self.adaptive_future_horizon_seconds, 0.0)
        steps = int(np.ceil(horizon_distance / step_dist)) if step_dist > 0.0 else 0
        steps = min(max(steps, 0), max(self.adaptive_future_max_steps, 0))
        if steps <= 0:
            return []

        rx_pos_arr = np.asarray(rx_pos, dtype=float)
        return [
            self._format_point((rx_pos_arr + dir_unit * (step_dist * k)).tolist())
            for k in range(1, steps + 1)
        ]

    def calculate_propagation(self):
        if self.scene is None:
            return []

        tx_names = list(self.scene.transmitters.keys())
        rx_names = list(self.scene.receivers.keys())
        if not tx_names or not rx_names:
            return []

        self.propagation_calculation_calls += 1
        self.propagation_calculation_receiver_count += len(rx_names)

        path_start = time.perf_counter()
        paths = self.path_solver(
            scene=self.scene,
            max_depth=self.max_depth,
            los=True,
            specular_reflection=True,
            diffuse_reflection=self.diffuse_reflection,
            refraction=True,
            synthetic_array=True,
            seed=41,
        )
        self._perf_add("python_path_solve_seconds", time.perf_counter() - path_start)

        tau = self._to_numpy(paths.tau)
        cfr_start = time.perf_counter()
        h_raw = paths.cfr(
            frequencies=self.export_frequencies,
            sampling_frequency=1.0,
            num_time_steps=1,
            reverse_direction=False,
            normalize=False,
            out_type="numpy",
        )
        h_raw = np.asarray(h_raw)
        self._perf_add("python_cfr_seconds", time.perf_counter() - cfr_start)

        export_start = time.perf_counter()
        interactions = self._to_numpy(paths.interactions)
        valid = self._to_numpy(paths.valid)

        num_rx = len(rx_names)
        num_tx = len(tx_names)
        num_paths = tau.shape[-1] if tau.ndim > 0 else 0

        if valid.ndim == 1 and num_rx > 0 and num_tx > 0 and num_paths > 0:
            valid = valid.reshape(num_rx, num_tx, num_paths)
        if interactions.ndim == 1 and num_rx > 0 and num_tx > 0 and num_paths > 0:
            depth = interactions.size // (num_rx * num_tx * num_paths)
            interactions = interactions.reshape(depth, num_rx, num_tx, num_paths)

        if tau.ndim != 3:
            raise ValueError(f"Unexpected synthetic-array tau shape: {tau.shape}")
        if h_raw.ndim != 6:
            raise ValueError(f"Unexpected synthetic-array CFR shape: {h_raw.shape}")
        if interactions.ndim != 4:
            raise ValueError(f"Unexpected synthetic-array interactions shape: {interactions.shape}")

        records = []
        for tx_idx, _ in enumerate(tx_names):
            for rx_idx, _ in enumerate(rx_names):
                link_tau = tau[rx_idx, tx_idx, :]
                h = h_raw[rx_idx, :, tx_idx, :, 0, :]

                valid_tau = link_tau[link_tau >= 0]
                link_delay = int(round(float(np.min(valid_tau) * 1e9), 0)) if valid_tau.size else 0

                # Export two views of the same CFR:
                #  - a legacy scalar frequency-selective response used by older
                #    C++ paths;
                #  - the normalized element-level MIMO CFR used to build the
                #    NR spectrumChannelMatrix.  The absolute wideband loss
                #    stays in path_loss so it is not counted twice.
                freq_power = np.mean(np.abs(h) ** 2, axis=tuple(range(h.ndim - 1)))
                power = float(np.mean(freq_power))
                if power > 0.0:
                    path_loss = float(-10.0 * np.log10(power))
                    normalized_power = np.maximum(freq_power / power, 0.0)
                    h_normalized = np.sqrt(normalized_power).astype(np.complex128)
                    h_mimo_normalized = (h / np.sqrt(power)).astype(np.complex128)
                else:
                    path_loss = 200.0
                    h_normalized = np.zeros(int(self.export_num_subcarriers), dtype=np.complex128)
                    h_mimo_normalized = np.zeros_like(h, dtype=np.complex128)

                if h_mimo_normalized.ndim != 3:
                    raise ValueError(f"Unexpected per-link MIMO CFR shape: {h_mimo_normalized.shape}")
                mimo_rx_elems = int(h_mimo_normalized.shape[0])
                mimo_tx_elems = int(h_mimo_normalized.shape[1])
                mimo_num_subcarriers = int(h_mimo_normalized.shape[2])

                # ns-3 MatrixArray uses page-major, column-major storage:
                # rb -> tx element -> rx element.
                h_mimo_flat = np.ascontiguousarray(
                    np.transpose(h_mimo_normalized, (2, 1, 0))
                ).ravel()

                link_interactions = interactions[:, rx_idx, tx_idx, :]
                link_valid = valid[rx_idx, tx_idx, :]
                is_none = np.all(link_interactions == InteractionType.NONE, axis=0)
                los_exists = bool(np.any(np.logical_and(link_valid, is_none)))

                records.append({
                    "src_id": self._node_id("tx", tx_idx),
                    "dst_id": self._node_id("rx", rx_idx),
                    "tx_position": self._format_point(self.scene.transmitters[tx_names[tx_idx]].position.numpy().ravel().tolist()),
                    "rx_position": self._format_point(self.scene.receivers[rx_names[rx_idx]].position.numpy().ravel().tolist()),
                    "delay": link_delay,
                    "path_loss": path_loss,
                    "power": power,
                    "real": np.ascontiguousarray(np.real(h_normalized).ravel(), dtype=np.float64),
                    "imag": np.ascontiguousarray(np.imag(h_normalized).ravel(), dtype=np.float64),
                    "num_subcarriers": int(self.export_num_subcarriers),
                    "mimo_real": np.ascontiguousarray(np.real(h_mimo_flat), dtype=np.float64),
                    "mimo_imag": np.ascontiguousarray(np.imag(h_mimo_flat), dtype=np.float64),
                    "mimo_rx_elems": mimo_rx_elems,
                    "mimo_tx_elems": mimo_tx_elems,
                    "mimo_num_subcarriers": mimo_num_subcarriers,
                    "subcarrier_frequencies": np.ascontiguousarray(
                        self.export_frequencies, dtype=np.float64
                    ),
                    "los_exist": los_exists,
                })

        self._perf_add("python_export_seconds", time.perf_counter() - export_start)
        return records

    # ------------------- START of ISAC pipeline -------------------
    def _install_radar_receivers(self):
        """Add receiver named 'radar_rx_<tx>' colocated at each TX. Returns created names."""
        created = []
        if self.scene is None:
            return created
        for tx_name, tx in list(self.scene.transmitters.items()):
            rname = f"radar_rx_{tx_name}"
            try:
                if rname in self.scene.receivers:
                    self.scene.remove(rname)
                pos = np.asarray(tx.position).ravel().tolist()[:3]
                self.scene.add(Receiver(name=rname, position=pos,
                                        color=[1, 0, 1], display_radius=0.2))
                created.append(rname)
            except Exception as exc:
                print(f"[ISAC] failed to install radar RX {rname}: {exc}", flush=True)
        return created

    def _remove_radar_receivers(self, names):
        for n in names:
            try:
                if n in self.scene.receivers:
                    self.scene.remove(n)
            except Exception:
                pass

    def _run_sensing_solve(self):
        """Run per-TX monostatic sensing solves and return confirmed track positions.

        For each registered transmitter a separate path-solve is performed with
        all other TXs (and their radar RXs) temporarily removed, so that Sionna
        averages tau/angles only over the single monostatic TX-RX pair. ERPs from
        all TXs are collected into a shared pool before MTI filtering and DBSCAN
        clustering, giving full-scene coverage when multiple gNBs are deployed.
        Comm receivers are removed for the duration of all solves.
        """
        # Remove comm receivers once for all solves
        comm_rx_snapshot = {}
        for name in list(self.receivers.keys()):
            if name in self.scene.receivers:
                pos = np.asarray(self.scene.receivers[name].position).ravel().tolist()
                self.scene.remove(name)
                comm_rx_snapshot[name] = pos

        # Snapshot all radar RX positions (installed before this call)
        all_radar_rx_snapshot = {}
        for name in list(self.scene.receivers.keys()):
            if name.startswith("radar_rx_"):
                all_radar_rx_snapshot[name] = (
                    np.asarray(self.scene.receivers[name].position).ravel().tolist()
                )

        # Snapshot all TX positions and power
        tx_names = list(self.transmitters.keys())
        tx_snapshot = {}
        for name in tx_names:
            if name in self.scene.transmitters:
                tx_obj = self.scene.transmitters[name]
                tx_snapshot[name] = {
                    "pos": np.asarray(tx_obj.position).ravel().tolist(),
                    "pwr": float(np.asarray(tx_obj.power_dbm).ravel()[0]),
                }

        clutter_scattering = float(getattr(self, "isac_static_clutter_scattering", 0.0))
        orig_scattering = {}
        if clutter_scattering > 0.0:
            for obj_name, obj in list(self.scene.objects.items()):
                if obj_name.startswith("rx_obj_"):
                    continue
                try:
                    mat = obj.radio_material
                    if mat is None:
                        continue
                    orig_s = float(np.asarray(mat.scattering_coefficient).ravel()[0])
                    if orig_s < clutter_scattering:
                        orig_scattering[obj_name] = orig_s
                        mat.scattering_coefficient = clutter_scattering
                except Exception:
                    pass

        try:
            sensing_depth = max(int(self.isac_max_depth), 1)
            all_erp = []
            all_pwr = []

            for radar_tx_name in tx_names:
                if radar_tx_name not in tx_snapshot:
                    continue
                gnb_pos = tx_snapshot[radar_tx_name]["pos"][:3]

                # Remove all TXs except current radar TX
                removed_txs = {}
                for name, info in tx_snapshot.items():
                    if name != radar_tx_name and name in self.scene.transmitters:
                        self.scene.remove(name)
                        removed_txs[name] = info

                # Remove all radar RXs except the one colocated with current TX
                keep_rx = f"radar_rx_{radar_tx_name}"
                removed_rxs = {}
                for name, pos in all_radar_rx_snapshot.items():
                    if name != keep_rx and name in self.scene.receivers:
                        self.scene.remove(name)
                        removed_rxs[name] = pos

                try:
                    paths = self.path_solver(
                        scene=self.scene,
                        samples_per_src=int(self.isac_samples_per_src),
                        max_depth=sensing_depth,
                        los=True,
                        specular_reflection=True,
                        diffuse_reflection=bool(self.isac_diffuse_reflection),
                        edge_diffraction=False,
                        refraction=False,
                        synthetic_array=True,
                        seed=42,
                    )
                    erp, pwr = _extract_erps(
                        paths, gnb_pos,
                        min_power=self.calib["min_power"],
                        z_min=self.calib["z_min"],
                        z_max=self.calib["z_max"],
                        single_bounce_only=self.isac_single_bounce_only,
                        phase_center_offset=self.isac_phase_center_offset,
                    )
                    if len(erp):
                        all_erp.append(erp)
                        all_pwr.append(pwr)
                except Exception as exc:
                    print(f"[ISAC] sensing solve for {radar_tx_name} failed: {exc}", flush=True)
                finally:
                    # Restore TXs and radar RXs removed for this solve
                    for name, info in removed_txs.items():
                        self.scene.add(Transmitter(name=name, position=info["pos"],
                                                   power_dbm=info["pwr"], color=[1, 0, 0]))
                    for name, pos in removed_rxs.items():
                        self.scene.add(Receiver(name=name, position=pos,
                                                color=[1, 0, 1], display_radius=0.2))
        finally:
            for obj_name, orig_s in orig_scattering.items():
                try:
                    self.scene.objects[obj_name].radio_material.scattering_coefficient = orig_s
                except Exception:
                    pass
            for name, pos in comm_rx_snapshot.items():
                self.scene.add(Receiver(name=name, position=pos,
                                        color=[0, 1, 0], display_radius=0.5))

        self._perf_add("python_isac_sensing_frames", 1.0)
        if not all_erp:
            self._perf_add("python_isac_empty_erp_frames", 1.0)
            return [], self.tracker.process_frame([])

        ERP = np.vstack(all_erp)
        PWR = np.concatenate(all_pwr)
        if self.isac_detection_roi is not None and len(ERP):
            x_min, x_max, y_min, y_max = self.isac_detection_roi
            roi_mask = (
                (ERP[:, 0] >= x_min) & (ERP[:, 0] <= x_max) &
                (ERP[:, 1] >= y_min) & (ERP[:, 1] <= y_max)
            )
            self._perf_add("python_isac_roi_rejected_points",
                           float(len(ERP) - int(np.count_nonzero(roi_mask))))
            ERP = ERP[roi_mask]
            PWR = PWR[roi_mask]
        self._perf_add("python_isac_raw_erp_points", float(len(ERP)))

        if self.mti_filter is not None:
            ERP, PWR = self.mti_filter.filter(ERP, PWR)
        self._perf_add("python_isac_post_mti_points", float(len(ERP)))

        if len(ERP) == 0:
            self._perf_add("python_isac_empty_post_mti_frames", 1.0)
            return [], self.tracker.process_frame([])

        labels = DBSCAN(
            eps=self.calib["eps_cluster"],
            min_samples=max(1, self.isac_dbscan_min_samples),
        ).fit(ERP).labels_
        ids = sorted(set(labels) - {-1})
        raw = []
        for k in ids:
            m = labels == k
            best_idx = np.argmax(PWR[m])
            raw.append({"position": ERP[m][best_idx], "power": float(PWR[m].sum())})
        raw.sort(key=lambda x: x["power"], reverse=True)
        raw_detections = [{"id": i, "position": r["position"], "power": r["power"]}
                          for i, r in enumerate(raw)]
        self._perf_add("python_isac_raw_clusters", float(len(raw_detections)))
        max_tracker_inputs = max(4, len(getattr(self, "receivers", {})))
        tracker_detections = raw_detections[:max_tracker_inputs]
        tracks = self.tracker.process_frame(tracker_detections)
        self._perf_add("python_isac_confirmed_tracks", float(len(tracks)))
        return raw_detections, tracks

    def prime_mti_background(self, n_frames: int = 3) -> None:
        """Seed the MTI background with static-clutter-only frames.

        Temporarily removes all robot mesh objects (rx_obj_*) from the scene,
        runs ``n_frames`` sensing path-solves, and feeds the resulting ERPs into
        ``mti_filter.prime()``.  Robot meshes are restored afterward.

        Call this once before the simulation loop when the scene contains static
        reflectors (walls, racks, furniture) that would otherwise contaminate
        the MTI background if it were seeded from a live frame that also
        contains robot echoes.
        """
        if not self.enable_SA or self.mti_filter is None:
            return

        # Stash and remove all robot mesh objects
        rx_obj_snapshot = self._hide_rx_mesh_objects()

        # Remove comm receivers so the solve is radar-only
        comm_rx_snapshot = {}
        for name in list(self.receivers.keys()):
            if name in self.scene.receivers:
                pos = np.asarray(self.scene.receivers[name].position).ravel().tolist()
                self.scene.remove(name)
                comm_rx_snapshot[name] = pos

        # Remove extra TXs beyond the first (radar TX) — same reason as _run_sensing_solve
        radar_tx_name = list(self.transmitters.keys())[0] if self.transmitters else None
        extra_tx_snapshot = {}
        for name in list(self.transmitters.keys()):
            if name == radar_tx_name:
                continue
            if name in self.scene.transmitters:
                pos = np.asarray(self.scene.transmitters[name].position).ravel().tolist()
                extra_tx_snapshot[name] = pos
                self.scene.remove(name)

        # Enable diffuse scattering on all static scene objects so they produce
        # clutter ERPs even when no robot meshes are present. This is needed in
        # scenes (e.g. warehouse) where built-in materials have scattering_coefficient=0.
        _clutter_scattering = 0.3
        _orig_scattering = {}
        for obj_name, obj in list(self.scene.objects.items()):
            try:
                mat = obj.radio_material
                if mat is not None:
                    orig_s = float(np.asarray(mat.scattering_coefficient).ravel()[0])
                    if orig_s < _clutter_scattering:
                        _orig_scattering[obj_name] = orig_s
                        mat.scattering_coefficient = _clutter_scattering
            except Exception:
                pass

        # Add radar RX colocated at gNB
        gnb_pos = self.radar_tx_pos
        if gnb_pos is None and self.scene.transmitters:
            gnb_pos = np.asarray(
                next(iter(self.scene.transmitters.values())).position
            ).ravel().tolist()[:3]
        rname = "_mti_prime_rx"
        try:
            if rname in self.scene.receivers:
                self.scene.remove(rname)
            self.scene.add(Receiver(name=rname, position=gnb_pos,
                                    color=[0.5, 0, 0.5], display_radius=0.1))
        except Exception as exc:
            print(f"[MTI-prime] could not add radar RX: {exc}", flush=True)

        saved_tx = self.scene.tx_array
        saved_rx = self.scene.rx_array
        self.scene.tx_array = self._radar_tx_array
        self.scene.rx_array = self._radar_rx_array

        try:
            for i in range(n_frames):
                try:
                    paths = self.path_solver(
                        scene=self.scene,
                        samples_per_src=int(self.isac_samples_per_src),
                        max_depth=max(int(self.isac_max_depth), 1),
                        los=True, specular_reflection=True,
                        diffuse_reflection=bool(self.isac_diffuse_reflection),
                        edge_diffraction=False, refraction=False,
                        synthetic_array=True, seed=42 + i,
                    )
                    a       = np.array(paths.a)
                    tau     = np.array(paths.tau)
                    theta_r = np.array(paths.theta_r)
                    phi_r   = np.array(paths.phi_r)
                    powers  = np.sum(np.abs(a)**2,    axis=tuple(range(a.ndim-1)))
                    tau_1d  = np.mean(tau,             axis=tuple(range(tau.ndim-1)))
                    th_1d   = np.mean(theta_r,         axis=tuple(range(theta_r.ndim-1)))
                    ph_1d   = np.mean(phi_r,           axis=tuple(range(phi_r.ndim-1)))

                    valid = tau_1d > 1e-9
                    L = SPEED_OF_LIGHT * tau_1d[valid]
                    scaling = np.maximum(L / 10.0, 1.0) ** 1.1
                    dyn_thr = self.calib["min_power"] / scaling
                    vp  = powers[valid] > dyn_thr
                    L_f = L[vp]; th_f = th_1d[valid][vp]; ph_f = ph_1d[valid][vp]
                    d_dir = np.stack([np.sin(th_f)*np.cos(ph_f),
                                      np.sin(th_f)*np.sin(ph_f),
                                      np.cos(th_f)], axis=-1)
                    ERP = np.array(gnb_pos) + (L_f[:, None] / 2.0) * d_dir
                    vz  = ((ERP[:, 2] >= self.calib["z_min"]) &
                           (ERP[:, 2] <= self.calib["z_max"]))
                    clutter_erps = ERP[vz]
                    if len(clutter_erps):
                        self.mti_filter.prime(clutter_erps)
                    print(f"[MTI-prime] frame {i+1}/{n_frames}: "
                          f"{len(clutter_erps)} clutter ERPs added to background",
                          flush=True)
                except Exception as exc:
                    print(f"[MTI-prime] frame {i+1} failed: {exc}", flush=True)
        finally:
            self.scene.tx_array = saved_tx
            self.scene.rx_array = saved_rx
            try:
                if rname in self.scene.receivers:
                    self.scene.remove(rname)
            except Exception:
                pass
            for name, pos in comm_rx_snapshot.items():
                self.scene.add(Receiver(name=name, position=pos,
                                        color=[0, 1, 0], display_radius=0.5))
            # Restore extra TXs
            for name, pos in extra_tx_snapshot.items():
                try:
                    pwr = 46.0
                    if self.scene.transmitters:
                        pwr = next(iter(self.scene.transmitters.values())).power_dbm
                    self.scene.add(Transmitter(name=name, position=pos,
                                               power_dbm=pwr, color=[1, 0, 0]))
                except Exception:
                    pass
            # Restore original scattering coefficients
            for obj_name, orig_s in _orig_scattering.items():
                try:
                    self.scene.objects[obj_name].radio_material.scattering_coefficient = orig_s
                except Exception:
                    pass
            # Restore robot mesh objects
            self._restore_rx_mesh_objects(rx_obj_snapshot)

    def _build_multilobe_tx_array(self, tracks):
        gnb_pos = self.radar_tx_pos
        bf = BeamformingPattern.from_positions(
            radar_position=gnb_pos,
            detected_positions=self._beam_target_positions(tracks),
            width_deg=self.beamwidth_deg,
            polarization=self._comm_tx_polarization,
        )
        polarization = self._comm_tx_polarization
        return bf.to_planar_array(
            num_rows=self._comm_tx_num_rows,
            num_cols=self._comm_tx_num_cols,
            spacing=self._comm_tx_h_spacing,
            polarization=polarization,
        )

    def _beam_target_positions(self, tracks):
        """Convert floor-level robot detections into antenna-height beam targets."""
        targets = []
        for pos in tracks:
            p = np.asarray(pos, dtype=float).copy()
            if self._beam_target_z is not None:
                p[2] = self._beam_target_z
            targets.append(p)
        return targets

    def _motion_track_fallback(self):
        """Return moving receiver antenna positions when ray-traced MTI has no confirmed track."""
        if not self.isac_enable_motion_track_fallback:
            return []
        tracks = []
        current_positions = {}
        for name in list(getattr(self, "receivers", {}).keys()):
            if name.startswith(("vrx_", "ns3vrx_", "radar_rx_")):
                continue
            if f"rx_obj_{name}" not in self.scene.objects:
                continue
            if name not in self.scene.receivers:
                continue
            pos = np.asarray(self.scene.receivers[name].position).ravel()[:3].astype(float)
            current_positions[name] = pos
            prev = self._last_sensing_rx_positions.get(name)
            if prev is None:
                continue
            if np.linalg.norm(pos[:2] - prev[:2]) >= self.isac_min_displacement:
                tracks.append(pos.copy())
        self._last_sensing_rx_positions.update(current_positions)
        if tracks:
            self._perf_add("python_isac_motion_fallback_tracks", float(len(tracks)))
        return tracks

    def _robot_receiver_positions(self):
        """Return current antenna positions for robot receivers only."""
        positions = {}
        for name in list(getattr(self, "receivers", {}).keys()):
            if name.startswith(("vrx_", "ns3vrx_", "radar_rx_")):
                continue
            if "robot" not in name.lower():
                continue
            if name not in self.scene.receivers:
                continue
            positions[name] = (
                np.asarray(self.scene.receivers[name].position)
                .ravel()[:3]
                .astype(float)
            )
        return positions

    def _filter_tracks_to_robot_receivers(self, tracks):
        """Associate sensing tracks to robot UEs and drop stale/static clutter.

        Sensing ERPs are often near the robot floor mesh while the communication
        receiver is at antenna height. Match in XY, then steer to the current
        robot antenna position. This prevents a persistent static-clutter track
        from consuming a communication beam after robots have left the area.
        """
        if not tracks:
            return []

        robot_positions = self._robot_receiver_positions()
        if not robot_positions:
            return list(tracks)

        matched = []
        used = set()
        rejected = 0
        radius = float(self.isac_track_match_radius)
        for track in tracks:
            t = np.asarray(track, dtype=float).ravel()[:3]
            best_name = None
            best_dist = None
            for name, pos in robot_positions.items():
                if name in used:
                    continue
                dist = float(np.linalg.norm(t[:2] - pos[:2]))
                if best_dist is None or dist < best_dist:
                    best_name = name
                    best_dist = dist
            if best_name is not None and best_dist <= radius:
                matched.append(robot_positions[best_name].copy())
                used.add(best_name)
            else:
                rejected += 1

        if matched:
            self._perf_add("python_isac_robot_matched_tracks", float(len(matched)))
        if rejected:
            self._perf_add("python_isac_rejected_unmatched_tracks", float(rejected))
        return matched

    def _beam_records_for_tracks(self, current_time, tracks):
        gnb_pos = np.asarray(self.radar_tx_pos, dtype=float)
        records = []
        for idx, pos in enumerate(self._beam_target_positions(tracks)):
            p = np.asarray(pos, dtype=float)
            vec = p - gnb_pos
            r = float(np.linalg.norm(vec))
            if r < 1e-9:
                continue
            records.append({
                "time": float(current_time),
                "beam_index": int(idx),
                "active": True,
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
                "theta_deg": float(np.degrees(np.arccos(vec[2] / r))),
                "phi_deg": float(np.degrees(np.arctan2(vec[1], vec[0]))),
                "beamwidth_deg": float(self.beamwidth_deg),
            })
        return records

    def _beam_epoch_perturb(self, tx_pos):
        """Deterministic offset > m_minDelta (0.1 m) tied to _beam_epoch.

        Forces SionnaPropagationCache::IsEntryValid to invalidate cached
        beamformed records when the beam set changes between snapshots.
        """
        eps = 0.15
        ex = eps * ((self._beam_epoch % 7) - 3)
        ey = eps * (((self._beam_epoch // 7) % 7) - 3)
        return [float(tx_pos[0]) + ex, float(tx_pos[1]) + ey, float(tx_pos[2])]

    def get_detected_objects(self, since_time: float = -1.0):
        """Return logged detections at or after ``since_time``; -1 returns all."""
        if since_time < 0.0:
            return list(self.detection_history)
        return [r for r in self.detection_history if r["time"] >= since_time]

    def get_raw_detected_objects(self, since_time: float = -1.0):
        """Return logged raw DBSCAN detections at or after ``since_time``; -1 returns all."""
        if since_time < 0.0:
            return list(self.raw_detection_history)
        return [r for r in self.raw_detection_history if r["time"] >= since_time]

    def get_beam_history(self, since_time: float = -1.0):
        """Return sensing-assisted communication beam lobes at or after since_time."""
        if not hasattr(self, "beam_history"):
            return []
        if since_time < 0.0:
            return list(self.beam_history)
        return [r for r in self.beam_history if r["time"] >= since_time]
    # ------------------- END of ISAC pipeline -------------------

    def perform_calculation(self, _current_time: float):
        perform_start = time.perf_counter()

        # --- ISAC pre-pass: sensing + (optional) beamforming array swap ---
        comm_tx_array_backup = None
        beam_active_this_call = False
        if self.enable_SA:
            try:
                isac_start = time.perf_counter()
                comm_tx_array_backup = self.scene.tx_array
                comm_rx_array_backup = self.scene.rx_array

                # Sensing pass: swap to radar arrays and add radar RXs at TXs
                self.scene.tx_array = self._radar_tx_array
                self.scene.rx_array = self._radar_rx_array
                radar_names = self._install_radar_receivers()
                try:
                    raw_detections, tracks = self._run_sensing_solve()
                finally:
                    self._remove_radar_receivers(radar_names)
                    self.scene.rx_array = comm_rx_array_backup
                tracks = self._filter_tracks_to_robot_receivers(tracks)
                if not tracks:
                    tracks = self._motion_track_fallback()

                # Log detection history. Powers come from raw_detections (pre-Kalman) because
                # the Kalman tracker returns positions only, breaking direct track↔power mapping.
                # We store one power per confirmed track using the highest-power raw cluster, which
                # is the physically meaningful quantity for array-gain comparison across array sizes.
                raw_powers = [float(r.get("power", 0.0)) for r in raw_detections if r.get("power", 0.0) > 0]
                # Assign best available power to each confirmed track (sorted by power already)
                track_powers = raw_powers[:len(tracks)] if raw_powers else [0.0] * len(tracks)
                self.detection_history.append({
                    "time": float(_current_time),
                    "positions": [pos.tolist() if hasattr(pos, "tolist") else list(pos)
                                  for pos in tracks],
                    "powers": track_powers,
                })
                self.raw_detection_history.append({
                    "time": float(_current_time),
                    "positions": [pos["position"].tolist() if hasattr(pos["position"], "tolist") else list(pos["position"]) for pos in raw_detections],
                })

                if tracks:
                    self._beam_epoch += 1
                    beam_active_this_call = True
                    self.beam_history.extend(
                        self._beam_records_for_tracks(_current_time, tracks)
                    )
                # Always restore the comm TX array — the ISAC multilobe array must
                # NOT be used for communication channel computation.  Applying it
                # bakes ISAC beam gain into the Sionna path_loss, which inflates CQI
                # reports.  The NR scheduler then over-allocates high MCS while the
                # actual channel (under the normal comm beam) cannot sustain it,
                # causing packet loss and degrading delivery vs no-ISAC.
                self.scene.tx_array = comm_tx_array_backup
                self._perf_add("python_isac_pre_seconds", time.perf_counter() - isac_start)
            except Exception as exc:
                import traceback
                print(f"[ISAC] pre-pass failed, falling back to comm: {exc}", flush=True)
                traceback.print_exc()
                if comm_tx_array_backup is not None:
                    self.scene.tx_array = comm_tx_array_backup
                beam_active_this_call = False
        self._beam_active = beam_active_this_call

        # --- Adaptive virtual receivers (unchanged) ---
        self.clear_virtual_receivers(prefix="ns3vrx_")
        created = []
        real_rx_names = [
            name for name in list(self.scene.receivers.keys())
            if not name.startswith(("vrx_", "ns3vrx_", "radar_rx_"))
        ]
        for rx_name in real_rx_names:
            rx_pos = self.scene.receivers[rx_name].position.numpy().ravel().tolist()
            virtual_points = self._adaptive_virtual_rx_positions(rx_name, rx_pos)
            for idx, point in enumerate(virtual_points):
                vrx_name = f"ns3vrx_{rx_name}_{idx}"
                self.add_receiver(vrx_name, point, rx_id=self.receivers.get(rx_name))
                created.append(vrx_name)

        rx_mesh_snapshot = self._hide_rx_mesh_objects()
        try:
            records = self.calculate_propagation()
        finally:
            self._restore_rx_mesh_objects(rx_mesh_snapshot)
            for name in created:
                self.remove_receiver(name)

        self._perf_add("python_perform_calculation_seconds", time.perf_counter() - perform_start)
        return records
