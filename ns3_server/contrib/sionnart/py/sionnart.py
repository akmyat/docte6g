import os
import numpy as np

import mitsuba as mi
for _variant in ("cuda_ad_mono_polarized", "cuda_ad_rgb", "llvm_ad_rgb"):
    if _variant in mi.variants():
        mi.set_variant(_variant)
        break
else:
    raise ImportError(f"No supported Mitsuba variant found. Available variants: {mi.variants()}")

from sionna.rt import load_scene, PlanarArray, Transmitter, Receiver
from sionna.rt import PathSolver, subcarrier_frequencies, InteractionType
from sionna.rt import RadioMaterial, SceneObject
from sionna.phy import SPEED_OF_LIGHT


class SionnaRT:
    def __init__(self):
        self.scene = None
        self.path_solver = PathSolver()
        self.assets_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "../../../../assets")
        )
        self.max_depth = 3
        self.diffuse_reflection = False
        # Cache coherence parameters — must match C++ SionnaPropagationCache attributes.
        # delta = alpha * d * 0.886 / tx_num_cols; step_dist = delta * buffer
        self.coherence_alpha = 0.4
        self.coherence_tx_num_cols = 8
        self.virtual_position_min_separation = 0.5  # floor on step_dist [m]
        self.virtual_position_min_steps = 40         # virtual receivers placed per miss
        self.virtual_position_num_angles = 8         # angles for omnidirectional fallback
        self.cache_threshold_buffer = 1.1            # 10% overlap so windows never gap
        self.propagation_record_mode = "future_records"
        self.rx_speeds = {}
        self.rx_prev_positions = {}
        self.rx_last_directions = {}
        self.propagation_calculation_calls = 0
        self.propagation_calculation_receiver_count = 0

    def initialize(self, sim_settings):
        self.scene = load_scene(
            sim_settings.get("scene", self.assets_dir + "/scenes/free_space/free_space.xml")
        )

        f_c = sim_settings.get("carrier_frequency", 3.5e9)
        num_subcarriers = sim_settings.get("num_subcarriers", 3276)
        subcarrier_spacing = sim_settings.get("subcarrier_spacing", 30000)

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
        pattern = sim_settings.get("pattern", "tr38901")
        polarization = sim_settings.get("polarization", "VH")

        array_kwargs = dict(
            vertical_spacing=v_spacing,
            horizontal_spacing=h_spacing,
            pattern=pattern,
            polarization=polarization,
        )
        self.scene.tx_array = PlanarArray(
            num_rows=sim_settings.get("tx_num_rows", 8),
            num_cols=sim_settings.get("tx_num_cols", 8),
            **array_kwargs,
        )
        self.scene.rx_array = PlanarArray(
            num_rows=sim_settings.get("rx_num_rows", 2),
            num_cols=sim_settings.get("rx_num_cols", 2),
            **array_kwargs,
        )

        tx_names = sim_settings.get("tx_names", [""])
        tx_ids = sim_settings.get("tx_ids", [])
        tx_locations = sim_settings.get("tx_locations", [[0.0, 0.0, 20.0]])
        tx_power = sim_settings.get("tx_power", 46.0)
        self.transmitters = dict(zip(tx_names, tx_ids))
        for name, pos in zip(tx_names, tx_locations):
            self.scene.add(Transmitter(name=name, position=pos, power_dbm=tx_power, color=[1, 0, 0]))

        rx_names = sim_settings.get("rx_names", [""])
        rx_ids = sim_settings.get("rx_ids", [])
        rx_locations = sim_settings.get("rx_locations", [[50.0, 50.0, 1.5]])
        rx_speed_list = sim_settings.get("rx_speed", [])
        self.propagation_record_mode = sim_settings.get("propagation_record_mode", self.propagation_record_mode)
        if self.propagation_record_mode not in ("future_records", "current_only"):
            raise ValueError("propagation_record_mode must be 'future_records' or 'current_only'")
        self.cache_threshold_buffer = float(sim_settings.get("cache_threshold_buffer", self.cache_threshold_buffer))
        self.virtual_position_min_steps = int(sim_settings.get("virtual_position_min_steps", self.virtual_position_min_steps))
        self.virtual_position_num_angles = int(sim_settings.get("virtual_position_num_angles", self.virtual_position_num_angles))
        self.receivers = dict(zip(rx_names, rx_ids))
        self.rx_speeds = {name: float(speed) for name, speed in zip(rx_names, rx_speed_list)}
        self.rx_prev_positions = {name: list(pos) for name, pos in zip(rx_names, rx_locations)}
        self.rx_last_directions = {}
        for name, pos in zip(rx_names, rx_locations):
            self.scene.add(Receiver(name=name, position=pos, color=[0, 1, 0], display_radius=0.5))

        rx_mesh = sim_settings.get("rx_mesh", self.assets_dir + "/objects/cube.obj")
        rx_material = RadioMaterial(
            "rx_material", relative_permittivity=1.0, conductivity=1e10, scattering_coefficient=0.1
        )
        self.scene.edit(add=[
            SceneObject(name=f"rx_obj_{n}", fname=rx_mesh, radio_material=rx_material)
            for n in rx_names
        ])
        for name, pos in zip(rx_names, rx_locations):
            self.scene.get(f"rx_obj_{name}").position = [pos[0], pos[1], 0.2]

    def update_position(self, rx_name: str, position: list[float]):
        if position is None:
            return
        prev = self.rx_prev_positions.get(rx_name)
        if prev is not None:
            dp = [position[i] - prev[i] for i in range(3)]
            norm_xy = (dp[0]**2 + dp[1]**2) ** 0.5
            if norm_xy > 1e-6:
                self.rx_last_directions[rx_name] = dp
        self.rx_prev_positions[rx_name] = list(position)
        if rx_name in self.scene.receivers:
            self.scene.receivers[rx_name].position = position
        obj_name = f"rx_obj_{rx_name}"
        if obj_name in self.scene.objects:
            self.scene.objects[obj_name].position = [position[0], position[1], 0.2]

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

    def _coherence_displacement_threshold(self, distance_m: float) -> float:
        # Step distance = C++ cache delta * buffer, so virtual-receiver windows overlap.
        # C++ delta = alpha * d * 0.886 / N_cols; buffer adds 10% so windows never gap.
        spatial = self.coherence_alpha * max(distance_m, 0.0) * 0.886 / float(self.coherence_tx_num_cols)
        return max(float(self.virtual_position_min_separation), spatial) * self.cache_threshold_buffer

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

    def _omnidirectional_virtual_rx_positions(self, rx_pos, step_dist, num_rings, rx_height):
        angles = np.linspace(0.0, 2.0 * np.pi, self.virtual_position_num_angles, endpoint=False)
        positions = []
        for ring in range(1, num_rings + 1):
            for angle in angles:
                c, s = np.cos(angle), np.sin(angle)
                positions.append(self._format_point([
                    rx_pos[0] + c * (step_dist * ring),
                    rx_pos[1] + s * (step_dist * ring),
                    rx_height,
                ]))
        return positions

    def calculate_propagation(self):
        if self.scene is None:
            return []

        tx_names = list(self.scene.transmitters.keys())
        rx_names = list(self.scene.receivers.keys())
        if not tx_names or not rx_names:
            return []

        self.propagation_calculation_calls += 1
        self.propagation_calculation_receiver_count += len(rx_names)

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

        tau = self._to_numpy(paths.tau)
        h_raw = paths.cfr(
            frequencies=self.export_frequencies,
            sampling_frequency=1.0,
            num_time_steps=1,
            reverse_direction=False,
            normalize=False,
            out_type="numpy",
        )
        h_raw = np.asarray(h_raw)
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

                power = float(np.mean(np.abs(h) ** 2))
                if power > 0.0:
                    path_loss = float(-10.0 * np.log10(power))
                    h_normalized = h / np.sqrt(power)
                else:
                    path_loss = 200.0
                    h_normalized = h

                link_interactions = interactions[:, rx_idx, tx_idx, :]
                link_valid = valid[rx_idx, tx_idx, :]
                is_none = np.all(link_interactions == InteractionType.NONE, axis=0)
                los_exists = bool(np.any(np.logical_and(link_valid, is_none)))

                records.append({
                    "tx_name": tx_names[tx_idx],
                    "rx_name": rx_names[rx_idx],
                    "src_id": self._node_id("tx", tx_idx),
                    "dst_id": self._node_id("rx", rx_idx),
                    "tx_position": self._format_point(self.scene.transmitters[tx_names[tx_idx]].position.numpy().ravel().tolist()),
                    "rx_position": self._format_point(self.scene.receivers[rx_names[rx_idx]].position.numpy().ravel().tolist()),
                    "delay": link_delay,
                    "path_loss": path_loss,
                    "power": power,
                    "real": np.real(h_normalized).flatten().tolist(),
                    "imag": np.imag(h_normalized).flatten().tolist(),
                    "num_subcarriers": int(self.export_num_subcarriers),
                    "subcarrier_frequencies": self.export_frequencies.tolist(),
                    "los_exist": los_exists,
                })

        return records

    def perform_calculation(self, _current_time: float):
        if self.propagation_record_mode != "future_records":
            return self.calculate_propagation()

        self.clear_virtual_receivers(prefix="ns3vrx_")
        created = []
        real_rx_names = [
            name for name in list(self.scene.receivers.keys())
            if not name.startswith(("vrx_", "ns3vrx_"))
        ]

        for rx_name in real_rx_names:
            rx_pos = self.scene.receivers[rx_name].position.numpy().ravel().tolist()
            rx_pos_arr = np.asarray(rx_pos, dtype=float)
            speed = self.rx_speeds.get(rx_name, 0.0)
            num_virtual = max(self.virtual_position_min_steps, 1)

            link_dist = self._nearest_transmitter_distance(rx_pos) or 0.0
            step_dist = self._coherence_displacement_threshold(link_dist)

            direction = self.rx_last_directions.get(rx_name)
            if direction is not None and speed > 1e-6:
                dx, dy = float(direction[0]), float(direction[1])
                norm = (dx * dx + dy * dy) ** 0.5
                if norm > 1e-6:
                    dir_unit = np.array([dx / norm, dy / norm, 0.0])
                    virtual_points = [
                        self._format_point((rx_pos_arr + dir_unit * (step_dist * k)).tolist())
                        for k in range(1, num_virtual + 1)
                    ]
                else:
                    direction = None

            if direction is None or speed <= 1e-6:
                virtual_points = self._omnidirectional_virtual_rx_positions(
                    rx_pos, step_dist, 1, float(rx_pos[2])
                )

            for idx, point in enumerate(virtual_points):
                vrx_name = f"ns3vrx_{rx_name}_{idx}"
                self.add_receiver(vrx_name, point, rx_id=self.receivers.get(rx_name))
                created.append(vrx_name)

        try:
            records = self.calculate_propagation()
        finally:
            for name in created:
                self.remove_receiver(name)

        return records
