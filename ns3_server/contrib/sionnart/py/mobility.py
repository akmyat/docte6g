import math
import heapq
from typing import List, Dict, Optional

import pybullet as p
import pybullet_data


class SionnaMobility:
    def __init__(self):
        self.obj_registry = {}
        self.half_extents_registry = {}
        self.obj_modes = {}
        self.obj_paths = {}
        self.obj_speeds = {}
        self.obj_last_timestamps = {}
        self.obj_path_start_times = {}
        self.is_recording = False
        self.animation_data = {}

    def initialize(self, mobility_settings):
        if p.isConnected():
            p.disconnect()
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        scene_path = mobility_settings.get("scene")
        if scene_path:
            self.add_scene(scene_path)

        rx_mesh = mobility_settings.get("rx_mesh", "")
        tx_mesh = mobility_settings.get("tx_mesh", rx_mesh)
        tx_names = mobility_settings.get("tx_names", [])
        tx_locations = mobility_settings.get("tx_locations", [])
        tx_modes = mobility_settings.get("tx_modes", [])
        tx_speeds = mobility_settings.get("tx_speed", mobility_settings.get("tx_speeds", []))
        for idx, (name, loc, mode) in enumerate(zip(tx_names, tx_locations, self._normalized_modes(tx_names, tx_modes))):
            self.add_object(tx_mesh, name, loc, mobility_mode=mode)
            self._set_initial_speed_from_settings(name, tx_speeds, idx)

        rx_names = mobility_settings.get("rx_names", [])
        rx_locations = mobility_settings.get("rx_locations", [])
        rx_modes = mobility_settings.get("rx_modes", [])
        rx_speeds = mobility_settings.get("rx_speed", mobility_settings.get("rx_speeds", []))
        for idx, (name, loc, mode) in enumerate(zip(rx_names, rx_locations, self._normalized_modes(rx_names, rx_modes))):
            self.add_object(rx_mesh, name, loc, mobility_mode=mode)
            self._set_initial_speed_from_settings(name, rx_speeds, idx)

    # ------------------------------------------------------------------
    # Scene / object management
    # ------------------------------------------------------------------

    def add_scene(self, scene_path: str) -> bool:
        if "scene" in self.obj_registry:
            p.removeBody(self.obj_registry["scene"])
        try:
            visual_id = p.createVisualShape(p.GEOM_MESH, fileName=scene_path)
            collision_id = p.createCollisionShape(
                p.GEOM_MESH, fileName=scene_path,
                flags=p.GEOM_FORCE_CONCAVE_TRIMESH
            )
            body_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id,
                                        basePosition=[0, 0, 0])
        except Exception:
            return False
        self.obj_registry["scene"] = body_id
        return True

    def remove_scene(self) -> bool:
        if "scene" not in self.obj_registry:
            return False
        p.removeBody(self.obj_registry.pop("scene"))
        return True

    def add_object(
        self,
        obj_path: str,
        obj_name: str,
        position=None,
        mobility_mode: str = "CONSTANT_POSITION",
    ) -> bool:
        if position is None:
            position = [0, 0, 0]
        if obj_name in self.obj_registry:
            p.removeBody(self.obj_registry[obj_name])
        try:
            half_extents = self._mesh_half_extents(obj_path)
            center_pos = self._bottom_to_center(half_extents, position)
            try:
                visual_id = p.createVisualShape(p.GEOM_MESH, fileName=obj_path)
            except Exception:
                visual_id = -1
            collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
            body_id = p.createMultiBody(baseMass=1,
                                        baseCollisionShapeIndex=collision_id,
                                        baseVisualShapeIndex=visual_id,
                                        basePosition=center_pos)
        except Exception:
            return False
        self.obj_registry[obj_name] = body_id
        self.half_extents_registry[obj_name] = half_extents
        self.obj_modes[obj_name] = mobility_mode
        self.obj_paths[obj_name] = []
        self.obj_speeds[obj_name] = 1.0
        self.obj_last_timestamps[obj_name] = 0.0
        self.obj_path_start_times[obj_name] = 0.0
        return True

    def remove_object(self, obj_name: str) -> bool:
        if obj_name not in self.obj_registry:
            return False
        p.removeBody(self.obj_registry.pop(obj_name))
        del self.half_extents_registry[obj_name]
        del self.obj_modes[obj_name]
        del self.obj_paths[obj_name]
        del self.obj_speeds[obj_name]
        del self.obj_last_timestamps[obj_name]
        del self.obj_path_start_times[obj_name]
        return True

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def get_position(self, obj_name: str) -> list:
        try:
            pos, _ = p.getBasePositionAndOrientation(self.obj_registry[obj_name])
            return self._center_to_bottom(self.half_extents_registry[obj_name], list(pos))
        except Exception:
            return [0.0, 0.0, 0.0]

    def set_speed(self, obj_name: str, speed: float) -> bool:
        if obj_name not in self.obj_speeds:
            return False
        self.obj_speeds[obj_name] = max(0.1, speed)
        return True

    def _set_initial_speed_from_settings(self, obj_name: str, speeds, idx: int) -> None:
        if isinstance(speeds, (int, float)):
            self.set_speed(obj_name, float(speeds))
            return
        if idx < len(speeds):
            self.set_speed(obj_name, float(speeds[idx]))

    def set_mobility_mode(self, obj_name: str, mobility_mode: str) -> bool:
        if obj_name not in self.obj_modes:
            return False
        self.obj_modes[obj_name] = mobility_mode
        return True

    def get_mobility_mode(self, obj_name: str) -> Optional[str]:
        return self.obj_modes.get(obj_name)

    def _path_state(self, obj_name: str, timestamp: float):
        path = self.obj_paths.get(obj_name)
        if not path:
            return None, 0.0, [], 0.0

        speed = self.obj_speeds.get(obj_name, 1.0)
        elapsed = timestamp - self.obj_path_start_times.get(obj_name, 0.0)
        dist_traveled = speed * elapsed

        cumulative = [0.0]
        for i in range(1, len(path)):
            cumulative.append(cumulative[-1] + math.dist(path[i - 1], path[i]))

        return path, speed, cumulative, dist_traveled

    @staticmethod
    def _segment_index(cumulative: List[float], dist_traveled: float) -> Optional[int]:
        if len(cumulative) < 2 or dist_traveled >= cumulative[-1]:
            return None
        try:
            return next(i for i in range(len(cumulative) - 1) if cumulative[i] <= dist_traveled < cumulative[i + 1])
        except StopIteration:
            return None

    def get_velocity(self, obj_name: str, timestamp: float) -> Optional[list]:
        """
        Return the current velocity vector for obj_name at timestamp.

        The magnitude comes from the stored scalar speed and the direction is
        derived from the active path segment. If the object has no path or the
        direction cannot be determined, returns a zero vector.
        """
        path, speed, cumulative, dist_traveled = self._path_state(obj_name, timestamp)
        if not path or len(path) == 1:
            return [0.0, 0.0, 0.0]

        seg = self._segment_index(cumulative, dist_traveled)
        if seg is None:
            return [0.0, 0.0, 0.0]
        seg_start, seg_end = path[seg], path[seg + 1]

        dx = seg_end[0] - seg_start[0]
        dy = seg_end[1] - seg_start[1]
        dz = seg_end[2] - seg_start[2]
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-9:
            return [0.0, 0.0, 0.0]

        scale = speed / norm
        return [dx * scale, dy * scale, dz * scale]

    def move_object(self, obj_name: str, position: list) -> bool:
        if self.check_collision(obj_name, position):
            return False
        obj_id = self.obj_registry[obj_name]
        _, orientation = p.getBasePositionAndOrientation(obj_id)
        center_pos = self._bottom_to_center(self.half_extents_registry[obj_name], position)
        try:
            p.resetBasePositionAndOrientation(obj_id, center_pos, orientation)
            return True
        except Exception:
            return False

    def check_collision(
        self,
        obj_name: str,
        position: list,
        exclude_object: Optional[str] = None
    ) -> bool:
        if obj_name not in self.obj_registry:
            return True
        obj_id = self.obj_registry[obj_name]
        try:
            p.getBodyInfo(obj_id)
        except Exception:
            return True

        original_pos, original_ori = p.getBasePositionAndOrientation(obj_id)
        center_pos = self._bottom_to_center(self.half_extents_registry[obj_name], position)
        try:
            p.resetBasePositionAndOrientation(obj_id, center_pos, original_ori)
            p.performCollisionDetection()

            for other_name, other_id in self.obj_registry.items():
                if other_name == obj_name:
                    continue
                if exclude_object == "*" and other_name != "scene":
                    continue
                if exclude_object == other_name:
                    continue
                try:
                    p.getBodyInfo(other_id)
                except Exception:
                    continue

                for pt in p.getContactPoints(bodyA=obj_id, bodyB=other_id) or []:
                    normal = pt[7] if len(pt) > 7 else (0, 0, 0)
                    dist   = pt[8] if len(pt) > 8 else 0.0
                    nx, ny, nz = normal[:3]
                    lateral = abs(nx) + abs(ny)
                    if abs(nz) > 0.9 and lateral < 0.2:
                        continue
                    if dist < -0.001 or (dist <= 0.005 and lateral > 0.2):
                        return True
        finally:
            try:
                p.resetBasePositionAndOrientation(obj_id, original_pos, original_ori)
            except Exception:
                pass
        return False

    def set_destination(
        self,
        obj_name: str,
        destination: list,
        grid_size: float = 0.5,
        max_iterations: int = 10000,
        safety_radius: float = 0.5,
        avoid_other_paths: bool = False,
    ) -> bool:
        if obj_name not in self.obj_paths:
            return False

        avoid_paths = [p_ for k, p_ in self.obj_paths.items() if k != obj_name and p_]
        path = self.find_path(obj_name, destination, grid_size, max_iterations,
                              avoid_paths, safety_radius, avoid_other_paths)
        self.obj_paths[obj_name] = path
        self.obj_path_start_times[obj_name] = self.obj_last_timestamps[obj_name]

        if not path:
            return False
        last = path[-1]
        return math.sqrt((last[0]-destination[0])**2 + (last[1]-destination[1])**2) <= grid_size

    def find_path(
        self,
        obj_name: str,
        destination: list,
        grid_size: float = 0.5,
        max_iterations: int = 10000,
        avoid_paths: Optional[List[List[List[float]]]] = None,
        safety_radius: float = 0.5,
        avoid_other_paths: bool = False,
    ) -> List[List[float]]:
        start = self.get_position(obj_name)
        if math.dist(start[:2], destination[:2]) < 0.01:
            return [start]

        def snap(pos):
            return (round(pos[0] / grid_size) * grid_size,
                    round(pos[1] / grid_size) * grid_size,
                    pos[2])

        blocked = set()
        for other_name in self.obj_registry:
            if other_name in (obj_name, "scene"):
                continue
            try:
                opos = self.get_position(other_name)
                for di in range(-2, 3):
                    for dj in range(-2, 3):
                        ox, oy = di * grid_size, dj * grid_size
                        if math.sqrt(ox**2 + oy**2) <= safety_radius:
                            blocked.add(snap((opos[0]+ox, opos[1]+oy, opos[2])))
            except Exception:
                continue

        if avoid_other_paths and avoid_paths:
            for ap in avoid_paths:
                for wp in ap:
                    blocked.add(snap(wp))
                    for di in range(-1, 2):
                        for dj in range(-1, 2):
                            ox, oy = di * grid_size, dj * grid_size
                            if math.sqrt(ox**2 + oy**2) <= safety_radius:
                                blocked.add(snap((wp[0]+ox, wp[1]+oy, wp[2])))

        cache = {}
        def is_free(pos):
            if pos in blocked:
                return False
            if pos not in cache:
                cache[pos] = not self.check_collision(obj_name, list(pos), exclude_object="*")
            return cache[pos]

        start_grid = snap(start)
        open_set = [(0, 0, start_grid)]
        came_from, g_score, closed_set = {}, {start_grid: 0}, set()
        best_dist, best_node = float('inf'), start_grid
        dirs = [(grid_size,0,0),(-grid_size,0,0),(0,grid_size,0),(0,-grid_size,0),
                (grid_size,grid_size,0),(-grid_size,-grid_size,0),
                (grid_size,-grid_size,0),(-grid_size,grid_size,0)]

        for _ in range(max_iterations):
            if not open_set:
                break
            _, _, cur = heapq.heappop(open_set)
            if cur in closed_set:
                continue
            closed_set.add(cur)

            d = math.dist(cur[:2], destination[:2])
            if d < best_dist:
                best_dist, best_node = d, cur

            if d <= grid_size:
                path = []
                node = cur
                while node in came_from:
                    path.append(list(node))
                    node = came_from[node]
                path.append(list(start))
                path.reverse()
                end = destination if not self.check_collision(obj_name, destination, exclude_object="*") else list(cur)
                path.append(end)
                return path

            for dx, dy, dz in dirs:
                nb = (cur[0]+dx, cur[1]+dy, cur[2])
                if nb in closed_set or not is_free(nb):
                    continue
                cost = grid_size * (math.sqrt(2) if dx and dy else 1)
                tg = g_score[cur] + cost
                if nb not in g_score or tg < g_score[nb]:
                    came_from[nb] = cur
                    g_score[nb] = tg
                    heapq.heappush(open_set, (tg + math.dist(nb[:2], destination[:2]), tg, nb))

        path = []
        node = best_node
        while node in came_from:
            path.append(list(node))
            node = came_from[node]
        path.append(list(start))
        path.reverse()
        return path

    def update_position(self, obj_name: str, timestamp: float) -> Optional[list]:
        """
        Advance obj_name along its path to the given timestamp.
        Returns the new bottom position, or None if there is no path or movement
        was blocked by a collision.
        """
        path, speed, cumulative, dist_traveled = self._path_state(obj_name, timestamp)
        if not path:
            return None

        self.obj_last_timestamps[obj_name] = timestamp

        if dist_traveled >= cumulative[-1]:
            final = path[-1]
            if self.check_collision(obj_name, final, exclude_object="scene"):
                return None
            self.move_object(obj_name, final)
            return final

        seg = self._segment_index(cumulative, dist_traveled)
        if seg is None:
            return None

        seg_len = cumulative[seg + 1] - cumulative[seg]
        if seg_len < 0.001:
            pos = path[seg]
        else:
            t = (dist_traveled - cumulative[seg]) / seg_len
            s, e = path[seg], path[seg+1]
            pos = [s[j] + t * (e[j] - s[j]) for j in range(3)]

        if self.check_collision(obj_name, pos, exclude_object="scene"):
            return None
        self.move_object(obj_name, pos)
        return pos

    def get_next_waypoint(self, obj_name: str, timestamp: float) -> Optional[list]:
        """
        Returns the next waypoint in the path for the given object and timestamp.
        """
        path, _, cumulative, dist_traveled = self._path_state(obj_name, timestamp)
        if not path or len(path) < 2:
            return None

        seg = self._segment_index(cumulative, dist_traveled)
        if seg is None:
            return None
        return path[seg + 1]

    def get_object_path(self, obj_name: str) -> List[List[float]]:
        return self.obj_paths.get(obj_name, [])

    def get_all_paths(self) -> Dict[str, List[List[float]]]:
        return self.obj_paths

    def get_all_mobility_modes(self) -> Dict[str, str]:
        return dict(self.obj_modes)

    # ------------------------------------------------------------------
    # Animation recording
    # ------------------------------------------------------------------

    def start_recording(self) -> None:
        self.is_recording = True

    def stop_recording(self) -> None:
        self.is_recording = False

    def record_animation_frame(self, timestamp: float) -> None:
        if not self.is_recording:
            return
        for obj_name, obj_id in self.obj_registry.items():
            if obj_name == "scene":
                continue
            pos, rot = p.getBasePositionAndOrientation(obj_id)
            bottom_pos = self._center_to_bottom(self.half_extents_registry[obj_name], list(pos))
            self.animation_data.setdefault(obj_name, []).append({
                'timestamp': timestamp,
                'position': bottom_pos,
                'rotation': list(rot),
            })

    def export_animation_for_blender(self, filename: str = "animation_data.json") -> bool:
        try:
            import json
            export_data = {
                'scene_info': {'fps': 30, 'unit': 'meters'},
                'objects': {
                    name: {'keyframes': frames, 'mesh_file': 'cube.obj'}
                    for name, frames in self.animation_data.items() if frames
                }
            }
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            return True
        except Exception:
            return False

    def clear_animation_data(self) -> None:
        self.animation_data = {}

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _bottom_to_center(half_extents: list, pos: list) -> list:
        return [pos[0], pos[1], pos[2] + half_extents[2]]

    @staticmethod
    def _center_to_bottom(half_extents: list, pos: list) -> list:
        return [pos[0], pos[1], pos[2] - half_extents[2]]

    @staticmethod
    def _mesh_half_extents(obj_path: str) -> list:
        mins = [float('inf')] * 3
        maxs = [float('-inf')] * 3
        with open(obj_path) as f:
            for line in f:
                if line.startswith('v '):
                    x, y, z = map(float, line.split()[1:4])
                    mins = [min(mins[i], v) for i, v in enumerate([x, y, z])]
                    maxs = [max(maxs[i], v) for i, v in enumerate([x, y, z])]
        return [(maxs[i] - mins[i]) / 2 for i in range(3)]

    @staticmethod
    def _normalized_modes(names: List[str], modes: Optional[List[str]]) -> List[str]:
        normalized = list(modes or [])
        if len(normalized) < len(names):
            normalized.extend(["CONSTANT_POSITION"] * (len(names) - len(normalized)))
        return normalized[:len(names)]
