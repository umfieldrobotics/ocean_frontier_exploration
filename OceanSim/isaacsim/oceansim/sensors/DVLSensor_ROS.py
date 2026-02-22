import numpy as np
import carb
import omni.graph.core as og
import omni.timeline

from isaacsim.oceansim.sensors.DVLsensor import DVLsensor


class DVLSensor_ROS(DVLsensor):
    """DVL sensor wrapper that publishes a custom ROS message via OmniGraph ROS2Publisher."""

    def __init__(
        self,
        name: str = "DVL",
        elevation: float = 22.5,
        rotation: float = 45,
        vel_cov=0,
        depth_cov=0,
        min_range: float = 0.1,
        max_range: float = 100,
        num_beams_out_range_threshold: int = 2,
        freq: int = None,
        freq_bound: tuple[int] = [5, 100],
        freq_dependenet_range_bound: tuple[float] = [7.5, 50.0],
        sound_speed: float = 1500,
        og_node=None,
    ):
        super().__init__(
            name=name,
            elevation=elevation,
            rotation=rotation,
            vel_cov=vel_cov,
            depth_cov=depth_cov,
            min_range=min_range,
            max_range=max_range,
            num_beams_out_range_threshold=num_beams_out_range_threshold,
            freq=freq,
            freq_bound=freq_bound,
            freq_dependenet_range_bound=freq_dependenet_range_bound,
            sound_speed=sound_speed,
        )

        self._og_node = og_node
        self._warned_missing_attrs = False
        self._last_payload = None
        self._default_velocity_covariance = [0.0] * 9
        self._resolved_attr_map = None

        self._attr_aliases = {
            "timestamp": ["inputs:timestamp", "inputs:stamp"],
            "time": ["inputs:time"],
            "vx": ["inputs:vx"],
            "vy": ["inputs:vy"],
            "vz": ["inputs:vz"],
            "velocity_covariance": ["inputs:velocity_covariance"],
            "fom": ["inputs:fom"],
            "altitude": ["inputs:altitude"],
            "velocity_valid": ["inputs:velocity_valid"],
            "status": ["inputs:status"],
        }

    def initialize(self, og_node=None):
        self._og_node = og_node
        self._resolved_attr_map = None
        self._warned_missing_attrs = False

    def _is_velocity_valid(self) -> bool:
        beam_hits = self.get_beam_hit()
        misses = sum(not bool(hit) for hit in beam_hits)
        return misses < self._num_beams_out_range_threshold

    def _compute_velocity_covariance(self) -> list[float]:
        sqrt_cov = np.asarray(self._mvn_vel.get_sqrt_cov(), dtype=np.float64)
        if sqrt_cov.shape != (4, 4):
            return self._default_velocity_covariance

        beam_cov = sqrt_cov @ sqrt_cov.T
        vel_cov = self._transform @ beam_cov @ self._transform.T
        if not np.isfinite(vel_cov).all():
            return self._default_velocity_covariance

        return vel_cov.reshape(9).astype(np.float64).tolist()

    def _compute_altitude(self) -> float:
        depth = np.asarray(self.get_depth(), dtype=np.float64)
        valid_depth = depth[np.isfinite(depth)]
        if valid_depth.size == 0:
            return 0.0
        return float(np.min(valid_depth))

    def _build_payload(self) -> dict:
        sim_time = float(omni.timeline.get_timeline_interface().get_current_time())
        velocity_valid = self._is_velocity_valid()

        if velocity_valid:
            vel = np.asarray(self.get_linear_vel(), dtype=np.float64)
        else:
            vel = np.zeros(3, dtype=np.float64)

        payload = {
            "timestamp": sim_time,
            "time": sim_time,
            "vx": float(vel[0]),
            "vy": float(vel[1]),
            "vz": float(vel[2]),
            "velocity_covariance": self._compute_velocity_covariance(),
            "fom": 0.0,
            "altitude": self._compute_altitude(),
            "velocity_valid": bool(velocity_valid),
            "status": 0 if velocity_valid else 1,
        }
        return payload

    def _resolve_attr_map(self):
        if self._og_node is None:
            return None

        resolved = {}
        for key, candidates in self._attr_aliases.items():
            selected = None
            for attr_name in candidates:
                if self._og_node.get_attribute_exists(attr_name):
                    selected = attr_name
                    break
            if selected is None:
                return None
            resolved[key] = selected
        return resolved

    def _set_og_attr(self, attr_name: str, value):
        if not self._og_node.get_attribute_exists(attr_name):
            raise RuntimeError(f"Missing OmniGraph attribute: {attr_name}")
        attr = self._og_node.get_attribute(attr_name)
        og.Controller.attribute(attr).set(value)

    def _publish_payload(self, payload: dict):
        if self._og_node is None:
            return

        if self._resolved_attr_map is None:
            self._resolved_attr_map = self._resolve_attr_map()
            if self._resolved_attr_map is None:
                if not self._warned_missing_attrs:
                    self._warned_missing_attrs = True
                    carb.log_warn(
                        f"[{self._name}] DVL ROS publish skipped: ROS2Publisher dynamic inputs are not ready. "
                        "Check messagePackage/messageSubfolder/messageName configuration."
                    )
                return

        try:
            self._set_og_attr(self._resolved_attr_map["timestamp"], payload["timestamp"])
            self._set_og_attr(self._resolved_attr_map["time"], payload["time"])
            self._set_og_attr(self._resolved_attr_map["vx"], payload["vx"])
            self._set_og_attr(self._resolved_attr_map["vy"], payload["vy"])
            self._set_og_attr(self._resolved_attr_map["vz"], payload["vz"])
            self._set_og_attr(self._resolved_attr_map["velocity_covariance"], payload["velocity_covariance"])
            self._set_og_attr(self._resolved_attr_map["fom"], payload["fom"])
            self._set_og_attr(self._resolved_attr_map["altitude"], payload["altitude"])
            self._set_og_attr(self._resolved_attr_map["velocity_valid"], payload["velocity_valid"])
            self._set_og_attr(self._resolved_attr_map["status"], payload["status"])
        except Exception as exc:
            if not self._warned_missing_attrs:
                self._warned_missing_attrs = True
                carb.log_warn(f"[{self._name}] DVL ROS publish skipped: {exc}")

    def read(self):
        payload = self._build_payload()
        self._last_payload = payload
        self._publish_payload(payload)

        return np.array([payload["vx"], payload["vy"], payload["vz"]], dtype=np.float64)
