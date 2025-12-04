import math
import os
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix

try:
    from ament_index_python.packages import (
        get_package_share_directory,
        PackageNotFoundError,
    )
except ImportError:  # pragma: no cover - at runtime we will log if missing
    get_package_share_directory = None  # type: ignore[assignment]

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover - at runtime we will log if missing
    yaml = None  # type: ignore[assignment]


class WrfWindPublisher(Node):
    """Node that reads WRF netCDF and publishes interpolated wind at robot position."""

    def __init__(self) -> None:
        super().__init__("wrf_wind_publisher")

        # Base defaults used when no overrides (CLI or YAML) are provided.
        defaults: Dict[str, object] = {
            "wrf_file_path": "wrfout_data/wrfout_d01_2022-07-01_00_00_00",
            "position_topic": "/gps/fix",
            "wind_topic": "/wrf_wind",
            "use_3d_wind": True,
            "time_interpolation": "linear",
            "default_wind_x": 0.0,
            "default_wind_y": 0.0,
            "default_wind_z": 0.0,
            "time_offset_seconds": 0.0,
        }

        # Declare parameters (configurable via YAML)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        # Apply default YAML config from the installed package, if available.
        # Priority: CLI overrides > YAML file > hard-coded defaults.
        self._apply_default_yaml(defaults)

        wrf_file_path = self.get_parameter("wrf_file_path").get_parameter_value().string_value
        position_topic = self.get_parameter("position_topic").get_parameter_value().string_value
        wind_topic = self.get_parameter("wind_topic").get_parameter_value().string_value
        self.use_3d_wind = self.get_parameter("use_3d_wind").get_parameter_value().bool_value
        self.time_interp = self.get_parameter("time_interpolation").get_parameter_value().string_value
        self.default_wind = (
            self.get_parameter("default_wind_x").get_parameter_value().double_value,
            self.get_parameter("default_wind_y").get_parameter_value().double_value,
            self.get_parameter("default_wind_z").get_parameter_value().double_value,
        )
        self.time_offset_seconds = (
            self.get_parameter("time_offset_seconds").get_parameter_value().double_value
        )

        # Lazy-import netCDF4 / numpy so we can show a clear error if missing
        try:
            import netCDF4  # type: ignore
            import numpy as np  # type: ignore
        except ImportError as exc:  # pragma: no cover - runtime hint
            self.get_logger().error(
                "Python libraries netCDF4 and numpy are required to read wrfout "
                "files. Please install them, for example:\n"
                "  sudo apt install python3-netcdf4 python3-numpy  (Ubuntu)\n"
                "or with pip:\n"
                "  pip install netCDF4 numpy\n"
                f"Import error: {exc}"
            )
            raise

        self._nc = netCDF4.Dataset(wrf_file_path, mode="r")
        self._np = np  # Keep a reference to avoid re-imports in other methods

        self._load_static_fields()

        # Record ROS time when the node starts, used to map onto WRF time axis
        self._start_ros_time = self.get_clock().now()

        # Subscribe to GPS (NavSatFix)
        self._position_sub = self.create_subscription(
            NavSatFix,
            position_topic,
            self._on_position,
            10,
        )

        # Publish wind vector using Vector3Stamped, unit m/s,
        # approximately east(x), north(y), up(z) in WRF grid/world coordinates
        self._wind_pub = self.create_publisher(Vector3Stamped, wind_topic, 10)

        self.get_logger().info(
            f"WRF wind publisher started. wrf file: {wrf_file_path}, "
            f"subscribing to position topic: {position_topic}, publishing wind topic: {wind_topic}"
        )

    def _apply_default_yaml(self, defaults: Dict[str, object]) -> None:
        """Load default parameters from installed YAML if present.

        This is only used when launching via ``ros2 run wrf_gazebo_bridge wrf_wind_publisher``
        without an explicit ``--params-file``. The precedence is:

        1. CLI / launch file overrides (highest)
        2. YAML file ``config/wrf_wind_config.yaml`` from this package
        3. Hard-coded defaults in this file (lowest)
        """
        # If ament_index or yaml is not available, just skip.
        if get_package_share_directory is None or yaml is None:
            return

        try:
            share_dir = get_package_share_directory("wrf_gazebo_bridge")
        except PackageNotFoundError:  # pragma: no cover - runtime only
            self.get_logger().warn(
                "Package share directory for 'wrf_gazebo_bridge' not found; "
                "skipping default YAML parameter loading."
            )
            return

        yaml_path = os.path.join(share_dir, "config", "wrf_wind_config.yaml")
        if not os.path.isfile(yaml_path):
            # Silent skip; this is not an error if the file is not installed.
            return

        try:
            with open(yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:  # pragma: no cover - runtime only
            self.get_logger().warn(
                f"Failed to load default parameter file '{yaml_path}': {exc}"
            )
            return

        # Try to locate the parameters section. The canonical structure is:
        # wrf_wind_publisher:
        #   ros__parameters:
        #     ...
        params_section = {}
        node_name = self.get_name()
        if node_name in data and isinstance(data[node_name], dict):
            params_section = data[node_name].get("ros__parameters", {}) or {}
        elif "wrf_wind_publisher" in data and isinstance(
            data["wrf_wind_publisher"], dict
        ):
            params_section = data["wrf_wind_publisher"].get("ros__parameters", {}) or {}
        elif "ros__parameters" in data and isinstance(data["ros__parameters"], dict):
            params_section = data["ros__parameters"]

        if not params_section:
            return

        params_to_set = []
        for name, yaml_value in params_section.items():
            # Only consider parameters that we know about.
            if name not in defaults:
                continue

            # If current value differs from the hard-coded default,
            # assume it was overridden via CLI/launch and do not touch it.
            current_param = self.get_parameter(name)
            current_value = current_param.value
            default_value = defaults[name]
            if current_value != default_value:
                continue

            params_to_set.append(Parameter(name=name, value=yaml_value))

        if params_to_set:
            self.set_parameters(params_to_set)
            self.get_logger().info(
                f"Loaded default parameters from '{yaml_path}' for: "
                f"{', '.join(p.name for p in params_to_set)}"
            )

    def _load_static_fields(self) -> None:
        """Load WRF grids (time, lat/lon, heights, wind fields) into memory."""
        np = self._np

        # Time axis (XTIME is typically 'minutes since ...')
        self._xtime_minutes = self._nc.variables["XTIME"][:]  # shape (Time,)
        self._nt = self._xtime_minutes.shape[0]

        # Lat / lon grid (assume time-invariant, take time index 0)
        xlat = self._nc.variables["XLAT"][0, :, :]  # (south_north, west_east)
        xlon = self._nc.variables["XLONG"][0, :, :]
        self._xlat = np.array(xlat)
        self._xlon = np.array(xlon)
        self._ny, self._nx = self._xlat.shape

        # Precompute lat/lon bounds to detect out-of-domain queries
        self._lat_min = float(np.min(self._xlat))
        self._lat_max = float(np.max(self._xlat))
        self._lon_min = float(np.min(self._xlon))
        self._lon_max = float(np.max(self._xlon))

        # Flattened lat/lon for neighbor search
        self._lat_flat = self._xlat.reshape(-1)
        self._lon_flat = self._xlon.reshape(-1)
        # Corresponding 2D indices (j, i) for each flattened element
        j_indices, i_indices = np.indices(self._xlat.shape)
        self._j_flat = j_indices.reshape(-1)
        self._i_flat = i_indices.reshape(-1)

        # Load wind fields
        if self.use_3d_wind:
            # U: (Time, bottom_top, south_north, west_east_stag)
            # V: (Time, bottom_top, south_north_stag, west_east)
            # W: (Time, bottom_top_stag, south_north, west_east)
            u_raw = self._nc.variables["U"][:]
            v_raw = self._nc.variables["V"][:]
            w_raw = self._nc.variables["W"][:]

            # Remove staggering so U, V, W are all on (Time, bottom_top, south_north, west_east)
            u_mass = 0.5 * (u_raw[..., :-1] + u_raw[..., 1:])
            v_mass = 0.5 * (v_raw[:, :, :-1, :] + v_raw[:, :, 1:, :])
            w_mass = 0.5 * (w_raw[:, :-1, :, :] + w_raw[:, 1:, :, :])

            self._u = np.array(u_mass)
            self._v = np.array(v_mass)
            self._w = np.array(w_mass)

            # Height (from geopotential)
            ph = self._nc.variables["PH"][:]   # (Time, bottom_top_stag, ny, nx)
            phb = self._nc.variables["PHB"][:]
            g = 9.81
            z_stag = (ph + phb) / g  # geometric height at W levels
            z_mass = 0.5 * (z_stag[:, :-1, :, :] + z_stag[:, 1:, :, :])
            self._z = np.array(z_mass)  # (Time, bottom_top, ny, nx)
        else:
            # Use only 10 m wind, ignore vertical dimension
            self._u10 = np.array(self._nc.variables["U10"][:])  # (Time, ny, nx)
            self._v10 = np.array(self._nc.variables["V10"][:])

    # ---- Interpolation utilities ----

    def _find_horizontal_neighbors(
        self, lat: float, lon: float, k_neighbors: int = 4
    ) -> Optional[Tuple["self._np.ndarray", "self._np.ndarray", "self._np.ndarray"]]:
        """Find several neighboring horizontal grid points with inverse-distance weights."""
        np = self._np

        if (
            lat < self._lat_min
            or lat > self._lat_max
            or lon < self._lon_min
            or lon > self._lon_max
        ):
            return None

        # Compute an approximate distance to every grid point (for weighting only, not geodesy)
        dlat = self._lat_flat - lat
        dlon = (self._lon_flat - lon) * math.cos(math.radians(lat))
        dist2 = dlat * dlat + dlon * dlon
        # If the query lies almost exactly on a grid point, use that point only
        idx_min = int(np.argmin(dist2))
        min_d2 = float(dist2[idx_min])
        if min_d2 < 1e-12:
            j = int(self._j_flat[idx_min])
            i = int(self._i_flat[idx_min])
            return (
                np.array([j], dtype=int),
                np.array([i], dtype=int),
                np.array([1.0], dtype=float),
            )

        # Select K nearest neighbors, use inverse-distance-squared weights
        k = int(min(k_neighbors, dist2.size))
        # argpartition is cheaper than a full sort
        idxs = np.argpartition(dist2, k - 1)[:k]
        d2_sel = dist2[idxs]
        eps = 1e-12
        inv_d2 = 1.0 / (d2_sel + eps)
        weights = inv_d2 / np.sum(inv_d2)

        js = self._j_flat[idxs].astype(int)
        is_ = self._i_flat[idxs].astype(int)
        ws = weights.astype(float)
        return js, is_, ws

    def _find_time_indices(self, now: Time) -> Tuple[int, int, float]:
        """Map ROS time to WRF time-step indices (k0, k1, alpha)."""
        np = self._np

        dt_ros = (now - self._start_ros_time).nanoseconds * 1e-9 + self.time_offset_seconds
        t_minutes = dt_ros / 60.0

        if self._nt == 1:
            return 0, 0, 0.0

        t_array = self._xtime_minutes

        if t_minutes <= float(t_array[0]):
            return 0, 0, 0.0
        if t_minutes >= float(t_array[-1]):
            k = self._nt - 1
            return k, k, 0.0

        # Find indices such that t_array[k0] <= t_minutes <= t_array[k1]
        k1 = int(np.searchsorted(t_array, t_minutes))
        k0 = k1 - 1
        t0 = float(t_array[k0])
        t1 = float(t_array[k1])
        if t1 <= t0:
            alpha = 0.0
        else:
            alpha = (t_minutes - t0) / (t1 - t0)

        if self.time_interp != "linear":
            # Nearest-neighbor mode
            if alpha < 0.5:
                return k0, k0, 0.0
            return k1, k1, 0.0

        return k0, k1, alpha

    def _interpolate_vertical(
        self, z_column: "self._np.ndarray", u_col, v_col, w_col, alt: float
    ) -> Tuple[float, float, float]:
        """Interpolate wind vertically within a single column of levels.

        Strategy:
        - If ``alt`` is between two levels: linear interpolation
        - If ``alt`` is below the lowest level: use the lowest level (near-surface approximation)
        - If ``alt`` is above the highest level: use the highest level
        """
        np = self._np

        # z_column: (bottom_top,)
        # Assume z is monotonically increasing with level index
        z0 = float(z_column[0])
        zN = float(z_column[-1])

        # Below the lowest level: use that level directly
        if alt <= z0:
            return (
                float(u_col[0]),
                float(v_col[0]),
                float(w_col[0]),
            )

        # Above the highest level: use that level directly
        if alt >= zN:
            return (
                float(u_col[-1]),
                float(v_col[-1]),
                float(w_col[-1]),
            )

        # Monotonic z, use searchsorted to find interval
        k1 = int(np.searchsorted(z_column, alt))
        k0 = k1 - 1
        z0 = float(z_column[k0])
        z1 = float(z_column[k1])
        dz = z1 - z0
        if dz <= 0.0:
            alpha = 0.0
        else:
            alpha = (alt - z0) / dz

        u0 = float(u_col[k0])
        u1 = float(u_col[k1])
        v0 = float(v_col[k0])
        v1 = float(v_col[k1])
        w0 = float(w_col[k0])
        w1 = float(w_col[k1])
        u = (1.0 - alpha) * u0 + alpha * u1
        v = (1.0 - alpha) * v0 + alpha * v1
        w = (1.0 - alpha) * w0 + alpha * w1
        return u, v, w

    def _wind_at_single_time(
        self, k_time: int, lat: float, lon: float, alt: float
    ) -> Tuple[float, float, float]:
        """Compute wind vector at a given WRF time index and position (with horizontal interpolation)."""
        neighbors = self._find_horizontal_neighbors(lat, lon)
        if neighbors is None:
            return self.default_wind

        js, is_, ws = neighbors

        if self.use_3d_wind:
            # First interpolate vertically at each neighbor, then do weighted horizontal averaging
            u_sum = 0.0
            v_sum = 0.0
            w_sum = 0.0
            for j, i, w_h in zip(js, is_, ws):
                z_column = self._z[k_time, :, j, i]
                u_col = self._u[k_time, :, j, i]
                v_col = self._v[k_time, :, j, i]
                w_col = self._w[k_time, :, j, i]
                u_loc, v_loc, w_loc = self._interpolate_vertical(
                    z_column, u_col, v_col, w_col, alt
                )
                u_sum += w_h * u_loc
                v_sum += w_h * v_loc
                w_sum += w_h * w_loc
            return float(u_sum), float(v_sum), float(w_sum)

        # 10 m wind only: inverse-distance weighting horizontally, ignore altitude
        u_sum = 0.0
        v_sum = 0.0
        for j, i, w_h in zip(js, is_, ws):
            u_loc = float(self._u10[k_time, j, i])
            v_loc = float(self._v10[k_time, j, i])
            u_sum += w_h * u_loc
            v_sum += w_h * v_loc
        return float(u_sum), float(v_sum), 0.0

    def _compute_wind(self, now: Time, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Compute wind vector at given ROS time and position, including time interpolation."""
        k0, k1, alpha = self._find_time_indices(now)

        if k0 == k1 or alpha <= 0.0:
            return self._wind_at_single_time(k0, lat, lon, alt)

        v0 = self._wind_at_single_time(k0, lat, lon, alt)
        v1 = self._wind_at_single_time(k1, lat, lon, alt)

        u = (1.0 - alpha) * v0[0] + alpha * v1[0]
        v = (1.0 - alpha) * v0[1] + alpha * v1[1]
        w = (1.0 - alpha) * v0[2] + alpha * v1[2]
        return u, v, w

    # ---- ROS callbacks ----

    def _on_position(self, msg: NavSatFix) -> None:
        """Callback for NavSatFix; compute and publish wind given the current geodetic position."""
        # If NavSatFix altitude is NaN, you may modify this to use a fixed height instead
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        now = self.get_clock().now()
        u, v, w = self._compute_wind(now, lat, lon, alt)

        wind_msg = Vector3Stamped()
        wind_msg.header.stamp = now.to_msg()
        wind_msg.header.frame_id = "wrf_wind"
        wind_msg.vector.x = float(u)
        wind_msg.vector.y = float(v)
        wind_msg.vector.z = float(w)

        self._wind_pub.publish(wind_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WrfWindPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
