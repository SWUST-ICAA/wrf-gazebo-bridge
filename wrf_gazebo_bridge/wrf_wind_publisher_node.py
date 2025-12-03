import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix


class WrfWindPublisher(Node):
    """Node that reads WRF netCDF and publishes interpolated wind at robot position."""

    def __init__(self) -> None:
        super().__init__("wrf_wind_publisher")

        # 声明参数（可在 YAML 中配置）
        self.declare_parameter("wrf_file_path", "wrfout_data/wrfout_d01_2022-07-01_00_00_00")
        self.declare_parameter("position_topic", "/gps/fix")
        self.declare_parameter("wind_topic", "/wrf_wind")
        self.declare_parameter("use_3d_wind", True)
        self.declare_parameter("time_interpolation", "linear")  # 或 "nearest"
        self.declare_parameter("default_wind_x", 0.0)
        self.declare_parameter("default_wind_y", 0.0)
        self.declare_parameter("default_wind_z", 0.0)
        self.declare_parameter("time_offset_seconds", 0.0)

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

        # 延迟导入 netCDF4 / numpy，便于在系统未安装时给出清晰错误
        try:
            import netCDF4  # type: ignore
            import numpy as np  # type: ignore
        except ImportError as exc:  # pragma: no cover - 运行时提示
            self.get_logger().error(
                "需要 Python 库 netCDF4 和 numpy 才能读取 wrfout 文件，请先安装：\n"
                "  sudo apt install python3-netcdf4 python3-numpy  (Ubuntu)\n"
                "或使用 pip 安装：\n"
                "  pip install netCDF4 numpy\n"
                f"当前导入错误: {exc}"
            )
            raise

        self._nc = netCDF4.Dataset(wrf_file_path, mode="r")
        self._np = np  # 保存引用，避免在其他方法重复导入

        self._load_static_fields()

        # 记录节点启动 ROS 时间，用于与 WRF 时间轴做简单映射
        self._start_ros_time = self.get_clock().now()

        # 订阅 GPS（NavSatFix）
        self._position_sub = self.create_subscription(
            NavSatFix,
            position_topic,
            self._on_position,
            10,
        )

        # 发布风矢量，使用 Vector3Stamped，单位 m/s，在 WRF 网格坐标近似东(x)、北(y)、上(z)
        self._wind_pub = self.create_publisher(Vector3Stamped, wind_topic, 10)

        self.get_logger().info(
            f"WRF wind publisher 已启动，wrf 文件: {wrf_file_path}, "
            f"订阅位置话题: {position_topic}, 发布风话题: {wind_topic}"
        )

    def _load_static_fields(self) -> None:
        """Load WRF grids (time, lat/lon, heights, wind fields) into memory."""
        np = self._np

        # 时间轴（XTIME 单位为 minutes since ...）
        self._xtime_minutes = self._nc.variables["XTIME"][:]  # shape (Time,)
        self._nt = self._xtime_minutes.shape[0]

        # 经纬度网格（假设随时间不变，取第 0 个时间步）
        xlat = self._nc.variables["XLAT"][0, :, :]  # (south_north, west_east)
        xlon = self._nc.variables["XLONG"][0, :, :]
        self._xlat = np.array(xlat)
        self._xlon = np.array(xlon)
        self._ny, self._nx = self._xlat.shape

        # 预计算经纬度范围，用于判断是否超出水平范围
        self._lat_min = float(np.min(self._xlat))
        self._lat_max = float(np.max(self._xlat))
        self._lon_min = float(np.min(self._xlon))
        self._lon_max = float(np.max(self._xlon))

        # 展平后的经纬度，用于最近邻搜索
        self._lat_flat = self._xlat.reshape(-1)
        self._lon_flat = self._xlon.reshape(-1)
        # 对应的二维索引
        j_indices, i_indices = np.indices(self._xlat.shape)
        self._j_flat = j_indices.reshape(-1)
        self._i_flat = i_indices.reshape(-1)

        # 读取风场
        if self.use_3d_wind:
            # U: (Time, bottom_top, south_north, west_east_stag)
            # V: (Time, bottom_top, south_north_stag, west_east)
            # W: (Time, bottom_top_stag, south_north, west_east)
            u_raw = self._nc.variables["U"][:]
            v_raw = self._nc.variables["V"][:]
            w_raw = self._nc.variables["W"][:]

            # 去除 stagger，使 U, V, W 都在 (Time, bottom_top, south_north, west_east) 上
            u_mass = 0.5 * (u_raw[..., :-1] + u_raw[..., 1:])
            v_mass = 0.5 * (v_raw[:, :, :-1, :] + v_raw[:, :, 1:, :])
            w_mass = 0.5 * (w_raw[:, :-1, :, :] + w_raw[:, 1:, :, :])

            self._u = np.array(u_mass)
            self._v = np.array(v_mass)
            self._w = np.array(w_mass)

            # 高度（利用 geopotential）
            ph = self._nc.variables["PH"][:]   # (Time, bottom_top_stag, ny, nx)
            phb = self._nc.variables["PHB"][:]
            g = 9.81
            z_stag = (ph + phb) / g  # W 水平上的几何高度
            z_mass = 0.5 * (z_stag[:, :-1, :, :] + z_stag[:, 1:, :, :])
            self._z = np.array(z_mass)  # (Time, bottom_top, ny, nx)
        else:
            # 只使用 10m 风，忽略垂直维
            self._u10 = np.array(self._nc.variables["U10"][:])  # (Time, ny, nx)
            self._v10 = np.array(self._nc.variables["V10"][:])

    # ---- 插值工具函数 ----

    def _find_horizontal_neighbors(
        self, lat: float, lon: float, k_neighbors: int = 4
    ) -> Optional[Tuple["self._np.ndarray", "self._np.ndarray", "self._np.ndarray"]]:
        """在水平网格上找到若干个邻近网格点，并给出反距离权重."""
        np = self._np

        if (
            lat < self._lat_min
            or lat > self._lat_max
            or lon < self._lon_min
            or lon > self._lon_max
        ):
            return None

        # 先计算到所有网格点的粗略距离（未考虑投影，只作为插值权重用）
        dlat = self._lat_flat - lat
        dlon = (self._lon_flat - lon) * math.cos(math.radians(lat))
        dist2 = dlat * dlat + dlon * dlon
        # 若恰好落在某个网格点上，则直接使用该点
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

        # 选取若干个最近邻，用反距离平方作为权重
        k = int(min(k_neighbors, dist2.size))
        # argpartition 比完整排序更快
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
        """Map ROS 时间到 WRF 时间步索引 (k0, k1, alpha)."""
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

        # 找到 t_array[k0] <= t_minutes <= t_array[k1]
        k1 = int(np.searchsorted(t_array, t_minutes))
        k0 = k1 - 1
        t0 = float(t_array[k0])
        t1 = float(t_array[k1])
        if t1 <= t0:
            alpha = 0.0
        else:
            alpha = (t_minutes - t0) / (t1 - t0)

        if self.time_interp != "linear":
            # 最近邻模式
            if alpha < 0.5:
                return k0, k0, 0.0
            return k1, k1, 0.0

        return k0, k1, alpha

    def _interpolate_vertical(
        self, z_column: "self._np.ndarray", u_col, v_col, w_col, alt: float
    ) -> Tuple[float, float, float]:
        """在单个垂直柱 (levels) 内按高度插值风速。"""
        np = self._np

        # z_column: (bottom_top,)
        # 若高度超出范围，这里简单返回默认风场，可根据需要改为夹紧到边界
        z_min = float(np.min(z_column))
        z_max = float(np.max(z_column))
        if alt <= z_min or alt >= z_max:
            return self.default_wind

        # 假设 z 随 level 单调，使用 searchsorted
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
        """给定 WRF 时间步索引和位置，计算该时刻风矢量（含水平插值）。"""
        neighbors = self._find_horizontal_neighbors(lat, lon)
        if neighbors is None:
            return self.default_wind

        js, is_, ws = neighbors

        if self.use_3d_wind:
            # 先在每个水平方向邻点的垂直方向做插值，再对这些结果做水平加权平均
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

        # 只使用 10m 风：先在水平上做反距离权重，再忽略高度
        u_sum = 0.0
        v_sum = 0.0
        for j, i, w_h in zip(js, is_, ws):
            u_loc = float(self._u10[k_time, j, i])
            v_loc = float(self._v10[k_time, j, i])
            u_sum += w_h * u_loc
            v_sum += w_h * v_loc
        return float(u_sum), float(v_sum), 0.0

    def _compute_wind(self, now: Time, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """根据当前 ROS 时间与位置计算风矢量（进行时间插值）。"""
        k0, k1, alpha = self._find_time_indices(now)

        if k0 == k1 or alpha <= 0.0:
            return self._wind_at_single_time(k0, lat, lon, alt)

        v0 = self._wind_at_single_time(k0, lat, lon, alt)
        v1 = self._wind_at_single_time(k1, lat, lon, alt)

        u = (1.0 - alpha) * v0[0] + alpha * v1[0]
        v = (1.0 - alpha) * v0[1] + alpha * v1[1]
        w = (1.0 - alpha) * v0[2] + alpha * v1[2]
        return u, v, w

    # ---- ROS 回调 ----

    def _on_position(self, msg: NavSatFix) -> None:
        """位置订阅回调，根据 NavSatFix 的经纬度/高度计算风场并发布。"""
        # 如果 NavSatFix 中的 altitude 为 NaN，可按需要修改为使用固定高度
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
