"""
简单的 WRF 文件信息查看脚本。

功能：
- 读取 wrfout NetCDF 文件
- 输出经纬度范围、中心点
- 输出高度范围（米）
- 估算地区水平范围（南北/东西跨度，单位：米）
- 统计风速（最大/最小/平均风速）

用法示例：

    python3 -m wrf_gazebo_bridge.wrf_wrf_info --wrf-file /path/to/wrfout_d01_...
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Tuple


def _load_nc(path: str):
    try:
        import netCDF4  # type: ignore
    except ImportError as exc:  # pragma: no cover
        print(
            "导入 netCDF4 失败，请先安装依赖：\n"
            "  sudo apt install python3-netcdf4\n"
            "或：\n"
            "  pip install netCDF4\n"
            f"当前错误：{exc}",
            file=sys.stderr,
        )
        raise

    return netCDF4.Dataset(path, mode="r")


def _latlon_info(nc) -> Tuple[float, float, float, float, float, float, float, float]:
    """从 XLAT/XLONG 中提取经纬度范围和中心，并估算水平范围（米）。"""
    import numpy as np  # type: ignore

    xlat = nc.variables["XLAT"][0, :, :]  # (south_north, west_east)
    xlon = nc.variables["XLONG"][0, :, :]

    lat_min = float(np.min(xlat))
    lat_max = float(np.max(xlat))
    lon_min = float(np.min(xlon))
    lon_max = float(np.max(xlon))

    lat_center = 0.5 * (lat_min + lat_max)
    lon_center = 0.5 * (lon_min + lon_max)

    # 使用简单球体地球模型估算南北/东西跨度
    earth_radius = 6378137.0  # m
    lat0_rad = math.radians(lat_center)

    dlat = math.radians(lat_max - lat_min)
    dlon = math.radians(lon_max - lon_min)

    north_south_extent = abs(dlat) * earth_radius
    east_west_extent = abs(dlon) * earth_radius * math.cos(lat0_rad)

    return (
        lat_min,
        lat_max,
        lon_min,
        lon_max,
        lat_center,
        lon_center,
        north_south_extent,
        east_west_extent,
    )


def _height_info(nc) -> Tuple[float, float]:
    """通过 PH/PHB 计算质量面高度范围（米）。"""
    import numpy as np  # type: ignore

    if "PH" not in nc.variables or "PHB" not in nc.variables:
        return float("nan"), float("nan")

    ph = nc.variables["PH"][:]  # (Time, bottom_top_stag, ny, nx)
    phb = nc.variables["PHB"][:]
    g = 9.81
    z_stag = (ph + phb) / g  # W 水平高度
    # 质量面高度：相邻 W 面的平均值
    z_mass = 0.5 * (z_stag[:, :-1, :, :] + z_stag[:, 1:, :, :])

    z_min = float(np.min(z_mass))
    z_max = float(np.max(z_mass))
    return z_min, z_max


def _wind_stats(nc) -> Tuple[float, float, float]:
    """统计风速的最小/最大/平均值（m/s）。优先使用 3D U/V/W。"""
    import numpy as np  # type: ignore

    if "U" in nc.variables and "V" in nc.variables:
        # 3D 风场
        u_raw = nc.variables["U"][:]  # (Time, bottom_top, ny, nx_stag)
        v_raw = nc.variables["V"][:]  # (Time, bottom_top, ny_stag, nx)

        # U/V 去除水平交错，得到质量点风速
        u_mass = 0.5 * (u_raw[:, :, :, :-1] + u_raw[:, :, :, 1:])
        v_mass = 0.5 * (v_raw[:, :, :-1, :] + v_raw[:, :, 1:, :])

        if "W" in nc.variables:
            w_raw = nc.variables["W"][:]  # (Time, bottom_top_stag, ny, nx)
            w_mass = 0.5 * (w_raw[:, :-1, :, :] + w_raw[:, 1:, :, :])
        else:
            w_mass = np.zeros_like(u_mass)

        speed = np.sqrt(u_mass ** 2 + v_mass ** 2 + w_mass ** 2)
    elif "U10" in nc.variables and "V10" in nc.variables:
        # 仅使用 10m 风
        u10 = nc.variables["U10"][:]  # (Time, ny, nx)
        v10 = nc.variables["V10"][:]
        speed = np.sqrt(u10 ** 2 + v10 ** 2)
    else:
        raise RuntimeError("未在 wrf 文件中找到 U/V 或 U10/V10 变量，无法计算风速统计量")

    v_min = float(np.min(speed))
    v_max = float(np.max(speed))
    v_mean = float(np.mean(speed))
    return v_min, v_max, v_mean


def summarize_wrf(path: str) -> None:
    """读取 WRF 文件并打印关键信息。"""
    nc = _load_nc(path)

    # 经纬度与水平范围
    (
        lat_min,
        lat_max,
        lon_min,
        lon_max,
        lat_center,
        lon_center,
        north_south_extent,
        east_west_extent,
    ) = _latlon_info(nc)

    # 高度范围
    z_min, z_max = _height_info(nc)

    # 风速统计
    v_min, v_max, v_mean = _wind_stats(nc)

    print("WRF 文件关键信息")
    print("----------------")
    print(f"文件路径: {path}")
    print()

    print("经纬度范围:")
    print(f"  纬度: {lat_min:.4f}°  ~  {lat_max:.4f}°")
    print(f"  经度: {lon_min:.4f}°  ~  {lon_max:.4f}°")
    print(f"  中心点: ({lat_center:.4f}°, {lon_center:.4f}°)")
    print()

    print("水平地区范围（近似值，单位：米）:")
    print(f"  南北跨度: {north_south_extent:.1f} m")
    print(f"  东西跨度: {east_west_extent:.1f} m")
    print()

    print("高度范围（质量面，高度单位：米）:")
    if math.isnan(z_min) or math.isnan(z_max):
        print("  未找到 PH/PHB 变量，无法计算高度范围")
    else:
        print(f"  高度: {z_min:.1f} m  ~  {z_max:.1f} m")
    print()

    print("风速统计（m/s）:")
    print(f"  最小风速: {v_min:.3f} m/s")
    print(f"  最大风速: {v_max:.3f} m/s")
    print(f"  平均风速: {v_mean:.3f} m/s")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="读取 WRF wrfout 文件并输出经纬度、风速和高度等关键信息",
    )
    parser.add_argument(
        "--wrf-file",
        required=True,
        help="wrfout NetCDF 文件路径（.nc 或 wrfout_d01_...）",
    )

    args = parser.parse_args(argv)
    wrf_path = os.path.expanduser(args.wrf_file)

    if not os.path.isfile(wrf_path):
        print(f"错误：文件不存在: {wrf_path}", file=sys.stderr)
        sys.exit(1)

    try:
        summarize_wrf(wrf_path)
    except Exception as exc:  # pragma: no cover
        print(f"分析 WRF 文件时发生错误: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":  # pragma: no cover
    main()

