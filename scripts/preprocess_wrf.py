#!/usr/bin/env python3
"""Preprocess WRF output so that Gazebo can consume structured wind fields."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
from netCDF4 import Dataset  # type: ignore

GRAVITY = 9.80665
EARTH_RADIUS_M = 6_378_137.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert a WRF wrfout file into a compact wind-field description "
            "that can be sampled by Gazebo or PX4 plugins."
        )
    )
    parser.add_argument("--wrf-path", required=True, help="Path to the wrfout NetCDF file.")
    parser.add_argument("--output-dir", default="processed_wind", help="Directory to store outputs.")
    parser.add_argument(
        "--time-index", type=int, default=0, help="Time index to export (0 corresponds to the first hour)."
    )
    parser.add_argument("--stride-x", type=int, default=1, help="Horizontal stride along the east-west axis.")
    parser.add_argument("--stride-y", type=int, default=1, help="Horizontal stride along the south-north axis.")
    parser.add_argument("--stride-z", type=int, default=1, help="Vertical stride.")
    parser.add_argument(
        "--float32", action="store_true", help="Store arrays as float32 instead of float64 to save disk space."
    )
    return parser.parse_args()


def decode_time_string(times_var, index: int) -> str:
    raw = times_var[index]
    if hasattr(raw, "tobytes"):
        return raw.tobytes().decode("utf-8").strip()
    return "".join(chr(v) for v in raw).strip()


def destagger(data: np.ndarray, axis: int) -> np.ndarray:
    """Convert WRF staggered grids to mass-grid positions."""
    if data.shape[axis] < 2:
        raise ValueError("Cannot destagger axis with fewer than two cells.")
    slices_a = [slice(None)] * data.ndim
    slices_b = [slice(None)] * data.ndim
    slices_a[axis] = slice(0, -1)
    slices_b[axis] = slice(1, None)
    return 0.5 * (data[tuple(slices_a)] + data[tuple(slices_b)])


def ensure_plain_array(array) -> np.ndarray:
    if np.ma.isMaskedArray(array):
        return np.asarray(array.filled(np.nan))
    return np.asarray(array)


@dataclass
class WindFieldData:
    timestamp: datetime
    east_m: np.ndarray
    north_m: np.ndarray
    terrain_m: np.ndarray
    heights_m: np.ndarray
    heights_agl_m: np.ndarray
    u_ms: np.ndarray
    v_ms: np.ndarray
    w_ms: np.ndarray
    lat0: float
    lon0: float

    @property
    def shape(self) -> Tuple[int, int, int]:
        nz, ny, nx = self.u_ms.shape
        return nx, ny, nz


def load_time_slice(ds: Dataset, args: argparse.Namespace, dtype) -> WindFieldData:
    time_count = len(ds.dimensions["Time"])
    if args.time_index < 0 or args.time_index >= time_count:
        raise IndexError(f"time-index {args.time_index} outside valid range [0,{time_count - 1}]")

    time_string = decode_time_string(ds.variables["Times"], args.time_index)
    try:
        timestamp = datetime.strptime(time_string, "%Y-%m-%d_%H:%M:%S")
    except ValueError:
        timestamp = datetime.fromisoformat(time_string.replace("_", " "))

    lat = ensure_plain_array(ds.variables["XLAT"][args.time_index])
    lon = ensure_plain_array(ds.variables["XLONG"][args.time_index])
    terrain = ensure_plain_array(ds.variables["HGT"][args.time_index])

    lat0 = float(np.nanmean(lat))
    lon0 = float(np.nanmean(lon))
    deg2rad = np.pi / 180.0
    east = (lon - lon0) * deg2rad * np.cos(lat0 * deg2rad) * EARTH_RADIUS_M
    north = (lat - lat0) * deg2rad * EARTH_RADIUS_M

    u = destagger(ensure_plain_array(ds.variables["U"][args.time_index]), axis=-1)
    v = destagger(ensure_plain_array(ds.variables["V"][args.time_index]), axis=-2)
    w = destagger(ensure_plain_array(ds.variables["W"][args.time_index]), axis=0)
    geopotential = destagger(
        ensure_plain_array(ds.variables["PH"][args.time_index] + ds.variables["PHB"][args.time_index]), axis=0
    )
    heights = geopotential / GRAVITY

    z_slice = slice(None, None, args.stride_z)
    y_slice = slice(None, None, args.stride_y)
    x_slice = slice(None, None, args.stride_x)

    east = east[y_slice, x_slice].astype(dtype, copy=False)
    north = north[y_slice, x_slice].astype(dtype, copy=False)
    terrain = terrain[y_slice, x_slice].astype(dtype, copy=False)

    heights = heights[z_slice, y_slice, x_slice].astype(dtype, copy=False)
    u = u[z_slice, y_slice, x_slice].astype(dtype, copy=False)
    v = v[z_slice, y_slice, x_slice].astype(dtype, copy=False)
    w = w[z_slice, y_slice, x_slice].astype(dtype, copy=False)

    heights_agl = heights - terrain[np.newaxis, ...]

    return WindFieldData(
        timestamp=timestamp,
        east_m=east,
        north_m=north,
        terrain_m=terrain,
        heights_m=heights,
        heights_agl_m=heights_agl,
        u_ms=u,
        v_ms=v,
        w_ms=w,
        lat0=lat0,
        lon0=lon0,
    )


def summarize_field(field: WindFieldData, args: argparse.Namespace) -> Dict:
    speed = np.sqrt(field.u_ms ** 2 + field.v_ms ** 2 + field.w_ms ** 2)
    dx = np.nanmedian(np.diff(field.east_m, axis=1))
    dy = np.nanmedian(np.diff(field.north_m, axis=0))
    grid_info = {
        "nx": int(field.u_ms.shape[2]),
        "ny": int(field.u_ms.shape[1]),
        "nz": int(field.u_ms.shape[0]),
        "ordering": "z,y,x",
        "stride": {"x": args.stride_x, "y": args.stride_y, "z": args.stride_z},
        "spacing_m": {"approx_dx": float(dx), "approx_dy": float(dy)},
    }
    extent = {
        "east_m": [float(np.min(field.east_m)), float(np.max(field.east_m))],
        "north_m": [float(np.min(field.north_m)), float(np.max(field.north_m))],
        "agl_m": [float(np.min(field.heights_agl_m)), float(np.max(field.heights_agl_m))],
    }
    stats = {
        "wind_speed_min": float(speed.min()),
        "wind_speed_max": float(speed.max()),
        "wind_speed_mean": float(speed.mean()),
        "wind_speed_std": float(speed.std()),
        "surface_speed_mean": float(np.sqrt(field.u_ms[0] ** 2 + field.v_ms[0] ** 2).mean()),
    }
    return {
        "source_file": str(Path(args.wrf_path).resolve()),
        "time_index": args.time_index,
        "timestamp": field.timestamp.isoformat(),
        "origin_lat_lon": {"lat": field.lat0, "lon": field.lon0},
        "grid": grid_info,
        "extent_m": extent,
        "wind_stats": stats,
    }


def export_csv(path: Path, field: WindFieldData) -> None:
    nx, ny, nz = field.shape
    with path.open("w", newline="") as csv_file:
        csv_file.write("# wind_field_csv,v1\n")
        csv_file.write(f"# timestamp={field.timestamp.isoformat()}\n")
        csv_file.write(f"# origin_lat={field.lat0},origin_lon={field.lon0}\n")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "iz",
                "iy",
                "ix",
                "east_m",
                "north_m",
                "terrain_m",
                "height_m",
                "height_agl_m",
                "u_ms",
                "v_ms",
                "w_ms",
            ]
        )
        for iz in range(nz):
            for iy in range(ny):
                for ix in range(nx):
                    writer.writerow(
                        [
                            iz,
                            iy,
                            ix,
                            float(field.east_m[iy, ix]),
                            float(field.north_m[iy, ix]),
                            float(field.terrain_m[iy, ix]),
                            float(field.heights_m[iz, iy, ix]),
                            float(field.heights_agl_m[iz, iy, ix]),
                            float(field.u_ms[iz, iy, ix]),
                            float(field.v_ms[iz, iy, ix]),
                            float(field.w_ms[iz, iy, ix]),
                        ]
                    )


def main() -> None:
    args = parse_args()
    args.wrf_path = Path(args.wrf_path)
    args.output_dir = Path(args.output_dir)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    dtype = np.float32 if args.float32 else np.float64

    with Dataset(args.wrf_path) as ds:
        field = load_time_slice(ds, args, dtype=dtype)

    summary = summarize_field(field, args)
    time_suffix = f"{args.time_index:02d}"
    data_path = args.output_dir / f"wind_field_t{time_suffix}.csv"
    meta_path = args.output_dir / f"wind_field_t{time_suffix}.json"

    export_csv(data_path, field)
    meta_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False))

    print(f"Exported wind field for time-step {args.time_index} -> {data_path}")
    print(f"Metadata summary: {meta_path}")
    print(
        f"Grid {summary['grid']['nx']}x{summary['grid']['ny']}x{summary['grid']['nz']} "
        f"cells, approx spacing dx={summary['grid']['spacing_m']['approx_dx']:.1f} m, "
        f"dy={summary['grid']['spacing_m']['approx_dy']:.1f} m"
    )
    print(
        f"Wind speed range {summary['wind_stats']['wind_speed_min']:.2f} .. "
        f"{summary['wind_stats']['wind_speed_max']:.2f} m/s "
        f"(mean {summary['wind_stats']['wind_speed_mean']:.2f} m/s)"
    )


if __name__ == "__main__":
    main()
