"""
Simple utility script to inspect a WRF ``wrfout`` NetCDF file.

Features:
- Read a WRF ``wrfout`` NetCDF file
- Print latitude / longitude range and domain center
- Print vertical height range (m)
- Estimate horizontal domain extent (north-south / east-west, meters)
- Compute basic wind statistics (min / max / mean speed)

Example usage:

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
            "Failed to import netCDF4, please install the dependency first:\n"
            "  sudo apt install python3-netcdf4\n"
            "or:\n"
            "  pip install netCDF4\n"
            f"Import error: {exc}",
            file=sys.stderr,
        )
        raise

    return netCDF4.Dataset(path, mode="r")


def _latlon_info(nc) -> Tuple[float, float, float, float, float, float, float, float]:
    """Extract lat / lon range and center from XLAT / XLONG and estimate horizontal extent (m)."""
    import numpy as np  # type: ignore

    xlat = nc.variables["XLAT"][0, :, :]  # (south_north, west_east)
    xlon = nc.variables["XLONG"][0, :, :]

    lat_min = float(np.min(xlat))
    lat_max = float(np.max(xlat))
    lon_min = float(np.min(xlon))
    lon_max = float(np.max(xlon))

    lat_center = 0.5 * (lat_min + lat_max)
    lon_center = 0.5 * (lon_min + lon_max)

    # Use a simple spherical Earth model to estimate north-south / east-west extents
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
    """Compute height range (m) of mass levels using PH / PHB."""
    import numpy as np  # type: ignore

    if "PH" not in nc.variables or "PHB" not in nc.variables:
        return float("nan"), float("nan")

    ph = nc.variables["PH"][:]  # (Time, bottom_top_stag, ny, nx)
    phb = nc.variables["PHB"][:]
    g = 9.81
    z_stag = (ph + phb) / g  # geopotential height at W levels
    # Mass-level height: average between adjacent W levels
    z_mass = 0.5 * (z_stag[:, :-1, :, :] + z_stag[:, 1:, :, :])

    z_min = float(np.min(z_mass))
    z_max = float(np.max(z_mass))
    return z_min, z_max


def _center_altitude(
    nc, lat_center: float, lon_center: float
) -> Tuple[float | None, float | None]:
    """Compute terrain height and lowest mass-level height (m) near the lat/lon center.

    Returns:
      (terrain_alt, lowest_mass_level_alt)
      Values are ``None`` if the corresponding variables are not present.
    """
    import numpy as np  # type: ignore

    xlat = nc.variables["XLAT"][0, :, :]
    xlon = nc.variables["XLONG"][0, :, :]

    # Find the grid point closest to the domain center
    lat0_rad = math.radians(lat_center)
    dlat = xlat - lat_center
    dlon = (xlon - lon_center) * math.cos(lat0_rad)
    dist2 = dlat * dlat + dlon * dlon

    idx_flat = int(np.argmin(dist2))
    j, i = np.unravel_index(idx_flat, xlat.shape)

    terrain_alt: float | None = None
    lowest_mass_alt: float | None = None

    # Terrain height HGT (if available)
    if "HGT" in nc.variables:
        hgt = nc.variables["HGT"][0, :, :]
        terrain_alt = float(hgt[j, i])

    # Lowest mass-level height from PH/PHB (if available)
    if "PH" in nc.variables and "PHB" in nc.variables:
        ph = nc.variables["PH"][:]  # (Time, bottom_top_stag, ny, nx)
        phb = nc.variables["PHB"][:]
        g = 9.81
        z_stag = (ph + phb) / g
        z_mass = 0.5 * (z_stag[:, :-1, :, :] + z_stag[:, 1:, :, :])
        lowest_mass_alt = float(z_mass[0, 0, j, i])

    return terrain_alt, lowest_mass_alt


def _wind_stats(nc) -> Tuple[float, float, float]:
    """Compute min / max / mean wind speed (m/s). Prefer 3D U/V/W if available."""
    import numpy as np  # type: ignore

    if "U" in nc.variables and "V" in nc.variables:
        # 3D wind field
        u_raw = nc.variables["U"][:]  # (Time, bottom_top, ny, nx_stag)
        v_raw = nc.variables["V"][:]  # (Time, bottom_top, ny_stag, nx)

        # Remove staggering in U/V to get wind at mass points
        u_mass = 0.5 * (u_raw[:, :, :, :-1] + u_raw[:, :, :, 1:])
        v_mass = 0.5 * (v_raw[:, :, :-1, :] + v_raw[:, :, 1:, :])

        if "W" in nc.variables:
            w_raw = nc.variables["W"][:]  # (Time, bottom_top_stag, ny, nx)
            w_mass = 0.5 * (w_raw[:, :-1, :, :] + w_raw[:, 1:, :, :])
        else:
            w_mass = np.zeros_like(u_mass)

        speed = np.sqrt(u_mass ** 2 + v_mass ** 2 + w_mass ** 2)
    elif "U10" in nc.variables and "V10" in nc.variables:
        # Fallback: 10 m wind only
        u10 = nc.variables["U10"][:]  # (Time, ny, nx)
        v10 = nc.variables["V10"][:]
        speed = np.sqrt(u10 ** 2 + v10 ** 2)
    else:
        raise RuntimeError(
            "Could not find U/V or U10/V10 variables in the wrf file; "
            "cannot compute wind speed statistics."
        )

    v_min = float(np.min(speed))
    v_max = float(np.max(speed))
    v_mean = float(np.mean(speed))
    return v_min, v_max, v_mean


def summarize_wrf(path: str) -> None:
    """Read a WRF file and print key domain and wind statistics."""
    nc = _load_nc(path)

    # Lat/lon and horizontal extent
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

    # Height range
    z_min, z_max = _height_info(nc)

    # Terrain / lowest mass-level height at domain center
    center_terrain_alt, center_lowest_mass_alt = _center_altitude(
        nc, lat_center, lon_center
    )

    # Wind statistics
    v_min, v_max, v_mean = _wind_stats(nc)

    print("WRF file summary")
    print("----------------")
    print(f"File path: {path}")
    print()

    print("Latitude / longitude range:")
    print(f"  Latitude: {lat_min:.4f}°  ~  {lat_max:.4f}°")
    print(f"  Longitude: {lon_min:.4f}°  ~  {lon_max:.4f}°")
    print(f"  Center: ({lat_center:.4f}°, {lon_center:.4f}°)")
    print()

    print("Horizontal extent (approximate, meters):")
    print(f"  North-south extent: {north_south_extent:.1f} m")
    print(f"  East-west extent:   {east_west_extent:.1f} m")
    print()

    print("Height range (mass levels, meters):")
    if math.isnan(z_min) or math.isnan(z_max):
        print("  PH/PHB variables not found; cannot compute height range.")
    else:
        print(f"  Height: {z_min:.1f} m  ~  {z_max:.1f} m")
    print()

    print("Heights at domain center (meters):")
    if center_terrain_alt is not None:
        print(f"  Terrain height HGT:         {center_terrain_alt:.1f} m")
    else:
        print("  HGT variable not found; terrain height unavailable.")

    if center_lowest_mass_alt is not None:
        print(f"  Lowest mass-level height:   {center_lowest_mass_alt:.1f} m")
    else:
        print("  PH/PHB variables not found; lowest mass level unavailable.")
    print()

    print("Wind speed statistics (m/s):")
    print(f"  Min speed:   {v_min:.3f} m/s")
    print(f"  Max speed:   {v_max:.3f} m/s")
    print(f"  Mean speed:  {v_mean:.3f} m/s")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Read a WRF wrfout file and print key geospatial and wind information.",
    )
    parser.add_argument(
        "--wrf-file",
        required=True,
        help="Path to wrfout NetCDF file (.nc or wrfout_d01_...).",
    )

    args = parser.parse_args(argv)
    wrf_path = os.path.expanduser(args.wrf_file)

    if not os.path.isfile(wrf_path):
        print(f"Error: file not found: {wrf_path}", file=sys.stderr)
        sys.exit(1)

    try:
        summarize_wrf(wrf_path)
    except Exception as exc:  # pragma: no cover
        print(f"Error while analyzing WRF file: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":  # pragma: no cover
    main()
