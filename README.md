# wrf-gazebo-bridge

ROS 2 → Gazebo Harmonic bridge based on WRF atmospheric wind fields.

- Read 3D wind from WRF `wrfout` (NetCDF) files
- Interpolate in space and time at the robot's current latitude / longitude / altitude and publish `/wrf_wind`
- Gazebo Harmonic system plugin reads `/wrf_wind` and applies aerodynamic forces and torques to the model
- The plugin also publishes `/robot_gpsfix` (robot latitude / longitude / altitude) which is used as input to the wind interpolation node

Target environment: ROS 2 Humble and Gazebo Harmonic (`gz-sim8`).

---

## Dependencies

- Ubuntu 22.04
- ROS 2 Humble (workspace already initialized)
- Gazebo Harmonic
- Python:
  - `netCDF4`
  - `numpy`

Recommended installation for Python dependencies:

```bash
sudo apt install python3-netcdf4 python3-numpy
```

---

## Layout

- `wrf_gazebo_bridge/`: ROS 2 Python package
  - Reads WRF NetCDF files
  - Subscribes to `/robot_gpsfix` (`sensor_msgs/NavSatFix`)
  - Publishes `/wrf_wind` (`geometry_msgs/Vector3Stamped`)
- `config/wrf_wind_config.yaml`: configuration for the wind node
- `wind_gz_plugin/`: Gazebo Harmonic system plugin (C++)
  - Subscribes to `/wrf_wind` and applies translational / rotational aerodynamic forces
  - Publishes `/robot_gpsfix` (`NavSatFix`)

---

## Build and Install

### 1. Build the ROS 2 package

From the workspace root (for example `~/Work_directory/wrfTOgazebo_ws`):

```bash
colcon build --packages-select wrf_gazebo_bridge
source install/setup.bash
```

### 2. Build the Gazebo plugin

From this repository:

```bash
cd wind_gz_plugin
mkdir -p build
cd build
cmake ..
make
sudo make install
```

`CMakeLists.txt` installs the plugin to:

```text
/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
```

This is the default Gazebo Harmonic (`gz-sim8`) system plugin path on Ubuntu, so you **do not** need to set `GZ_SIM_SYSTEM_PLUGIN_PATH`.

---

## Configure the WRF Wind Node

Configuration file: `config/wrf_wind_config.yaml`, example:

```yaml
wrf_wind_publisher:
  ros__parameters:
    wrf_file_path: "/absolute/path/to/wrfout_d01_2022-07-01_00_00_00"
    position_topic: "/robot_gpsfix"
    wind_topic: "/wrf_wind"
    use_3d_wind: true
    time_interpolation: "linear"
    time_offset_seconds: 0.0
    default_wind_x: 0.0
    default_wind_y: 0.0
    default_wind_z: 0.0
```

Notes:

- It is recommended to use an absolute path for `wrf_file_path` to avoid issues when the working directory changes.
- When the robot position is outside the WRF lat/lon domain, the wind falls back to the default value (usually 0).
- The time axis uses WRF `XTIME` (minutes). The node supports arbitrary time resolution with linear or nearest interpolation.

Run the node:

```bash
ros2 run wrf_gazebo_bridge wrf_wind_publisher \
  --ros-args \
  --params-file $(ros2 pkg prefix wrf_gazebo_bridge)/share/wrf_gazebo_bridge/config/wrf_wind_config.yaml
```

---

## Use the Plugin in Gazebo

### 1. Add the plugin to your model SDF

Inside the `<model>` that should be affected by wind:

```xml
<plugin filename="libWindGzPlugin.so" name="wrf_gz::WindGzPlugin">
  <!-- Link where the aerodynamic wrench is applied -->
  <link_name>base_link</link_name>

  <!-- Geodetic reference point (lat / lon / alt) for the world origin -->
  <reference_latitude>30.0</reference_latitude>
  <reference_longitude>120.0</reference_longitude>
  <reference_altitude>0.0</reference_altitude>

  <!-- Air properties and simple drag coefficients (tune for your model) -->
  <fluid_density>1.225</fluid_density>           <!-- Air density kg/m^3 -->
  <reference_area>1.0</reference_area>           <!-- Reference area m^2 -->
  <linear_drag_coefficient>1.0</linear_drag_coefficient>
  <angular_drag_coefficient>1.0</angular_drag_coefficient>

  <!-- Optional safety limits for the applied wrench -->
  <max_force>200.0</max_force>
  <max_torque>50.0</max_torque>
</plugin>
```

World frame convention:

- `+X` points east
- `+Y` points north
- `+Z` points up

The plugin converts the model position in world coordinates to latitude / longitude / altitude and publishes it on `/robot_gpsfix`.

### 2. Start Gazebo

Example:

```bash
gz sim your_world.sdf
```

Verify that the plugin is loaded by checking the terminal output or via ROS topics:

```bash
ros2 topic echo /robot_gpsfix
ros2 topic echo /wrf_wind
```

When the model moves inside the WRF domain:

- `/robot_gpsfix` latitude / longitude will change with the model pose
- `/wrf_wind` will provide interpolated wind vectors at the current position
- The model will experience translational and rotational aerodynamic drag

---

## Data Flow Overview

1. The Gazebo plugin computes the robot's current geodetic position from its pose and publishes `/robot_gpsfix`.
2. `wrf_wind_publisher` subscribes to `/robot_gpsfix`, interpolates the 3D WRF wind field in space and time, and publishes `/wrf_wind`.
3. The Gazebo plugin subscribes to `/wrf_wind`, computes aerodynamic forces / torques, and applies them to the model.

---

## Troubleshooting

- **Gazebo cannot find the plugin**
  - Ensure `sudo make install` has been run.
  - Check that `/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libWindGzPlugin.so` exists.
- **WRF file cannot be opened**
  - Verify that `wrf_file_path` is a correct absolute path.
  - Make sure you have sourced `install/setup.bash` in your terminal.
- **Wind is always zero**
  - Check that the robot lat/lon is inside the WRF domain.
  - Check `use_3d_wind` and time interpolation parameters in the YAML config.
