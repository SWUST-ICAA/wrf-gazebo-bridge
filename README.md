# wrf-gazebo-bridge

基于 WRF 气象风场的 ROS 2 → Gazebo Harmonic 联合仿真桥接。

- 从 WRF `wrfout`（NetCDF）文件中读取 3D 风场
- 根据机器人当前经纬度和高度做时空插值，发布风矢量话题 `/wrf_wind`
- Gazebo Harmonic 系统插件读取 `/wrf_wind`，计算气动力和阻力矩作用到模型
- 插件同时发布机器人经纬度与高度 `/robot_gpsfix`，作为风场插值输入

适配 ROS 2 Humble 与 Gazebo Harmonic (`gz-sim8`)。

---

## 依赖环境

- Ubuntu 22.04
- ROS 2 Humble（已初始化工作空间）
- Gazebo Harmonic
- Python 依赖：
  - `netCDF4`
  - `numpy`

推荐安装方式（如果尚未安装）：

```bash
sudo apt install python3-netcdf4 python3-numpy
```

---

## 工程结构

- `wrf_gazebo_bridge/`：ROS 2 Python 包，负责
  - 读取 WRF NetCDF 文件
  - 订阅机器人位置 `/robot_gpsfix`（`sensor_msgs/NavSatFix`）
  - 发布风矢量 `/wrf_wind`（`geometry_msgs/Vector3Stamped`）
- `config/wrf_wind_config.yaml`：WRF 风场节点配置
- `wind_gz_plugin/`：Gazebo Harmonic 系统插件（C++）
  - 订阅 `/wrf_wind`，计算平动/转动气动力
  - 发布 `/robot_gpsfix`（`NavSatFix`）

---

## 编译与安装

### 1. 编译 ROS 2 包

在工作空间根目录（例如 `~/Work_directory/wrfTOgazebo_ws`）：

```bash
colcon build --packages-select wrf_gazebo_bridge
source install/setup.bash
```

### 2. 编译 Gazebo 插件

在本仓库内：

```bash
cd wind_gz_plugin
mkdir -p build
cd build
cmake ..
make 
sudo make install
```

`CMakeLists.txt` 已固定将插件安装到：

```text
/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
```

这是 Ubuntu 上 `gz-sim8` 的系统插件默认搜索路径，因此**无需**额外设置 `GZ_SIM_SYSTEM_PLUGIN_PATH`。

---

## 配置 WRF 风场节点

配置文件：`config/wrf_wind_config.yaml`，示例：

```yaml
wrf_wind_publisher:
  ros__parameters:
    wrf_file_path: "/绝对路径/到/wrfout_d01_2022-07-01_00_00_00"
    position_topic: "/robot_gpsfix"
    wind_topic: "/wrf_wind"
    use_3d_wind: true
    time_interpolation: "linear"
    time_offset_seconds: 0.0
    default_wind_x: 0.0
    default_wind_y: 0.0
    default_wind_z: 0.0
```

注意：

- `wrf_file_path` 建议使用绝对路径，避免运行目录变化导致文件找不到。
- 当机器人位置经纬度超出 WRF 网格范围时，风场自动退化为默认值（通常为 0）。
- 时间轴使用 WRF `XTIME`（单位为分钟），节点内部支持任意时间分辨率的线性插值或最近邻。

启动节点：

```bash
ros2 run wrf_gazebo_bridge wrf_wind_publisher \
  --ros-args \
  --params-file $(ros2 pkg prefix wrf_gazebo_bridge)/share/wrf_gazebo_bridge/config/wrf_wind_config.yaml
```

---

## 在 Gazebo 中使用插件

### 1. 在模型 SDF 中加入插件

在需要受风影响的 `<model>` 内添加：

```xml
<plugin filename="libWindGzPlugin.so" name="wrf_gz::WindGzPlugin">
  <!-- Gazebo 模型中用来施加力的链接名称 -->
  <link_name>base_link</link_name>

  <!-- 场景原点对应的地理参考点（经纬度与高度） -->
  <reference_latitude>30.0</reference_latitude>
  <reference_longitude>120.0</reference_longitude>
  <reference_altitude>0.0</reference_altitude>

  <!-- 空气参数与简单气动系数，可根据模型外形调整 -->
  <fluid_density>1.225</fluid_density>           <!-- 空气密度 kg/m^3 -->
  <reference_area>1.0</reference_area>           <!-- 参考迎风面积 m^2 -->
  <linear_drag_coefficient>1.0</linear_drag_coefficient>
  <angular_drag_coefficient>1.0</angular_drag_coefficient>
</plugin>
```

假定世界坐标系满足：

- `+X` 指向东
- `+Y` 指向北
- `+Z` 指向上

插件内部将模型在世界坐标下的位置转换为经纬度 / 高度，并通过 `/robot_gpsfix` 发布。

### 2. 启动 Gazebo

示例：

```bash
gz sim your_world.sdf
```

确认插件已被加载，可在终端输出中看到类似日志；也可以通过 ROS 话题检查：

```bash
ros2 topic echo /robot_gpsfix
ros2 topic echo /wrf_wind
```

当模型在 WRF 范围内移动时：

- `/robot_gpsfix` 中的经纬度会随模型运动变化
- `/wrf_wind` 会给出对应的插值风矢量
- 模型将受到与相对风速相关的平动阻力和转动阻力矩

---

## 典型数据流

1. Gazebo 插件根据模型姿态计算当前经纬度 / 高度，发布到 `/robot_gpsfix`
2. `wrf_wind_publisher` 订阅 `/robot_gpsfix`，对 WRF 网格做三维空间 + 时间插值，发布 `/wrf_wind`
3. Gazebo 插件订阅 `/wrf_wind`，根据相对风速计算气动力与阻力矩，并施加到模型上

---

## 常见问题

- **Gazebo 找不到插件**
  - 确认已经执行 `sudo make install`
  - 确认系统中存在 `/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/libWindGzPlugin.so`
- **WRF 文件找不到**
  - 检查 `wrf_file_path` 是否为正确的绝对路径
  - 运行节点时确认已经 `source install/setup.bash`
- **风场一直为 0**
  - 确认机器人经纬度在 WRF 网格范围内
  - 确认 `use_3d_wind` 和时间插值参数是否按预期设置

