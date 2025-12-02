# wrf-gazebo-bridge

用于将 WRF（Weather Research and Forecasting）数值预报结果转换为 Gazebo 可读取的风场 CSV，并通过自定义系统插件在仿真中施加风力。

## 目录概览
- `scripts/preprocess_wrf.py`：WRF NetCDF → CSV/JSON 预处理脚本。
- `processed_wind/`：脚本默认输出目录，包含风场 CSV 与统计信息。
- `gz_wind_plugin/`：`WindFieldSystem` Gazebo System 插件源码。
- `weather_data/`：示例或原始气象数据存放位置。

## Python 预处理脚本
1. 安装依赖（Python ≥3.8）：
   ```bash
   pip install -r <(printf "numpy\nnetCDF4\n")
   ```
2. 执行脚本，将 WRF `wrfout*` 转换为稀疏化后的风场栅格（支持步长/时间索引控制）：
   ```bash
   python3 scripts/preprocess_wrf.py \
     --wrf-path weather_data/wrfout_d01_2024-05-01_00:00:00 \
     --output-dir processed_wind \
     --time-index 0 \
     --stride-x 2 --stride-y 2 --stride-z 1
   ```
3. 结果：
   - `processed_wind/*.csv`：包含 `east_m,north_m,height_agl_m,u_ms,v_ms,w_ms` 等列。
   - `processed_wind/*.json`：描述网格大小、步长、统计信息，便于验证。

## 插件编译与安装
> 避免 Conda 等环境变量污染，请使用干净环境变量构建。

```bash
cd gz_wind_plugin
rm -rf build && mkdir build && cd build
env -i HOME="$HOME" LANG="$LANG" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" cmake ..
env -i HOME="$HOME" LANG="$LANG" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" make -j$(nproc)
sudo install -m 755 libWindFieldSystem.so /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/
```

如无权限安装，可改为 `export GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/gz_wind_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH`，让 Gazebo 从自定义位置加载。

## 在 SDF/World 中使用
```xml
<plugin filename="libWindFieldSystem.so" name="custom::WindFieldSystem">
  <csv_path>/absolute/path/to/processed_wind/wind_slice.csv</csv_path>
  <link_name>base_link</link_name>        <!-- 可选，默认取 canonical link -->
  <drag_coeff>0.5</drag_coeff>            <!-- 根据机体受力特性调节 -->
</plugin>
```

Gazebo 每帧会读取机器人位置、查表获得风速，并将计算出的外力写入 `ExternalWorldWrenchCmd` 组件，实现基于 WRF 的三维风场驱动。aligned to final instructions? *** End Patch***}?
