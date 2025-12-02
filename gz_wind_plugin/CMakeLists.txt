cmake_minimum_required(VERSION 3.8)
project(gz_wind_plugin)

# 查找依赖：Gazebo Sim (Harmonic 对应 gz-sim8)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(WindFieldSystem SHARED WindFieldSystem.cc)

target_link_libraries(WindFieldSystem
  PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

# 安装路径（可选，方便 export）
install(TARGETS WindFieldSystem DESTINATION lib)