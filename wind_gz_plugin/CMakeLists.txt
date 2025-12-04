cmake_minimum_required(VERSION 3.10)
project(wind_gz_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(WindGzPlugin SHARED
  src/WindGzPlugin.cc
)

target_include_directories(WindGzPlugin
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(WindGzPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

ament_target_dependencies(WindGzPlugin
  rclcpp
  geometry_msgs
  sensor_msgs
)

install(TARGETS WindGzPlugin
  # Install directly into the Gazebo Harmonic (gz-sim8) system plugin directory,
  # so GZ_SIM_SYSTEM_PLUGIN_PATH does not need to be set manually.
  #
  # On Ubuntu x86_64 with gz-sim8 from the official apt repository,
  # the system plugin path is:
  #   /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
  LIBRARY DESTINATION /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins
)
