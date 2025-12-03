#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace wrf_gz
{
/// Gazebo Harmonic System plugin.
/// - 放在 <model> 标签内
/// - 订阅 ROS2 话题 /wrf_wind (geometry_msgs::msg::Vector3Stamped)
/// - 发布机器人当前经纬度和高度到 /robot_gpsfix (sensor_msgs::msg::NavSatFix)
/// - 根据相对风速与角速度计算平动/转动阻力，并施加到模型上
class WindGzPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  WindGzPlugin();
  ~WindGzPlugin() override;

  // Called once when plugin is attached to a model
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  // Called every simulation step
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  struct Data;
  std::unique_ptr<Data> dataPtr;
};

}  // namespace wrf_gz

// 注册为 Gazebo System 插件
GZ_ADD_PLUGIN(
  wrf_gz::WindGzPlugin,
  gz::sim::System,
  wrf_gz::WindGzPlugin::ISystemConfigure,
  wrf_gz::WindGzPlugin::ISystemPreUpdate)

