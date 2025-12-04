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
/// - Attach inside a <model> tag
/// - Subscribe to ROS 2 topic /wrf_wind (geometry_msgs::msg::Vector3Stamped)
/// - Publish current robot latitude / longitude / altitude to /robot_gpsfix (sensor_msgs::msg::NavSatFix)
/// - Compute translational and rotational aerodynamic drag based on relative wind and angular velocity,
///   and apply the resulting wrench to the model
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

// Register as a Gazebo System plugin
GZ_ADD_PLUGIN(
  wrf_gz::WindGzPlugin,
  gz::sim::System,
  wrf_gz::WindGzPlugin::ISystemConfigure,
  wrf_gz::WindGzPlugin::ISystemPreUpdate)
