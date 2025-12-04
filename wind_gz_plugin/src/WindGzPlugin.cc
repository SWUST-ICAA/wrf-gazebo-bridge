#include "WindGzPlugin.hh"

#include <chrono>
#include <cmath>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

using namespace std::chrono_literals;

namespace wrf_gz
{

struct WindGzPlugin::Data
{
  // Gazebo model / link
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  std::string linkName{"base_link"};

  // ROS 2
  std::shared_ptr<rclcpp::Node> rosNode;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr windSub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPub;
  std::thread rosSpinThread;
  std::atomic<bool> stopRos{false};

  // Most recent wind vector received (world frame)
  std::mutex windMutex;
  gz::math::Vector3d windWorld{0, 0, 0};
  bool haveWind{false};

  // Reference geodetic coordinates (lat/lon/alt corresponding to world origin)
  double refLatDeg{0.0};
  double refLonDeg{0.0};
  double refAlt{0.0};

  // Drag parameters
  double fluidDensity{1.225};   // kg/m^3
  double referenceArea{1.0};    // m^2
  double linearCd{1.0};         // Translational drag coefficient (dimensionless)
  double angularCd{1.0};        // Rotational drag coefficient (dimensionless)

  // Safety limits for force / torque to avoid numerical blow-up in the physics engine
  double maxForce{1e3};         // N
  double maxTorque{1e3};        // N·m
};

//////////////////////////////////////////////////
WindGzPlugin::WindGzPlugin()
  : dataPtr(std::make_unique<WindGzPlugin::Data>())
{
}

//////////////////////////////////////////////////
WindGzPlugin::~WindGzPlugin()
{
  if (this->dataPtr)
  {
    this->dataPtr->stopRos = true;
    if (this->dataPtr->rosSpinThread.joinable())
    {
      this->dataPtr->rosSpinThread.join();
    }
  }
}

//////////////////////////////////////////////////
void WindGzPlugin::Configure(const gz::sim::Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             gz::sim::EntityComponentManager &_ecm,
                             gz::sim::EventManager &)
{
  this->dataPtr->model = gz::sim::Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    std::cerr << "[WindGzPlugin] Model is not valid.\n";
    return;
  }

  // Read SDF parameters
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  if (sdf->HasElement("link_name"))
  {
    this->dataPtr->linkName = sdf->Get<std::string>("link_name");
  }

  if (sdf->HasElement("reference_latitude"))
  {
    this->dataPtr->refLatDeg = sdf->Get<double>("reference_latitude");
  }
  if (sdf->HasElement("reference_longitude"))
  {
    this->dataPtr->refLonDeg = sdf->Get<double>("reference_longitude");
  }
  if (sdf->HasElement("reference_altitude"))
  {
    this->dataPtr->refAlt = sdf->Get<double>("reference_altitude");
  }

  if (sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = sdf->Get<double>("fluid_density");
  }
  if (sdf->HasElement("reference_area"))
  {
    this->dataPtr->referenceArea = sdf->Get<double>("reference_area");
  }
  if (sdf->HasElement("linear_drag_coefficient"))
  {
    this->dataPtr->linearCd = sdf->Get<double>("linear_drag_coefficient");
  }
  if (sdf->HasElement("angular_drag_coefficient"))
  {
    this->dataPtr->angularCd = sdf->Get<double>("angular_drag_coefficient");
  }

  // Optional: limit max force / torque to avoid divergence due to extreme coefficients
  if (sdf->HasElement("max_force"))
  {
    this->dataPtr->maxForce = sdf->Get<double>("max_force");
  }
  if (sdf->HasElement("max_torque"))
  {
    this->dataPtr->maxTorque = sdf->Get<double>("max_torque");
  }

  // Look up the link entity
  this->dataPtr->linkEntity =
    this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);

  if (this->dataPtr->linkEntity == gz::sim::kNullEntity)
  {
    std::cerr << "[WindGzPlugin] Failed to find link ["
              << this->dataPtr->linkName << "].\n";
    return;
  }

  // Enable velocity checks so WorldLinearVelocity / WorldAngularVelocity components exist
  {
    gz::sim::Link link(this->dataPtr->linkEntity);
    link.EnableVelocityChecks(_ecm, true);
  }

  // Initialize ROS 2
  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  this->dataPtr->rosNode = std::make_shared<rclcpp::Node>("wind_gz_plugin");

  // Subscribe to /wrf_wind
  this->dataPtr->windSub =
    this->dataPtr->rosNode->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/wrf_wind", 10,
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->windMutex);
        this->dataPtr->windWorld.Set(msg->vector.x, msg->vector.y, msg->vector.z);
        this->dataPtr->haveWind = true;
      });

  // Publisher for /robot_gpsfix
  this->dataPtr->gpsPub =
    this->dataPtr->rosNode->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/robot_gpsfix", 10);

  // Dedicated ROS spin thread
  this->dataPtr->rosSpinThread =
    std::thread(
      [this]()
      {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->dataPtr->rosNode);
        while (rclcpp::ok() && !this->dataPtr->stopRos)
        {
          exec.spin_some();
          std::this_thread::sleep_for(10ms);
        }
      });

  std::cout << "[WindGzPlugin] Configured. Link: " << this->dataPtr->linkName
            << ", ref lat/lon/alt: " << this->dataPtr->refLatDeg << ", "
            << this->dataPtr->refLonDeg << ", " << this->dataPtr->refAlt
            << std::endl;
}

//////////////////////////////////////////////////
void WindGzPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->dataPtr->linkEntity == gz::sim::kNullEntity)
    return;

  gz::sim::Link link(this->dataPtr->linkEntity);

  // Use Link helpers to get world pose and velocities (std::optional)
  auto poseOpt = link.WorldPose(_ecm);
  auto velOpt = link.WorldLinearVelocity(_ecm);
  auto angVelOpt = link.WorldAngularVelocity(_ecm);

  // Without pose we cannot do anything
  if (!poseOpt)
    return;

  const auto &pose = *poseOpt;
  // Velocities are only needed for drag; if missing we assume 0 but still publish GPS
  gz::math::Vector3d velWorld(0, 0, 0);
  if (velOpt)
    velWorld = *velOpt;

  gz::math::Vector3d angVelWorld(0, 0, 0);
  if (angVelOpt)
    angVelWorld = *angVelOpt;

  // Current wind vector in world frame
  gz::math::Vector3d windWorld(0, 0, 0);
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->windMutex);
    if (this->dataPtr->haveWind)
      windWorld = this->dataPtr->windWorld;
  }

  // Relative wind velocity (body velocity minus wind)
  gz::math::Vector3d vRel = velWorld - windWorld;
  const double speed = vRel.Length();

  gz::math::Vector3d force(0, 0, 0);
  if (speed > 1e-3)
  {
    // F = -0.5 * rho * Cd * A * |v_rel| * v_rel
    const double k =
      0.5 * this->dataPtr->fluidDensity *
      this->dataPtr->linearCd *
      this->dataPtr->referenceArea;
    force = -k * speed * vRel;
  }

  // Rotational drag torque (simple quadratic damping)
  const double omegaMag = angVelWorld.Length();
  gz::math::Vector3d torque(0, 0, 0);
  if (omegaMag > 1e-3)
  {
    // M = -0.5 * rho * Cd_rot * A_ref * |omega| * omega
    const double kR =
      0.5 * this->dataPtr->fluidDensity *
      this->dataPtr->angularCd *
      this->dataPtr->referenceArea;
    torque = -kR * omegaMag * angVelWorld;
  }

  // Clamp force / torque to safe limits to avoid extreme values crashing ODE/physics
  auto sanitizeVec = [](const gz::math::Vector3d &v, double maxMag) {
    gz::math::Vector3d out = v;
    if (!std::isfinite(out.X()) || !std::isfinite(out.Y()) || !std::isfinite(out.Z()))
      return gz::math::Vector3d::Zero;

    if (maxMag > 0.0)
    {
      const double mag = out.Length();
      if (mag > maxMag && mag > 1e-6)
      {
        out *= (maxMag / mag);
      }
    }
    return out;
  };

  force = sanitizeVec(force, this->dataPtr->maxForce);
  torque = sanitizeVec(torque, this->dataPtr->maxTorque);

  // Apply drag force and torque to the link in world coordinates
  if (force != gz::math::Vector3d::Zero ||
      torque != gz::math::Vector3d::Zero)
  {
    link.AddWorldWrench(_ecm, force, torque);
  }

  // Compute and publish GPS (lat/lon/alt)
  if (this->dataPtr->gpsPub && this->dataPtr->rosNode)
  {
    const double earthRadius = 6378137.0;  // m

    const auto &pos = pose.Pos();  // World position (x, y, z), meters

    // Assume world frame ENU: x east, y north, z up
    const double dEast = pos.X();
    const double dNorth = pos.Y();
    const double alt = this->dataPtr->refAlt + pos.Z();

    const double lat0Rad = this->dataPtr->refLatDeg * M_PI / 180.0;
    const double lon0Rad = this->dataPtr->refLonDeg * M_PI / 180.0;

    const double latRad = lat0Rad + dNorth / earthRadius;
    const double lonRad =
      lon0Rad + dEast / (earthRadius * std::cos(lat0Rad));

    const double latDeg = latRad * 180.0 / M_PI;
    const double lonDeg = lonRad * 180.0 / M_PI;

    sensor_msgs::msg::NavSatFix gpsMsg;
    gpsMsg.header.stamp = this->dataPtr->rosNode->now();
    gpsMsg.header.frame_id = "world";  // Frame id can be adjusted as needed

    gpsMsg.latitude = latDeg;
    gpsMsg.longitude = lonDeg;
    gpsMsg.altitude = alt;

    gpsMsg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gpsMsg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gpsMsg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    this->dataPtr->gpsPub->publish(gpsMsg);
  }
}

}  // namespace wrf_gz
