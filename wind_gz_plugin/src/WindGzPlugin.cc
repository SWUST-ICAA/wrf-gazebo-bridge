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
#include <gz/sim/components/Geometry.hh>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

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
  rclcpp::Time lastWindStamp{};
  // If no wind message has been received within this timeout, treat as "no wind"
  double windTimeoutSec{1.0};

  // Reference geodetic coordinates (lat/lon/alt corresponding to world origin)
  double refLatDeg{0.0};
  double refLonDeg{0.0};
  double refAlt{0.0};

  // Drag parameters
  double fluidDensity{1.225};    // kg/m^3
  double referenceArea{1.0};     // m^2

  // Isotropic drag coefficients (for backwards compatibility and as defaults)
  double linearCd{1.0};          // Translational drag coefficient (dimensionless)
  double angularCd{1.0};         // Rotational drag coefficient (dimensionless)

  // Body-frame anisotropic drag coefficients (per-axis in body frame).
  // These are derived from the scalar coefficients above unless explicitly
  // overridden via SDF parameters such as linear_drag_coefficient_x, etc.
  gz::math::Vector3d linearCdBody{1.0, 1.0, 1.0};
  gz::math::Vector3d angularCdBody{0.5, 0.5, 0.5};

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

  // Drag-related parameters
  //
  // If the SDF file explicitly specifies these elements, use the provided
  // values. Otherwise, fall back to automatic estimation / sensible defaults
  // aimed at multirotor-style vehicles.
  if (sdf->HasElement("fluid_density"))
  {
    // User-provided constant air density
    this->dataPtr->fluidDensity = sdf->Get<double>("fluid_density");
  }
  else
  {
    // Approximate ISA density at the reference altitude (troposphere model).
    // This keeps density realistic without requiring manual tuning.
    const double h = this->dataPtr->refAlt;           // m
    const double rho0 = 1.225;                        // kg/m^3 at sea level
    const double T0 = 288.15;                         // K
    const double L = 0.0065;                          // K/m
    const double g = 9.80665;                         // m/s^2
    const double R = 287.058;                         // J/(kg·K)

    if (h > 0.0 && h < 11000.0)
    {
      const double term = 1.0 - L * h / T0;
      if (term > 0.0)
      {
        const double exponent = g / (R * L) - 1.0;
        this->dataPtr->fluidDensity = rho0 * std::pow(term, exponent);
      }
      else
      {
        this->dataPtr->fluidDensity = rho0 * 0.5;
      }
    }
    else
    {
      // At very low or very high altitudes, just keep the sea level value.
      this->dataPtr->fluidDensity = rho0;
    }
  }

  if (sdf->HasElement("reference_area"))
  {
    // Respect explicitly configured reference area.
    this->dataPtr->referenceArea = sdf->Get<double>("reference_area");
  }
  else
  {
    // Try to infer a reasonable reference area from the link's collision
    // geometry. For multirotors this will effectively approximate the sum
    // of rotor disk areas when possible.
    this->dataPtr->referenceArea = this->EstimateReferenceArea(_ecm);
  }

  if (sdf->HasElement("linear_drag_coefficient"))
  {
    this->dataPtr->linearCd = sdf->Get<double>("linear_drag_coefficient");
  }
  else
  {
    // Default translational drag coefficient suitable for a bluff body /
    // multirotor fuselage. Users can override per model in SDF.
    this->dataPtr->linearCd = 1.0;
  }

  if (sdf->HasElement("angular_drag_coefficient"))
  {
    this->dataPtr->angularCd = sdf->Get<double>("angular_drag_coefficient");
  }
  else
  {
    // Default rotational damping coefficient; smaller than translational
    // drag to avoid overly strong attitude damping.
    this->dataPtr->angularCd = 0.5;
  }

  // Initialize body-frame drag coefficients from the scalar defaults
  this->dataPtr->linearCdBody.Set(
    this->dataPtr->linearCd,
    this->dataPtr->linearCd,
    this->dataPtr->linearCd);
  this->dataPtr->angularCdBody.Set(
    this->dataPtr->angularCd,
    this->dataPtr->angularCd,
    this->dataPtr->angularCd);

  // Optional per-axis overrides in body frame
  if (sdf->HasElement("linear_drag_coefficient_x"))
  {
    this->dataPtr->linearCdBody.X() =
      sdf->Get<double>("linear_drag_coefficient_x");
  }
  if (sdf->HasElement("linear_drag_coefficient_y"))
  {
    this->dataPtr->linearCdBody.Y() =
      sdf->Get<double>("linear_drag_coefficient_y");
  }
  if (sdf->HasElement("linear_drag_coefficient_z"))
  {
    this->dataPtr->linearCdBody.Z() =
      sdf->Get<double>("linear_drag_coefficient_z");
  }

  if (sdf->HasElement("angular_drag_coefficient_x"))
  {
    this->dataPtr->angularCdBody.X() =
      sdf->Get<double>("angular_drag_coefficient_x");
  }
  if (sdf->HasElement("angular_drag_coefficient_y"))
  {
    this->dataPtr->angularCdBody.Y() =
      sdf->Get<double>("angular_drag_coefficient_y");
  }
  if (sdf->HasElement("angular_drag_coefficient_z"))
  {
    this->dataPtr->angularCdBody.Z() =
      sdf->Get<double>("angular_drag_coefficient_z");
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
        if (this->dataPtr->rosNode)
        {
          this->dataPtr->lastWindStamp = this->dataPtr->rosNode->now();
        }
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

  // Current wind vector in world frame and whether we have a recent wind input
  gz::math::Vector3d windWorld(0, 0, 0);
  bool windActive = false;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->windMutex);
    if (this->dataPtr->haveWind && this->dataPtr->rosNode)
    {
      const auto now = this->dataPtr->rosNode->now();
      double dt = 0.0;
      if (now > this->dataPtr->lastWindStamp)
      {
        dt = (now - this->dataPtr->lastWindStamp).seconds();
      }

      // Only treat wind as active if we have received a message recently.
      if (dt <= this->dataPtr->windTimeoutSec)
      {
        windWorld = this->dataPtr->windWorld;
        windActive = true;
      }
    }
  }

  gz::math::Vector3d force(0, 0, 0);
  gz::math::Vector3d torque(0, 0, 0);

  if (windActive)
  {
    // Body-frame anisotropic drag model.
    // 1) Compute relative wind and angular velocity in the body frame.
    const auto &rot = pose.Rot();

    // Relative linear wind: body velocity minus wind, in body frame
    const gz::math::Vector3d vRelWorld = velWorld - windWorld;
    const gz::math::Vector3d vRelBody = rot.RotateVectorReverse(vRelWorld);
    const double speed = vRelBody.Length();

    // Relative angular velocity in body frame
    const gz::math::Vector3d omegaBody =
      rot.RotateVectorReverse(angVelWorld);
    const double omegaMag = omegaBody.Length();

    // 2) Translational drag in body frame: per-axis coefficients
    if (speed > 1e-3)
    {
      // F_body = -0.5 * rho * A_ref * |v_rel| * diag(Cd_body) * v_rel_body
      const double k =
        0.5 * this->dataPtr->fluidDensity *
        this->dataPtr->referenceArea *
        speed;

      gz::math::Vector3d forceBody(
        -k * this->dataPtr->linearCdBody.X() * vRelBody.X(),
        -k * this->dataPtr->linearCdBody.Y() * vRelBody.Y(),
        -k * this->dataPtr->linearCdBody.Z() * vRelBody.Z());

      // Transform back to world frame
      force = rot.RotateVector(forceBody);
    }

    // 3) Rotational drag torque in body frame: per-axis coefficients
    if (omegaMag > 1e-3)
    {
      // M_body = -0.5 * rho * A_ref * |omega| * diag(Cd_ang_body) * omega_body
      const double kR =
        0.5 * this->dataPtr->fluidDensity *
        this->dataPtr->referenceArea *
        omegaMag;

      gz::math::Vector3d torqueBody(
        -kR * this->dataPtr->angularCdBody.X() * omegaBody.X(),
        -kR * this->dataPtr->angularCdBody.Y() * omegaBody.Y(),
        -kR * this->dataPtr->angularCdBody.Z() * omegaBody.Z());

      // Transform back to world frame
      torque = rot.RotateVector(torqueBody);
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
  }

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

//////////////////////////////////////////////////
double WindGzPlugin::EstimateReferenceArea(
  gz::sim::EntityComponentManager &_ecm) const
{
  // Look up the link entity for estimation. This may be called before
  // Configure() stores linkEntity, so we query the model directly here.
  auto linkEntity =
    this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);

  if (linkEntity == gz::sim::kNullEntity)
  {
    return this->dataPtr->referenceArea;
  }

  gz::sim::Link link(linkEntity);
  const auto collisions = link.Collisions(_ecm);

  if (collisions.empty())
  {
    return this->dataPtr->referenceArea;
  }

  double rotorArea = 0.0;
  double genericArea = 0.0;

  // Minimum radius to consider a sphere / cylinder as a rotor disk (meters).
  const double rotorRadiusThreshold = 0.05;

  for (const auto &collEntity : collisions)
  {
    auto geomComp =
      _ecm.Component<gz::sim::components::Geometry>(collEntity);

    if (!geomComp)
      continue;

    const sdf::Geometry &geom = geomComp->Data();

    switch (geom.Type())
    {
      case sdf::GeometryType::SPHERE:
      {
        auto sphere = geom.SphereShape();
        if (!sphere)
          break;
        const double r = sphere->Radius();
        if (r <= 0.0)
          break;
        const double area = M_PI * r * r;

        if (r > rotorRadiusThreshold)
          rotorArea += area;
        else
          genericArea += area;
        break;
      }

      case sdf::GeometryType::CYLINDER:
      {
        auto cyl = geom.CylinderShape();
        if (!cyl)
          break;
        const double r = cyl->Radius();
        if (r <= 0.0)
          break;
        const double area = M_PI * r * r;

        if (r > rotorRadiusThreshold)
          rotorArea += area;
        else
          genericArea += area;
        break;
      }

      case sdf::GeometryType::BOX:
      {
        auto box = geom.BoxShape();
        if (!box)
          break;
        const auto size = box->Size();

        // Approximate average projected area over principal planes:
        // XY, XZ, YZ.
        const double xy = size.X() * size.Y();
        const double xz = size.X() * size.Z();
        const double yz = size.Y() * size.Z();
        const double area = (xy + xz + yz) / 3.0;

        genericArea += area;
        break;
      }

      default:
        // Other geometry types (mesh, plane, etc.) are ignored here.
        break;
    }
  }

  if (rotorArea > 0.0)
  {
    return rotorArea;
  }

  if (genericArea > 0.0)
  {
    return genericArea;
  }

  // As a last resort, keep the existing default area.
  return this->dataPtr->referenceArea;
}

}  // namespace wrf_gz
