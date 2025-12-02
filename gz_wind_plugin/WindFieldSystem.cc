#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

using namespace gz;
using namespace sim;

// 用于存储从 CSV 读取的单点数据
struct WindPoint {
    double east, north, height; // 位置 (m)
    double u, v, w;             // 风速 (m/s)
};

class WindFieldSystem : public System,
                        public ISystemConfigure,
                        public ISystemPreUpdate
{
public:
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override
    {
        // 1. 读取 SDF 参数
        std::string csvPath;
        if (_sdf->HasElement("csv_path"))
            csvPath = _sdf->Get<std::string>("csv_path");
        else
        {
            gzerr << "[WindFieldSystem] Missing <csv_path> parameter!" << std::endl;
            return;
        }

        if (_sdf->HasElement("link_name"))
            this->targetLinkName = _sdf->Get<std::string>("link_name");
        
        // 空气阻力系数 (简单的 F = coeff * v^2)
        if (_sdf->HasElement("drag_coeff"))
            this->dragCoeff = _sdf->Get<double>("drag_coeff");

        // 2. 加载 CSV 数据
        if (!LoadCSV(csvPath)) {
            gzerr << "[WindFieldSystem] Failed to load CSV: " << csvPath << std::endl;
            return;
        }

        this->model = Model(_entity);
        gzmsg << "[WindFieldSystem] Loaded wind field with " << windData.size() << " points." << std::endl;
    }

    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {
        if (_info.paused) return;

        // 1. 初始化：查找目标 Link 的 Entity ID
        if (this->linkEntity == kNullEntity)
        {
            // 如果没有指定 link_name，默认取模型的第一个 link (Canonical Link)
            if (this->targetLinkName.empty()) {
                const auto &links = this->model.Links(_ecm);
                if (!links.empty()) this->linkEntity = links[0];
            } else {
                this->linkEntity = this->model.LinkByName(_ecm, this->targetLinkName);
            }

            if (this->linkEntity == kNullEntity) return;

            // 确保创建必要的组件，以便后续读取 Pose 和 Velocity
            if (!_ecm.Component<components::WorldPose>(this->linkEntity))
                _ecm.CreateComponent(this->linkEntity, components::WorldPose());
            if (!_ecm.Component<components::WorldLinearVelocity>(this->linkEntity))
                _ecm.CreateComponent(this->linkEntity, components::WorldLinearVelocity());
        }

        // 2. 获取机器人当前状态
        auto poseComp = _ecm.Component<components::WorldPose>(this->linkEntity);
        auto velComp = _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);

        if (!poseComp || !velComp) return;

        math::Vector3d pos = poseComp->Data().Pos();
        math::Vector3d velBody = velComp->Data();

        // 3. 查表获取风速 (使用最近邻查找，可升级为插值)
        math::Vector3d windVel = GetWindAt(pos);

        // 4. 计算相对速度 V_rel = V_wind - V_robot
        math::Vector3d relativeVel = windVel - velBody;

        // 5. 计算风力 (简化模型：F = 0.5 * rho * Cd * A * |V| * V)
        // 这里简化为 F = dragCoeff * |V_rel| * V_rel
        double speed = relativeVel.Length();
        if (speed < 0.01) return; // 忽略微小速度

        math::Vector3d force = this->dragCoeff * speed * relativeVel;

        // 6. 施加力到 Gazebo 物理引擎
        // 使用 ExternalWorldWrenchCmd 组件添加外力
        auto setForceMsg = [&](auto &_wrench)
        {
            auto msgForce = _wrench.mutable_force();
            if (msgForce)
                msgs::Set(msgForce, force);
        };

        auto wrenchComp = _ecm.Component<components::ExternalWorldWrenchCmd>(this->linkEntity);
        if (!wrenchComp) {
            components::ExternalWorldWrenchCmd wrench;
            setForceMsg(wrench.Data());
            _ecm.CreateComponent(this->linkEntity, wrench);
        } else {
            // 如果组件已存在，直接修改力。注意：这里是设置新的力，不是累加。
            // Gazebo Sim 在每个时间步结束会清除这个 Component，所以需要每帧设置。
            setForceMsg(wrenchComp->Data()); 
            // 如果需要施加扭矩，也可以设置 wrenchComp->Data().torque
        }
    }

private:
    bool LoadCSV(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) return false;

        std::string line;
        // 跳过 CSV 头部注释和标题行
        // 根据 Python 脚本：前3行是注释，第4行是标题，或者包含 '#'
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#' || line.find("iz,iy,ix") != std::string::npos) continue;
            
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }

            // CSV 格式: iz, iy, ix, east, north, terrain, height, height_agl, u, v, w
            if (row.size() < 11) continue;

            WindPoint p;
            try {
                // Gazebo X = East, Gazebo Y = North
                p.east = std::stof(row[3]);
                p.north = std::stof(row[4]);
                // 使用 height_agl (相对地面高度) 还是 height (海拔)?
                // 如果 Gazebo 地面在 Z=0，建议用 height_agl。这里取 height_agl (index 7)
                p.height = std::stof(row[7]); 
                
                p.u = std::stof(row[8]);
                p.v = std::stof(row[9]);
                p.w = std::stof(row[10]);
                
                windData.push_back(p);
            } catch (...) { continue; }
        }
        return !windData.empty();
    }

    // 简单的最近邻查找 (Nearest Neighbor)
    // 为了性能，建议实际项目中将其优化为 3D Grid 索引数组
    math::Vector3d GetWindAt(const math::Vector3d &pos)
    {
        if (windData.empty()) return math::Vector3d::Zero;

        double minDistSq = std::numeric_limits<double>::max();
        const WindPoint* closest = nullptr;

        // 暴力搜索 (注意：如果数据量很大，这会很慢，需要换成 KD-Tree 或 Grid Index)
        // 考虑到 Python 脚本做了 stride，数据量可能在可接受范围内 (例如 < 10000点)
        // 如果卡顿，请告知，我会提供优化的 Grid 查找代码。
        for (const auto &p : windData) {
            double dx = p.east - pos.X();
            double dy = p.north - pos.Y();
            double dz = p.height - pos.Z();
            double distSq = dx*dx + dy*dy + dz*dz;
            
            if (distSq < minDistSq) {
                minDistSq = distSq;
                closest = &p;
            }
        }

        // 如果距离太远（例如超出了数据覆盖范围），返回 0 风
        if (minDistSq > 500.0 * 500.0) return math::Vector3d::Zero;

        if (closest) return math::Vector3d(closest->u, closest->v, closest->w);
        return math::Vector3d::Zero;
    }

    Model model;
    Entity linkEntity = kNullEntity;
    std::string targetLinkName;
    double dragCoeff = 1.0; // 默认阻力系数
    std::vector<WindPoint> windData;
};

GZ_ADD_PLUGIN(WindFieldSystem,
              gz::sim::System,
              WindFieldSystem::ISystemConfigure,
              WindFieldSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WindFieldSystem, "custom::WindFieldSystem")
