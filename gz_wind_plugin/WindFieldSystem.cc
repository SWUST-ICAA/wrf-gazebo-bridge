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
#include <algorithm>

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

        double minEast = std::numeric_limits<double>::max();
        double maxEast = std::numeric_limits<double>::lowest();
        double minNorth = std::numeric_limits<double>::max();
        double maxNorth = std::numeric_limits<double>::lowest();
        double minHeight = std::numeric_limits<double>::max();
        double maxHeight = std::numeric_limits<double>::lowest();
        size_t maxIx = 0, maxIy = 0, maxIz = 0;
        bool seenIx = false, seenIy = false, seenIz = false;

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
                const int iz = std::stoi(row[0]);
                const int iy = std::stoi(row[1]);
                const int ix = std::stoi(row[2]);

                p.east = std::stod(row[3]);
                p.north = std::stod(row[4]);
                // 使用 height_agl (相对地面高度) 还是 height (海拔)?
                // 如果 Gazebo 地面在 Z=0，建议用 height_agl。这里取 height_agl (index 7)
                p.height = std::stod(row[7]); 
                
                p.u = std::stod(row[8]);
                p.v = std::stod(row[9]);
                p.w = std::stod(row[10]);
                
                windData.push_back(p);

                minEast = std::min(minEast, p.east);
                maxEast = std::max(maxEast, p.east);
                minNorth = std::min(minNorth, p.north);
                maxNorth = std::max(maxNorth, p.north);
                minHeight = std::min(minHeight, p.height);
                maxHeight = std::max(maxHeight, p.height);

                if (ix >= 0) {
                    maxIx = std::max(maxIx, static_cast<size_t>(ix));
                    seenIx = true;
                }
                if (iy >= 0) {
                    maxIy = std::max(maxIy, static_cast<size_t>(iy));
                    seenIy = true;
                }
                if (iz >= 0) {
                    maxIz = std::max(maxIz, static_cast<size_t>(iz));
                    seenIz = true;
                }
            } catch (...) { continue; }
        }
        if (windData.empty()) return false;

        this->bboxMin = math::Vector3d(minEast, minNorth, minHeight);
        this->bboxMax = math::Vector3d(maxEast, maxNorth, maxHeight);

        auto computePadding = [](double span, size_t count, double minPad)
        {
            if (count > 1 && span > 0.0) {
                double spacing = span / static_cast<double>(count - 1);
                return std::max(0.5 * spacing, minPad);
            }
            return minPad;
        };

        constexpr double kHorizontalMinPad = 100.0;
        constexpr double kVerticalMinPad = 50.0;

        size_t nx = seenIx ? maxIx + 1 : 0;
        size_t ny = seenIy ? maxIy + 1 : 0;
        size_t nz = seenIz ? maxIz + 1 : 0;

        this->bboxPadding = math::Vector3d(
            computePadding(maxEast - minEast, nx, kHorizontalMinPad),
            computePadding(maxNorth - minNorth, ny, kHorizontalMinPad),
            computePadding(maxHeight - minHeight, nz, kVerticalMinPad));
        this->hasBounds = true;
        return true;
    }

    // 简单的最近邻查找 (Nearest Neighbor)
    // 为了性能，建议实际项目中将其优化为 3D Grid 索引数组
    math::Vector3d GetWindAt(const math::Vector3d &pos)
    {
        if (windData.empty()) return math::Vector3d::Zero;
        if (!InsideBounds(pos)) return math::Vector3d::Zero;

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

        if (closest) return math::Vector3d(closest->u, closest->v, closest->w);
        return math::Vector3d::Zero;
    }

    bool InsideBounds(const math::Vector3d &pos) const
    {
        if (!this->hasBounds) return true;
        const math::Vector3d min = this->bboxMin - this->bboxPadding;
        const math::Vector3d max = this->bboxMax + this->bboxPadding;
        return (pos.X() >= min.X() && pos.X() <= max.X() &&
                pos.Y() >= min.Y() && pos.Y() <= max.Y() &&
                pos.Z() >= min.Z() && pos.Z() <= max.Z());
    }

    Model model;
    Entity linkEntity = kNullEntity;
    std::string targetLinkName;
    double dragCoeff = 1.0; // 默认阻力系数
    std::vector<WindPoint> windData;
    math::Vector3d bboxMin{0, 0, 0};
    math::Vector3d bboxMax{0, 0, 0};
    math::Vector3d bboxPadding{0, 0, 0};
    bool hasBounds{false};
};

GZ_ADD_PLUGIN(WindFieldSystem,
              gz::sim::System,
              WindFieldSystem::ISystemConfigure,
              WindFieldSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WindFieldSystem, "custom::WindFieldSystem")
