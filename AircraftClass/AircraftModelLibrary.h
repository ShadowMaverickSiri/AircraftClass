#ifndef AIRCRAFT_MODEL_LIBRARY_H
#define AIRCRAFT_MODEL_LIBRARY_H

#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <cmath>
#include <string>

// ============================================================================
// 常量定义
// ============================================================================
namespace Constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double EARTH_RADIUS = 6378137.0;
    constexpr double G = 9.80665;  // 标准重力加速度
}

// 向后兼容性宏
#ifndef M_PI
#define M_PI Constants::PI
#endif

// ============================================================================
// 基础数据结构
// ============================================================================
struct GeoPosition {
    double longitude;  // 经度 (度)
    double latitude;   // 纬度 (度)
    double altitude;   // 高度 (米)
};

struct Velocity3 {
    double north; // 北向速度 (m/s)
    double up;    // 天向速度 (m/s)
    double east;  // 东向速度 (m/s)
};

struct AttitudeAngles {
    double pitch;    // 俯仰角 (弧度)
    double roll;     // 滚转角 (弧度)
    double yaw;      // 偏航角 (弧度)

    AttitudeAngles() : pitch(0.0), roll(0.0), yaw(0.0) {}

    double getPitchDegrees() const { return pitch * Constants::RAD_TO_DEG; }
    double getRollDegrees() const { return roll * Constants::RAD_TO_DEG; }
    double getYawDegrees() const { return yaw * Constants::RAD_TO_DEG; }

    void setPitchDegrees(double degrees) { pitch = degrees * Constants::DEG_TO_RAD; }
    void setRollDegrees(double degrees) { roll = degrees * Constants::DEG_TO_RAD; }
    void setYawDegrees(double degrees) { yaw = degrees * Constants::DEG_TO_RAD; }
};

struct AircraftPerformance {
    double maxTurnRate = 0.5;      // 最大转弯率 (弧度/秒)
    double maxClimbRate = 50.0;     // 最大爬升率 (米/秒)
    double maxRollRate = 2.0;       // 最大滚转率 (弧度/秒)
    double maxPitchRate = 1.0;      // 最大俯仰率 (弧度/秒)
    double maxThrust = 200000.0;    // 最大推力 (牛顿)
    double dragCoefficient = 0.02;  // 阻力系数
    double wingArea = 50.0;         // 机翼面积 (平方米)
    double mass = 10000.0;          // 飞机质量 (千克)
    double maxGForce = 9.0;         // 最大过载 (G)
    double stallSpeed = 80.0;       // 失速速度 (m/s)
    double maxSpeed = 800.0;        // 最大速度 (m/s)
};

// ============================================================================
// 功能模块系统
// ============================================================================
class Aircraft;  // 前向声明

class AircraftModule {
public:
    virtual ~AircraftModule() = default;
    virtual std::string getModuleName() const = 0;
    virtual void update(Aircraft& aircraft, double dt) {}
};

// 示例模块：干扰器
class JammerModule : public AircraftModule {
public:
    std::string getModuleName() const override { return "Jammer"; }
    void activate() { isActive = true; }
    void deactivate() { isActive = false; }
    bool isJamming() const { return isActive; }
    void update(Aircraft& aircraft, double dt) override {
        // 干扰逻辑
    }
private:
    bool isActive = false;
};

// ============================================================================
// 飞行器基类
// ============================================================================
class Aircraft {
public:
    using ManeuverFunc = std::function<void(Aircraft&, double)>;

    Aircraft(const std::string& type, const std::string& model);
    virtual ~Aircraft();

    std::string getType() const { return type; }
    std::string getModel() const { return model; }

    GeoPosition position;
    Velocity3 velocity;
    AttitudeAngles attitude;
    AircraftPerformance performance;

    virtual void updateKinematics(double dt);
    virtual Velocity3 computeAcceleration() const = 0;

    void setManeuver(const std::string& name);
    void performManeuver(double dt);
    static void registerManeuver(const std::string& name, ManeuverFunc func);

    void setReferencePosition(const GeoPosition& refPos);
    GeoPosition getReferencePosition() const { return referencePosition; }
    Velocity3 getECEFPosition() const;
    Velocity3 getLocalNUEPosition() const;
    double getDistanceFromReference() const;
    double getBearingFromReference() const;

    // 模块管理
    void addModule(std::shared_ptr<AircraftModule> module) {
        modules.push_back(module);
    }
    template<typename T>
    std::shared_ptr<T> getModule() const {
        for (const auto& m : modules) {
            auto casted = std::dynamic_pointer_cast<T>(m);
            if (casted) return casted;
        }
        return nullptr;
    }
    void updateModules(double dt) {
        for (auto& m : modules) {
            m->update(*this, dt);
        }
    }

protected:
    std::string type;
    std::string model;
    ManeuverFunc currentManeuver;
    static std::map<std::string, ManeuverFunc> maneuvers;
    GeoPosition referencePosition;
    std::vector<std::shared_ptr<AircraftModule>> modules;
};

// 辅助函数
GeoPosition updateGeoPosition(const GeoPosition& pos, const Velocity3& velocity, double dt);

#endif // AIRCRAFT_MODEL_LIBRARY_H
