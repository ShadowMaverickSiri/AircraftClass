#ifndef KINEMATIC_MANEUVER_SYSTEM_H
#define KINEMATIC_MANEUVER_SYSTEM_H

#include "AircraftModelLibrary.h"
#include <string>
#include <memory>
#include <cmath>

namespace KinematicManeuver {

// ============================================================================
// 机动类型枚举
// ============================================================================
enum class Type {
    LEVEL_TURN,    // 水平转弯
    LOOP,          // 筋斗
    ROLL,          // 横滚
    SPLIT_S        // 半滚倒转
};

// ============================================================================
// 机动参数结构
// ============================================================================
struct Parameters {
    Type type = Type::LEVEL_TURN;
    double targetGForce = 3.0;       // 目标过载 (G)
    double duration = 10.0;          // 机动持续时间 (秒)
    double startTime = 0.0;          // 机动起始时间 (秒)
    double turnDirection = 1.0;      // 转弯方向 (1.0=右转, -1.0=左转)
    double rollDirection = 1.0;      // 滚转方向 (1.0=右滚, -1.0=左滚)

    // 初始状态
    GeoPosition initialPosition;
    Velocity3 initialVelocity;

    // 获取默认参数
    static Parameters getDefault(Type type);
};

// ============================================================================
// 机动模型基类
// ============================================================================
class Model {
public:
    virtual ~Model() = default;

    // 核心接口
    virtual void initialize(const Parameters& params) = 0;
    virtual void update(double currentTime, double dt,
                       GeoPosition& position,
                       Velocity3& velocity,
                       AttitudeAngles& attitude) = 0;
    virtual void reset() = 0;
    virtual std::string getName() const = 0;

    // 状态查询
    virtual double getCurrentGForce() const { return currentGForce; }
    virtual double getProgress() const {
        return (currentTime - params.startTime) / params.duration;
    }
    virtual bool isActive(double currentTime) const;

protected:
    Parameters params;
    double currentTime = 0.0;
    double currentGForce = 1.0;

    // 辅助工具方法
    void applyGForceConstraint(double& targetG);
    double calculateTurnRadius(double speed, double gForce) const;
    double calculateTurnRate(double speed, double gForce) const;

    // 根据北天东速度更新经纬高位置
    static void updatePositionFromVelocity(double dt, GeoPosition& position, const Velocity3& velocity);
};

// ============================================================================
// 水平转弯模型
// ============================================================================
class LevelTurn : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Level Turn"; }

private:
    double turnCenterNorth = 0.0;
    double turnCenterEast = 0.0;
    double initialHeading = 0.0;
    double turnRadius = 0.0;
    double turnRate = 0.0;
};

// ============================================================================
// 筋斗模型
// ============================================================================
class Loop : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Loop"; }

private:
    double loopRadius = 0.0;
    double loopCenterNorth = 0.0;
    double loopCenterUp = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;
};

// ============================================================================
// 横滚模型
// ============================================================================
class Roll : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Roll"; }

private:
    double rollRate = 0.0;
    double initialRoll = 0.0;
    double initialPitch = 0.0;
    double initialYaw = 0.0;
};

// ============================================================================
// 半滚倒转模型
// ============================================================================
class SplitS : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Split-S"; }

private:
    enum Phase { ROLL_PHASE, PUSH_DOWN_PHASE, COMPLETE };

    Phase currentPhase = ROLL_PHASE;
    double phaseTime = 0.0;
    double rollRate = 0.0;
    double pushDownRate = 0.0;
    double initialRoll = 0.0;
    double initialPitch = 0.0;
    double initialYaw = 0.0;
    double initialSpeed = 0.0;
};

// ============================================================================
// 工厂类 - 用于创建机动模型
// ============================================================================
class Factory {
public:
    static std::shared_ptr<Model> create(Type type);
    static Parameters getDefaultParams(Type type);
};

} // namespace KinematicManeuver

#endif // KINEMATIC_MANEUVER_SYSTEM_H
