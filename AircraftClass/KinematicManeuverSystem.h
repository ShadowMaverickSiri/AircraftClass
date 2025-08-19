#ifndef KINEMATIC_MANEUVER_SYSTEM_H
#define KINEMATIC_MANEUVER_SYSTEM_H

#include <string>
#include <memory>
#include <cmath>
#include "AircraftModelLibrary.h"

// 机动样式枚举
enum class ManeuverType {
    LEVEL_TURN,  // 水平转弯
    LOOP,        // 筋斗
    ROLL,        // 横滚
    SPLIT_S      // 半滚倒转
};

// 运动学机动参数
struct KinematicManeuverParameters {
    ManeuverType type = ManeuverType::LEVEL_TURN;
    double targetGForce = 3.0;        // 目标过载 (G)
    double duration = 10.0;           // 机动持续时间 (秒)
    double startTime = 0.0;           // 机动起始时间 (秒)
    double turnDirection = 1.0;       // 转弯方向 (1.0=右转, -1.0=左转)
    double rollDirection = 1.0;       // 滚转方向 (1.0=右滚, -1.0=左滚)
    
    // 输入参数
    GeoPosition initialPosition;      // 初始位置 (经纬高)
    Velocity3 initialVelocity;       // 初始速度 (北天东)
};

// 运动学机动模型基类
class KinematicManeuverModel {
public:
    virtual ~KinematicManeuverModel() = default;
    
    // 核心接口
    virtual void initialize(const KinematicManeuverParameters& params) = 0;
    virtual void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) = 0;
    virtual void reset() = 0;
    virtual std::string getName() const = 0;
    
    // 状态查询
    virtual double getCurrentGForce() const { return currentGForce; }
    virtual double getProgress() const { return (currentTime - params.startTime) / params.duration; }
    virtual bool isActive(double currentTime) const;
    
protected:
    KinematicManeuverParameters params;
    double currentTime = 0.0;
    double currentGForce = 1.0;
    
    // 辅助方法
    void applyGForceConstraint(double& targetG);
    double calculateTurnRadius(double speed, double gForce) const;
    double calculateTurnRate(double speed, double gForce) const;
    void LLAcalculate(double dt, GeoPosition& position, const Velocity3& velocity ); //根据北天东速度计算经纬高
};

// 水平转弯模型
class LevelTurnModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Level Turn"; }
    
private:
    double turnCenterNorth = 0.0;
    double turnCenterEast = 0.0;
    double initialHeading = 0.0;
    double turnRadius = 0.0;
    double turnRate = 0.0;
};

// 筋斗模型
class LoopModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Loop"; }
    
private:
    double loopRadius = 0.0;
    double loopCenterNorth = 0.0;
    double loopCenterUp = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;
};

// 横滚模型
class RollModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Roll"; }
    
private:
    double rollRate = 0.0;
    double initialRoll = 0.0;
    double initialPitch = 0.0;
    double initialYaw = 0.0;
};

// 半滚倒转模型
class SplitSModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
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

// 工厂类
class KinematicManeuverFactory {
public:
    static std::shared_ptr<KinematicManeuverModel> create(ManeuverType type);
    static KinematicManeuverParameters getDefaultParams(ManeuverType type);
};

#endif // KINEMATIC_MANEUVER_SYSTEM_H
