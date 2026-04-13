#ifndef KINEMATIC_MANEUVER_SYSTEM_H
#define KINEMATIC_MANEUVER_SYSTEM_H

#include "AircraftModelLibrary.h"
#include <string>
#include <memory>
#include <cmath>

// ============================================================
// 使用 SimTools 提供的四元数类
// ============================================================
// 定义 USE_EIGEN 使用 Eigen 版本
#define USE_EIGEN
#define SIMTOOLS_STATIC
#include "SimTools_v2.h"

namespace KinematicManeuver {

// ============================================================
// 类型别名
// ============================================================
using Quaternion = SimTools::Quaternion;
using Vector3d = SimTools::Vector3d;

// ============================================================
// 适配 AttitudeAngles 结构体的转换
// ============================================================
inline Quaternion FromEuler(const AttitudeAngles& attitude) {
    return Quaternion::FromEuler(attitude.roll, attitude.pitch, attitude.yaw);
}

inline AttitudeAngles ToAttitudeAngles(const Quaternion& q) {
    Vector3d euler = q.ToEuler();
    AttitudeAngles result;
    result.roll = euler[0];
    result.pitch = euler[1];
    result.yaw = euler[2];
    return result;
}

// ============================================================================
// 机动类型枚举
// ============================================================================
enum class Type {
    LEVEL_TURN,    // 水平转弯
    LOOP,          // 筋斗
    ROLL,          // 横滚
    SPLIT_S,       // 半滚倒转

    // 新增类型
    CONSTANT_TURN,       // 定速定高盘旋
    RACETRACK_PATTERN,   // 跑道型盘旋
    EIGHT_PATTERN,       // 8字型盘旋
    IMMELMAN_ESCAPE,     // 置尾降高逃逸
    L_PATTERN,           // L型机动
    S_PATTERN            // S型机动
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

    // 物理模型选项
    bool autoCalculateDuration = false;  // 自动计算完成时间

    // 初始状态
    GeoPosition initialPosition;
    Velocity3 initialVelocity;

    // ========== 新增参数 ==========
    // 多圈/重复机动
    int numCircles = 1;              // 圈数（用于定速定高盘旋）
    int numLaps = 1;                 // 跑道圈数

    // 跑道型盘旋
    double straightLength = 5000.0;  // 直道段长度（米）
    double turnRadius = 1000.0;      // 转弯半径（米）

    // 8字型盘旋
    double patternWidth = 2000.0;    // 8字宽度（米）
    double patternHeight = 1000.0;   // 8字高度（米）

    // 置尾降高逃逸
    double targetAltitude = 5000.0;  // 目标高度（米）
    double descentRate = 50.0;       // 下降速率（米/秒）
    double maxTurnRate = 10.0;       // 最大转弯速率（度/秒）

    // L型机动
    double leg1Distance = 5000.0;    // 第一段距离（米）
    double turnAngle = 90.0;         // 转弯角度（度）

    // S型机动
    double sTurnRadius = 1000.0;     // S型转弯半径（米）
    int numTurns = 2;                // S型转弯次数（2次形成完整S）

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

    // 计算筋斗理论完成时间
    static double calculateTheoreticalDuration(double speed, double gForce);

    // 获取当前实时信息
    double getCurrentSpeed() const { return initialSpeed; }
    double getLoopRadius() const { return loopRadius; }

private:
    // 几何参数
    double loopRadius = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;
    double initialLatitude = 0.0;
    double initialLongitude = 0.0;
    double initialAltitude = 0.0;
    double loopCenterLatitude = 0.0;
    double loopCenterLongitude = 0.0;
    double loopCenterAltitude = 0.0;
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

    // 阶段状态
    Phase currentPhase = ROLL_PHASE;
    double phaseTime = 0.0;

    // 时间参数
    double rollDuration = 0.0;    // 滚转阶段时长
    double pitchDuration = 0.0;   // 半筋斗阶段时长

    // 初始状态
    double initialYaw = 0.0;
    double initialSpeed = 0.0;
    double initialLatitude = 0.0;
    double initialLongitude = 0.0;
    double initialAltitude = 0.0;

    // 几何参数
    double loopRadius = 0.0;
    double rollEndLatitude = 0.0;
    double rollEndLongitude = 0.0;
    double rollEndAltitude = 0.0;

    // 四元数姿态
    Quaternion currentQuaternion;   // 当前姿态四元数
    Quaternion rollEndQuaternion;   // 滚转结束时的姿态四元数（倒飞，半筋斗起点）
    Quaternion halfLoopEndQuaternion; // 半筋斗结束时的姿态四元数（正飞，航向反转）
};

// ============================================================================
// 定速定高盘旋模型
// ============================================================================
class ConstantTurn : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Constant Turn"; }

private:
    double turnRadius = 0.0;
    double turnRate = 0.0;
    double initialHeading = 0.0;
    double turnCenterNorth = 0.0;
    double turnCenterEast = 0.0;
};

// ============================================================================
// 跑道型盘旋模型
// ============================================================================
class RacetrackPattern : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Racetrack Pattern"; }

private:
    enum Phase { LEG1_STRAIGHT, LEG2_TURN, LEG3_STRAIGHT, LEG4_TURN };
    Phase currentPhase = LEG1_STRAIGHT;

    double straightLength = 0.0;
    double turnRadius = 0.0;
    double turnRate = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;

    // 阶段时间
    double t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;

    // 转弯圆心
    double turn1CenterNorth = 0.0, turn1CenterEast = 0.0;
    double turn2CenterNorth = 0.0, turn2CenterEast = 0.0;

    // 各阶段起点位置
    double leg1EndLat = 0.0, leg1EndLon = 0.0, leg1EndAlt = 0.0;
    double leg2EndLat = 0.0, leg2EndLon = 0.0, leg2EndAlt = 0.0;
    double leg3EndLat = 0.0, leg3EndLon = 0.0, leg3EndAlt = 0.0;

    int currentLap = 0;
};

// ============================================================================
// 8字型盘旋模型
// ============================================================================
class EightPattern : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Eight Pattern"; }

private:
    enum Phase { FIRST_CIRCLE, SECOND_CIRCLE };
    Phase currentPhase = FIRST_CIRCLE;

    double turnRadius = 0.0;
    double turnRate = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;

    // 两个圆的圆心
    double circle1CenterNorth = 0.0, circle1CenterEast = 0.0;
    double circle2CenterNorth = 0.0, circle2CenterEast = 0.0;

    double circleDuration = 0.0;  // 一个圆的持续时间
};

// ============================================================================
// 置尾降高逃逸模型
// ============================================================================
class ImmelmanEscape : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Immelman Escape"; }

private:
    enum Phase { TURN_PHASE, DESCENT_PHASE, LEVEL_PHASE };
    Phase currentPhase = TURN_PHASE;

    double turnRate = 0.0;           // 弧度/秒
    double descentRate = 0.0;        // 米/秒
    double targetAltitude = 0.0;

    double turnDuration = 0.0;
    double descentDuration = 0.0;
    double levelDuration = 0.0;

    double initialHeading = 0.0;
    double initialAltitude = 0.0;
    double initialSpeed = 0.0;

    double turnEndLat = 0.0, turnEndLon = 0.0, turnEndAlt = 0.0;
};

// ============================================================================
// L型机动模型
// ============================================================================
class LPattern : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "L Pattern"; }

private:
    enum Phase { LEG1_STRAIGHT, TURN_PHASE, LEG2_STRAIGHT };
    Phase currentPhase = LEG1_STRAIGHT;

    double leg1Distance = 0.0;
    double turnAngle = 0.0;
    double turnRadius = 0.0;
    double turnRate = 0.0;
    double initialSpeed = 0.0;

    double leg1Duration = 0.0;
    double turnDuration = 0.0;
    double leg2Duration = 0.0;

    double initialHeading = 0.0;
    double turnEndLat = 0.0, turnEndLon = 0.0, turnEndAlt = 0.0;
};

// ============================================================================
// S型机动模型
// ============================================================================
class SPattern : public Model {
public:
    void initialize(const Parameters& params) override;
    void update(double currentTime, double dt,
               GeoPosition& position,
               Velocity3& velocity,
               AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "S Pattern"; }

private:
    enum Phase { FIRST_TURN, SECOND_TURN };
    Phase currentPhase = FIRST_TURN;

    double turnRadius = 0.0;
    double turnRate = 0.0;
    double initialHeading = 0.0;
    double initialSpeed = 0.0;

    double turnDuration = 0.0;
    int currentTurn = 0;  // 当前执行到第几个转弯

    // 第一个转弯的圆心
    double turn1CenterNorth = 0.0, turn1CenterEast = 0.0;
    // 第二个转弯的圆心
    double turn2CenterNorth = 0.0, turn2CenterEast = 0.0;

    double turn1EndLat = 0.0, turn1EndLon = 0.0, turn1EndAlt = 0.0;
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
