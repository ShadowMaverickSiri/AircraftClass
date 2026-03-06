#ifndef KINEMATIC_MANEUVER_SYSTEM_H
#define KINEMATIC_MANEUVER_SYSTEM_H

#include "AircraftModelLibrary.h"
#include <string>
#include <memory>
#include <cmath>

namespace KinematicManeuver {

// ============================================================================
// 四元数工具类 - 用于姿态解算
// ============================================================================
// 四元数格式: q = w + xi + yj + zk (w是标量部分)
// 使用四元数可以避免欧拉角的万向节锁问题，且旋转插值更平滑
// ============================================================================
class Quaternion {
public:
    double w, x, y, z;

    // 构造函数
    Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}  // 单位四元数
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // 从欧拉角构造四元数 (ZYX顺序: yaw->pitch->roll)
    static Quaternion fromEuler(double roll, double pitch, double yaw);
    static Quaternion fromEuler(const AttitudeAngles& attitude);

    // 从旋转轴和角度构造四元数
    static Quaternion fromAxisAngle(double axisX, double axisY, double axisZ, double angle);

    // 从旋转轴和角度构造四元数（向量形式）
    static Quaternion fromAxisAngle(const double axis[3], double angle);

    // 转换为欧拉角
    AttitudeAngles toEuler() const;
    void toEuler(double& roll, double& pitch, double& yaw) const;

    // 四元数归一化
    Quaternion normalized() const;

    // 四元数共轭
    Quaternion conjugate() const;

    // 四元数求逆
    Quaternion inverse() const;

    // 四元数乘法
    Quaternion operator*(const Quaternion& q) const;

    // 四元数与标量乘法
    Quaternion operator*(double scalar) const;

    // 四元数加法
    Quaternion operator+(const Quaternion& q) const;

    // 计算四元数的模（长度）
    double norm() const;

    // 计算四元数的模的平方
    double normSquared() const;

    // 点积
    double dot(const Quaternion& q) const;

    // 球面线性插值 (SLERP) - 用于平滑旋转过渡
    // t: [0, 1] 之间的插值参数
    static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double t);

    // 应用旋转向量
    void rotateVector(double vx, double vy, double vz,
                      double& rx, double& ry, double& rz) const;

    // 获取旋转矩阵 (3x3, 行主序)
    void toRotationMatrix(double matrix[3][3]) const;
};

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

    // 物理模型选项
    bool autoCalculateDuration = false;  // 自动计算完成时间

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
// 工厂类 - 用于创建机动模型
// ============================================================================
class Factory {
public:
    static std::shared_ptr<Model> create(Type type);
    static Parameters getDefaultParams(Type type);
};

} // namespace KinematicManeuver

#endif // KINEMATIC_MANEUVER_SYSTEM_H
