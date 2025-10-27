/**
 * @file KinematicManeuverSystem.h
 * @brief 运动学机动系统头文件
 * @details 定义了飞机各种机动飞行的运动学模型，包括水平转弯、筋斗、横滚、半滚倒转等
 * @author 飞机仿真系统
 * @date 2024
 */

#ifndef KINEMATIC_MANEUVER_SYSTEM_H
#define KINEMATIC_MANEUVER_SYSTEM_H

#include <string>
#include <memory>
#include <cmath>
#include "AircraftModelLibrary.h"

/**
 * @brief 机动类型枚举
 * @details 定义了系统支持的各种飞机机动类型
 */
enum class ManeuverType {
    LEVEL_TURN,  ///< 水平转弯：飞机在水平面内进行转弯机动
    LOOP,        ///< 筋斗：飞机在垂直面内画完整圆形的机动
    ROLL,        ///< 横滚：飞机绕纵轴旋转360度的机动
    SPLIT_S      ///< 半滚倒转：先横滚180度再拉杆做半个筋斗的复合机动
};

/**
 * @brief 运动学机动参数结构体
 * @details 包含执行各种机动所需的所有参数，包括机动类型、目标过载、持续时间等
 */
struct KinematicManeuverParameters {
    ManeuverType type = ManeuverType::LEVEL_TURN; ///< 机动类型
    double targetGForce = 3.0;        ///< 目标过载值 (G)，表示飞机承受的重力加速度倍数
    double duration = 10.0;           ///< 机动持续时间 (秒)
    double startTime = 0.0;           ///< 机动起始时间 (秒)，相对于仿真开始时间
    double turnDirection = 1.0;       ///< 转弯方向 (1.0=右转, -1.0=左转)
    double rollDirection = 1.0;       ///< 滚转方向 (1.0=右滚, -1.0=左滚)
    
    // 输入参数
    GeoPosition initialPosition;      ///< 初始位置 (经纬高)，单位为度、度、米
    Velocity3 initialVelocity;       ///< 初始速度 (北天东坐标系)，单位为m/s
};

/**
 * @brief 运动学机动模型基类
 * @details 定义了所有机动模型的通用接口和基础功能
 * 
 * 该类提供了机动模型的核心接口，包括初始化、更新、重置等功能。
 * 所有具体的机动模型都继承自此类，实现特定的机动算法。
 */
class KinematicManeuverModel {
public:
    /**
     * @brief 虚析构函数
     */
    virtual ~KinematicManeuverModel() = default;
    
    // 核心接口
    /**
     * @brief 初始化机动模型
     * @param params 机动参数，包含机动类型、目标过载、持续时间等
     * @details 根据给定的参数初始化机动模型，计算初始状态和关键参数
     */
    virtual void initialize(const KinematicManeuverParameters& params) = 0;
    
    /**
     * @brief 更新机动状态
     * @param currentTime 当前仿真时间 (秒)
     * @param dt 仿真时间步长 (秒)
     * @param position 飞机位置 (经纬高，输入输出参数)
     * @param velocity 飞机速度 (北天东坐标系，输入输出参数)
     * @param attitude 飞机姿态角 (俯仰、滚转、偏航，输入输出参数)
     * @details 根据当前时间和时间步长更新飞机的运动状态
     */
    virtual void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) = 0;
    
    /**
     * @brief 重置机动模型
     * @details 将机动模型重置到初始状态，清除所有内部状态变量
     */
    virtual void reset() = 0;
    
    /**
     * @brief 获取机动名称
     * @return 机动类型的字符串描述
     */
    virtual std::string getName() const = 0;
    
    // 状态查询
    /**
     * @brief 获取当前过载值
     * @return 当前过载值 (G)
     */
    virtual double getCurrentGForce() const { return currentGForce; }
    
    /**
     * @brief 获取机动进度
     * @return 机动完成进度 (0.0-1.0)
     */
    virtual double getProgress() const { return (currentTime - params.startTime) / params.duration; }
    
    /**
     * @brief 检查机动是否处于活跃状态
     * @param currentTime 当前仿真时间 (秒)
     * @return true表示机动正在进行中，false表示机动未开始或已结束
     */
    virtual bool isActive(double currentTime) const;
    
protected:
    KinematicManeuverParameters params;  ///< 机动参数
    double currentTime = 0.0;            ///< 当前时间 (秒)
    double currentGForce = 1.0;          ///< 当前过载值 (G)
    
    // 辅助方法
    /**
     * @brief 应用过载约束
     * @param targetG 目标过载值 (输入输出参数)
     * @details 将过载值限制在合理范围内 (1.0G - 9.0G)
     */
    void applyGForceConstraint(double& targetG);
    
    /**
     * @brief 计算转弯半径
     * @param speed 飞行速度 (m/s)
     * @param gForce 过载值 (G)
     * @return 转弯半径 (m)
     * @details 基于向心力公式计算转弯半径：R = V²/(g*(n-1))
     */
    double calculateTurnRadius(double speed, double gForce) const;
    
    /**
     * @brief 计算转弯角速度
     * @param speed 飞行速度 (m/s)
     * @param gForce 过载值 (G)
     * @return 转弯角速度 (rad/s)
     * @details 基于向心力公式计算转弯角速度：ω = g*(n-1)/V
     */
    double calculateTurnRate(double speed, double gForce) const;
    
    /**
     * @brief 根据北天东速度计算经纬高变化
     * @param dt 时间步长 (秒)
     * @param position 位置信息 (经纬高，输入输出参数)
     * @param velocity 速度信息 (北天东坐标系，m/s)
     * @details 基于地球椭球模型计算位置变化，考虑地球曲率影响
     */
    void LLAcalculate(double dt, GeoPosition& position, const Velocity3& velocity );
};

/**
 * @brief 水平转弯模型
 * @details 实现飞机在水平面内的转弯机动
 * 
 * 该模型模拟飞机在水平面内进行恒定半径的转弯机动。
 * 飞机保持恒定高度，通过倾斜产生向心力来完成转弯。
 * 转弯过程中过载逐渐增加，航向角持续变化。
 */
class LevelTurnModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Level Turn"; }
    
private:
    double turnCenterNorth = 0.0;  ///< 转弯中心点北向坐标 (度)
    double turnCenterEast = 0.0;   ///< 转弯中心点东向坐标 (度)
    double initialHeading = 0.0;   ///< 初始航向角 (弧度)
    double turnRadius = 0.0;       ///< 转弯半径 (米)
    double turnRate = 0.0;         ///< 转弯角速度 (弧度/秒)
};

/**
 * @brief 筋斗模型
 * @details 实现飞机在垂直面内的完整圆形机动
 * 
 * 该模型模拟飞机在垂直面内画完整圆形的筋斗机动。
 * 飞机从水平飞行开始，向上拉起进入筋斗，经过顶点后向下完成圆形轨迹。
 * 筋斗过程中航向角保持不变，俯仰角从0度变化到360度。
 */
class LoopModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Loop"; }
    
private:
    double loopRadius = 0.0;        ///< 筋斗半径 (米)
    double loopCenterNorth = 0.0;   ///< 筋斗中心点北向坐标 (度)
    double loopCenterUp = 0.0;      ///< 筋斗中心点高度 (米)
    double initialHeading = 0.0;    ///< 初始航向角 (弧度)
    double initialSpeed = 0.0;      ///< 初始速度大小 (m/s)
};

/**
 * @brief 横滚模型
 * @details 实现飞机绕纵轴旋转360度的横滚机动
 * 
 * 该模型模拟飞机绕纵轴进行360度横滚的机动。
 * 横滚过程中飞机保持直线飞行，航向和俯仰角不变，只有滚转角持续变化。
 * 横滚机动通常用于展示飞行技巧或改变飞行姿态。
 */
class RollModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Roll"; }
    
private:
    double rollRate = 0.0;      ///< 横滚角速度 (弧度/秒)
    double initialRoll = 0.0;   ///< 初始滚转角 (弧度)
    double initialPitch = 0.0;  ///< 初始俯仰角 (弧度)
    double initialYaw = 0.0;    ///< 初始偏航角 (弧度)
};

/**
 * @brief 半滚倒转模型
 * @details 实现半滚倒转(Split-S)复合机动
 * 
 * 半滚倒转是一种典型的战斗机机动，分为两个阶段：
 * 1. 横滚阶段：飞机先横滚180度进入倒飞状态
 * 2. 筋斗阶段：在倒飞状态下拉杆做半个筋斗，使机头向下转过180度
 * 
 * 该机动能迅速将高度转化为速度，同时改变航向，是空战中的常用机动。
 */
class SplitSModel : public KinematicManeuverModel {
public:
    void initialize(const KinematicManeuverParameters& params) override;
    void update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) override;
    void reset() override;
    std::string getName() const override { return "Split-S"; }

	double Vx_n, Vy_n, Vz_n;  ///< 速度分量（用于调试）
    
private:
    /**
     * @brief 机动阶段枚举
     */
    enum Phase { 
        ROLL_PHASE,        ///< 横滚阶段：飞机横滚180度进入倒飞
        PUSH_DOWN_PHASE,   ///< 筋斗阶段：在倒飞状态下拉杆做半个筋斗
        COMPLETE           ///< 完成阶段：机动完成
    };
    
    Phase currentPhase = ROLL_PHASE;  ///< 当前机动阶段
    double phaseTime = 0.0;           ///< 当前阶段时间 (秒)
    double rollRate = 0.0;            ///< 横滚角速度 (弧度/秒)
    double pushDownRate = 0.0;        ///< 筋斗角速度 (弧度/秒)
    double initialRoll = 0.0;         ///< 初始滚转角 (弧度)
    double initialPitch = 0.0;        ///< 初始俯仰角 (弧度)
    double initialYaw = 0.0;          ///< 初始偏航角 (弧度)
    double initialSpeed = 0.0;        ///< 初始速度大小 (m/s)
	bool  isFirst = false;            ///< 航向角跳变标志位
};

/**
 * @brief 机动模型工厂类
 * @details 提供创建各种机动模型实例和获取默认参数的静态方法
 * 
 * 该工厂类采用工厂模式，根据机动类型创建相应的机动模型实例，
 * 并提供各种机动类型的默认参数配置。
 */
class KinematicManeuverFactory {
public:
    /**
     * @brief 创建机动模型实例
     * @param type 机动类型
     * @return 指向机动模型的智能指针
     * @throws std::invalid_argument 当机动类型未知时抛出异常
     */
    static std::shared_ptr<KinematicManeuverModel> create(ManeuverType type);
    
    /**
     * @brief 获取指定机动类型的默认参数
     * @param type 机动类型
     * @return 包含默认参数的KinematicManeuverParameters结构体
     * @details 为每种机动类型提供合理的默认参数配置
     */
    static KinematicManeuverParameters getDefaultParams(ManeuverType type);
};

#endif // KINEMATIC_MANEUVER_SYSTEM_H
