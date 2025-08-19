#ifndef UNIFIED_MANEUVER_SYSTEM_H
#define UNIFIED_MANEUVER_SYSTEM_H

#include <string>
#include <memory>
#include <cmath>
#include <functional>
#include "AircraftModelLibrary.h"

// 统一的机动参数结构体
struct UnifiedManeuverParameters {
    // 控制模式
    enum ControlMode {
        ANGLE_RATE,     // 角度/速率控制
        G_FORCE,        // 过载控制
        HYBRID          // 混合控制
    };
    
    ControlMode mode = ANGLE_RATE;
    
    // 通用参数
    double duration = 10.0;        // 机动持续时间
    double transitionTime = 1.0;   // 过渡时间
    
    // 角度/速率控制参数
    double turnRate = 0.5;         // 转向速率比例 (0.0-1.0)
    double climbRate = 0.3;        // 爬升速率比例 (0.0-1.0)
    double rollRate = 0.5;         // 滚转速率比例 (0.0-1.0)
    double pitchRate = 0.5;        // 俯仰速率比例 (0.0-1.0)
    double period = 5.0;           // 周期
    double amplitude = 0.5;        // 幅度
    
    // 过载控制参数
    double targetGForce = 3.0;     // 目标过载 (G)
    double maxAllowedG = 9.0;      // 最大允许过载
    double minAllowedG = -3.0;     // 最小允许过载
    
    // 构造函数
    UnifiedManeuverParameters() = default;
    
    // 根据模式获取实际参数
    double getActualTurnRate(const AircraftPerformance& perf) const;
    double getActualClimbRate(const AircraftPerformance& perf) const;
    double getActualRollRate(const AircraftPerformance& perf) const;
    double getActualPitchRate(const AircraftPerformance& perf) const;
    
    // 过载相关计算
    double getTurnRadius(double speed, double gForce) const;
    double getTurnRate(double speed, double gForce) const;
    double getMaxAvailableG(double speed, const AircraftPerformance& perf) const;
    
    // 参数验证
    bool validate() const;
};

// 飞行动力学状态结构
struct FlightDynamicsState {
    double angleOfAttack;      // 攻角 (弧度)
    double pitchAngle;         // 俯仰角 (弧度)
    double pitchRate;          // 俯仰角速度 (弧度/秒)
    double liftCoefficient;    // 升力系数
    double dragCoefficient;    // 阻力系数
    double currentLift;        // 当前升力 (N)
    double currentDrag;        // 当前阻力 (N)
    
    FlightDynamicsState() : angleOfAttack(0.0), pitchAngle(0.0), pitchRate(0.0),
                           liftCoefficient(0.0), dragCoefficient(0.0),
                           currentLift(0.0), currentDrag(0.0) {}
};

// 统一的机动模型基类
class UnifiedManeuverModel {
public:
    virtual ~UnifiedManeuverModel() = default;
    
    // 核心接口
    virtual void initialize(const UnifiedManeuverParameters& params) = 0;
    virtual void update(Aircraft& aircraft, double dt) = 0;
    virtual void reset() = 0;
    virtual std::string getName() const = 0;
    
    // 状态查询
    virtual double getCurrentGForce(const Aircraft& aircraft) const;
    virtual double getProgress() const { return currentTime / params.duration; }
    
protected:
    UnifiedManeuverParameters params;
    double currentTime = 0.0;
    FlightDynamicsState flightState;
    
    // 动力学参数
    static constexpr double LIFT_TIME_CONSTANT = 0.1;    // 升力响应时间常数 (秒)
    static constexpr double PITCH_TIME_CONSTANT = 0.2;   // 俯仰响应时间常数 (秒)
    static constexpr double AOA_TIME_CONSTANT = 0.15;    // 攻角响应时间常数 (秒)
    
    // 辅助方法
    double calculateAirspeed(const Aircraft& aircraft) const;
    double calculateLiftCoefficient(const Aircraft& aircraft) const;
    double calculateStallMargin(const Aircraft& aircraft) const;
    bool checkSafetyLimits(const Aircraft& aircraft) const;
    
    // 改进的控制输入计算
    Velocity3 calculateAngleRateControl(const Aircraft& aircraft, double dt) const;
    Velocity3 calculateGForceControl(const Aircraft& aircraft, double targetG, double dt) const;
    
    // 新增的动力学计算方法
    double calculateAngleOfAttack(const Aircraft& aircraft) const;
    double calculateRequiredAOA(double targetG, double airspeed, const Aircraft& aircraft) const;
    double calculateLiftFromAOA(double angleOfAttack, double airspeed, const Aircraft& aircraft) const;
    double calculateDragFromAOA(double angleOfAttack, double airspeed, const Aircraft& aircraft) const;
    void updateFlightDynamics(Aircraft& aircraft, double dt);
    void applyControlInputs(Aircraft& aircraft, double pitchRate, double dt);
};

// 简化的机动模型实现
class SimpleTurnModel : public UnifiedManeuverModel {
public:
    void initialize(const UnifiedManeuverParameters& params) override;
    void update(Aircraft& aircraft, double dt) override;
    void reset() override;
    std::string getName() const override { return "Simple Turn"; }
    
private:
    double turnDirection = 1.0;
};

class SimpleLoopModel : public UnifiedManeuverModel {
public:
    void initialize(const UnifiedManeuverParameters& params) override;
    void update(Aircraft& aircraft, double dt) override;
    void reset() override;
    std::string getName() const override { return "Simple Loop"; }
    
private:
    double loopPhase = 0.0;
};

class SimpleRollModel : public UnifiedManeuverModel {
public:
	void initialize(const UnifiedManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	void reset() override;
	std::string getName() const override { return "Simple Roll"; }
	
private:
	double rollDirection = 1.0;
	double rollAngle = 0.0;
};

// 半滚倒转机动模型
class SplitSManeuverModel : public UnifiedManeuverModel {
public:
	void initialize(const UnifiedManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	void reset() override;
	std::string getName() const override { return "Split-S Maneuver"; }
	
private:
	enum Phase {
		ROLL_PHASE,      // 横滚阶段
		LOOP_PHASE,      // 半筋斗阶段
		COMPLETE         // 完成
	};
	
	Phase currentPhase = ROLL_PHASE;
	double rollAngle = 0.0;//当前滚转角
	double loopPhase = 0.0;//当前半筋斗阶段
	double rollDirection = 1.0;  // 滚转方向 1.0表示向右转转弯, -1.0 向左转弯
};

// 工厂类
class ManeuverFactory {
public:
    static std::shared_ptr<UnifiedManeuverModel> create(const std::string& name);
    static UnifiedManeuverParameters getDefaultParams(const std::string& maneuverType);
    
private:
    static std::map<std::string, std::function<std::shared_ptr<UnifiedManeuverModel>()>> creators;
};

#endif // UNIFIED_MANEUVER_SYSTEM_H
