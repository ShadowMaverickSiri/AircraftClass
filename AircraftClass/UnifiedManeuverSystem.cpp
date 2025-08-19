#include "UnifiedManeuverSystem.h"
#include "FighterJet.h"
#include <algorithm>
#include <map>

// UnifiedManeuverParameters 实现
double UnifiedManeuverParameters::getActualTurnRate(const AircraftPerformance& perf) const {
    return turnRate * perf.maxTurnRate;
}

double UnifiedManeuverParameters::getActualClimbRate(const AircraftPerformance& perf) const {
    return climbRate * perf.maxClimbRate;
}

double UnifiedManeuverParameters::getActualRollRate(const AircraftPerformance& perf) const {
    return rollRate * perf.maxRollRate;
}

double UnifiedManeuverParameters::getActualPitchRate(const AircraftPerformance& perf) const {
    return pitchRate * perf.maxPitchRate;
}

double UnifiedManeuverParameters::getTurnRadius(double speed, double gForce) const {
    if (gForce <= 1.0) return std::numeric_limits<double>::infinity();
    return (speed * speed) / (9.81 * (gForce - 1.0));
}

double UnifiedManeuverParameters::getTurnRate(double speed, double gForce) const {
    if (gForce <= 1.0) return 0.0;
    return 9.81 * (gForce - 1.0) / speed;
}

double UnifiedManeuverParameters::getMaxAvailableG(double speed, const AircraftPerformance& perf) const {
    double stallMargin = speed / perf.stallSpeed;
    if (stallMargin < 1.2) return 1.0;
    return std::min(perf.maxGForce * std::min(stallMargin * stallMargin, 1.0), maxAllowedG);
}

bool UnifiedManeuverParameters::validate() const {
    if (mode == G_FORCE) {
        return (targetGForce <= maxAllowedG && targetGForce >= minAllowedG);
    }
    return (turnRate >= 0.0 && turnRate <= 1.0) &&
           (climbRate >= 0.0 && climbRate <= 1.0) &&
           (rollRate >= 0.0 && rollRate <= 1.0) &&
           (pitchRate >= 0.0 && pitchRate <= 1.0);
}

// UnifiedManeuverModel 实现
double UnifiedManeuverModel::getCurrentGForce(const Aircraft& aircraft) const {
    double airspeed = calculateAirspeed(aircraft);
    if (airspeed < 1.0) return 1.0;
    
    double liftCoeff = calculateLiftCoefficient(aircraft);
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    double lift = liftCoeff * dynamicPressure * aircraft.performance.wingArea;
    
    return lift / (aircraft.performance.mass * 9.81);
}

double UnifiedManeuverModel::calculateAirspeed(const Aircraft& aircraft) const {
    const Velocity3& v = aircraft.velocity;
    return std::sqrt(v.north * v.north + v.up * v.up + v.east * v.east);
}

double UnifiedManeuverModel::calculateLiftCoefficient(const Aircraft& aircraft) const {
    double airspeed = calculateAirspeed(aircraft);
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    double lift = aircraft.performance.mass * 9.81;
    return lift / (dynamicPressure * aircraft.performance.wingArea);
}

double UnifiedManeuverModel::calculateStallMargin(const Aircraft& aircraft) const {
    double airspeed = calculateAirspeed(aircraft);
    return airspeed / aircraft.performance.stallSpeed;
}

bool UnifiedManeuverModel::checkSafetyLimits(const Aircraft& aircraft) const {
    double stallMargin = calculateStallMargin(aircraft);
    if (stallMargin < 1.2) return false;
    
    if (params.mode == UnifiedManeuverParameters::G_FORCE) {
        double currentG = getCurrentGForce(aircraft);
        if (currentG > params.maxAllowedG || currentG < params.minAllowedG) return false;
    }
    
    return true;
}

Velocity3 UnifiedManeuverModel::calculateAngleRateControl(const Aircraft& aircraft, double dt) const {
    Velocity3 control = {0.0, 0.0, 0.0};
    
    double airspeed = calculateAirspeed(aircraft);
    if (airspeed < 1.0) return control;
    
    // 简化的角度控制
    double turnRate = params.getActualTurnRate(aircraft.performance);
    double climbRate = params.getActualClimbRate(aircraft.performance);
    
    control.north = turnRate * airspeed * 0.1;
    control.up = climbRate * 10.0;
    
    return control;
}

// 改进的过载控制方法
Velocity3 UnifiedManeuverModel::calculateGForceControl(const Aircraft& aircraft, double targetG, double dt) const {
    Velocity3 control = {0.0, 0.0, 0.0};
    
    double airspeed = calculateAirspeed(aircraft);
    if (airspeed < 1.0) return control;
    
    // 1. 计算实现目标过载所需的攻角
    double requiredAOA = calculateRequiredAOA(targetG, airspeed, aircraft);
    
    // 2. 计算当前攻角
    double currentAOA = calculateAngleOfAttack(aircraft);
    
    // 3. 计算攻角变化率
    double aoaRate = (requiredAOA - currentAOA) / AOA_TIME_CONSTANT;
    
    // 4. 通过俯仰角速度控制攻角
    // 攻角变化率 = 俯仰角速度 - 航迹角变化率
    // 在水平飞行时，航迹角变化率 ≈ 0，所以攻角变化率 ≈ 俯仰角速度
    double requiredPitchRate = aoaRate;
    
    // 5. 限制俯仰角速度在合理范围内
    double maxPitchRate = aircraft.performance.maxPitchRate;
    requiredPitchRate = std::max(-maxPitchRate, std::min(maxPitchRate, requiredPitchRate));
    
    // 6. 将俯仰角速度转换为控制输入
    // 这里我们通过修改俯仰角来实现控制
    control.up = requiredPitchRate * airspeed * 0.5;  // 考虑空速影响
    
    return control;
}

// 新增的动力学计算方法实现
double UnifiedManeuverModel::calculateAngleOfAttack(const Aircraft& aircraft) const {
    // 简化的攻角计算：攻角 ≈ 俯仰角 - 航迹角
    // 在水平飞行时，航迹角 ≈ 0，所以攻角 ≈ 俯仰角
    return aircraft.attitude.pitch;
}

double UnifiedManeuverModel::calculateRequiredAOA(double targetG, double airspeed, const Aircraft& aircraft) const {
    // 1. 计算所需升力
    double requiredLift = targetG * aircraft.performance.mass * 9.81;
    
    // 2. 计算动压
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    
    // 3. 计算所需升力系数
    double requiredLiftCoeff = requiredLift / (dynamicPressure * aircraft.performance.wingArea);
    
    // 4. 检查是否是战斗机，如果是则使用战斗机的气动参数
    const FighterJet* fighter = dynamic_cast<const FighterJet*>(&aircraft);
    if (fighter) {
        // 使用战斗机特有的气动参数
        double CL0 = fighter->getCL0();
        double CLalpha = fighter->getCLalpha();
        double requiredAOA = (requiredLiftCoeff - CL0) / CLalpha;
        
        // 限制攻角在战斗机允许的范围内
        double maxAOA = fighter->getMaxAOA();
        double minAOA = fighter->getMinAOA();
        
        return std::max(minAOA, std::min(maxAOA, requiredAOA));
    } else {
        // 对于非战斗机，使用通用的经验参数
        double CL0 = 0.12;           // 零攻角升力系数（经验值）
        double CLalpha = 5.5;        // 升力曲线斜率（经验值，弧度制）
        double requiredAOA = (requiredLiftCoeff - CL0) / CLalpha;
        
        // 限制攻角在合理范围内
        double maxAOA = 12.0 * M_PI / 180.0;  // 12度（保守值）
        double minAOA = -3.0 * M_PI / 180.0;  // -3度（保守值）
        
        return std::max(minAOA, std::min(maxAOA, requiredAOA));
    }
}

double UnifiedManeuverModel::calculateLiftFromAOA(double angleOfAttack, double airspeed, const Aircraft& aircraft) const {
    // 计算给定攻角下的升力
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    
    // 检查是否是战斗机，如果是则使用战斗机的气动参数
    const FighterJet* fighter = dynamic_cast<const FighterJet*>(&aircraft);
    if (fighter) {
        // 使用战斗机特有的气动参数
        double liftCoeff = fighter->calculateLiftCoefficient(angleOfAttack);
        return liftCoeff * dynamicPressure * aircraft.performance.wingArea;
    } else {
        // 对于非战斗机，使用通用的经验参数
        double CL0 = 0.12;           // 零攻角升力系数（经验值）
        double CLalpha = 5.5;        // 升力曲线斜率（经验值，弧度制）
        double liftCoeff = CL0 + CLalpha * angleOfAttack;
        
        return liftCoeff * dynamicPressure * aircraft.performance.wingArea;
    }
}

double UnifiedManeuverModel::calculateDragFromAOA(double angleOfAttack, double airspeed, const Aircraft& aircraft) const {
    // 计算给定攻角下的阻力
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    
    // 检查是否是战斗机，如果是则使用战斗机的气动参数
    const FighterJet* fighter = dynamic_cast<const FighterJet*>(&aircraft);
    if (fighter) {
        // 使用战斗机特有的气动参数
        double dragCoeff = fighter->calculateDragCoefficient(angleOfAttack);
        return dragCoeff * dynamicPressure * aircraft.performance.wingArea;
    } else {
        // 对于非战斗机，使用通用的经验参数
        double CD0 = 0.018;          // 零升阻力系数（经验值）
        double K = 0.08;             // 诱导阻力因子（经验值）
        double liftCoeff = 0.12 + 5.5 * angleOfAttack;  // 与升力计算保持一致
        double dragCoeff = CD0 + K * liftCoeff * liftCoeff;
        
        return dragCoeff * dynamicPressure * aircraft.performance.wingArea;
    }
}

void UnifiedManeuverModel::updateFlightDynamics(Aircraft& aircraft, double dt) {
    double airspeed = calculateAirspeed(aircraft);
    
    // 1. 更新攻角
    flightState.angleOfAttack = calculateAngleOfAttack(aircraft);
    
    // 2. 更新升力和阻力
    flightState.currentLift = calculateLiftFromAOA(flightState.angleOfAttack, airspeed, aircraft);
    flightState.currentDrag = calculateDragFromAOA(flightState.angleOfAttack, airspeed, aircraft);
    
    // 3. 更新升力系数和阻力系数
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    flightState.liftCoefficient = flightState.currentLift / (dynamicPressure * aircraft.performance.wingArea);
    flightState.dragCoefficient = flightState.currentDrag / (dynamicPressure * aircraft.performance.wingArea);
    
    // 4. 更新俯仰角速度（考虑重力矩和升力矩）
    double weight = aircraft.performance.mass * 9.81;
    double momentArm = 2.0;  // 简化的力矩臂长度 (米)
    
    // 重力矩
    double gravityMoment = weight * momentArm * sin(aircraft.attitude.pitch);
    
    // 升力矩
    double liftMoment = flightState.currentLift * momentArm * cos(flightState.angleOfAttack);
    
    // 总力矩
    double totalMoment = liftMoment - gravityMoment;
    
    // 俯仰角加速度 (简化模型)
    double momentOfInertia = aircraft.performance.mass * 5.0;  // 简化的转动惯量
    double pitchAcceleration = totalMoment / momentOfInertia;
    
    // 更新俯仰角速度
    flightState.pitchRate += pitchAcceleration * dt;
    
    // 限制俯仰角速度
    double maxPitchRate = aircraft.performance.maxPitchRate;
    flightState.pitchRate = std::max(-maxPitchRate, std::min(maxPitchRate, flightState.pitchRate));
}

void UnifiedManeuverModel::applyControlInputs(Aircraft& aircraft, double pitchRate, double dt) {
    // 1. 更新俯仰角
    aircraft.attitude.pitch += pitchRate * dt;
    
    // 2. 限制俯仰角在合理范围内
    double maxPitch = 90.0 * M_PI / 180.0;  // 90度
    double minPitch = -90.0 * M_PI / 180.0; // -90度
    aircraft.attitude.pitch = std::max(minPitch, std::min(maxPitch, aircraft.attitude.pitch));
    
    // 3. 根据姿态更新速度
    double airspeed = calculateAirspeed(aircraft);
    
    // 简化的速度更新：考虑升力和重力
    double lift = flightState.currentLift;
    double weight = aircraft.performance.mass * 9.81;
    double thrust = aircraft.performance.maxThrust * 0.3;  // 降低到30%推力，更合理
    double drag = flightState.currentDrag;
    
    // 垂直方向的力平衡
    double verticalForce = lift * cos(aircraft.attitude.pitch) - weight;
    double verticalAcceleration = verticalForce / aircraft.performance.mass;
    
    // 水平方向的力平衡
    double horizontalForce = thrust * cos(aircraft.attitude.pitch) - drag;
    double horizontalAcceleration = horizontalForce / aircraft.performance.mass;
    
    // 更新速度
    aircraft.velocity.up += verticalAcceleration * dt;
    aircraft.velocity.north += horizontalAcceleration * dt;
}

// SimpleTurnModel 实现
void SimpleTurnModel::initialize(const UnifiedManeuverParameters& params) {
    this->params = params;
    currentTime = 0.0;
    if (!params.validate()) {
        throw std::invalid_argument("Invalid turn parameters");
    }
}

void SimpleTurnModel::update(Aircraft& aircraft, double dt) {
    currentTime += dt;
    
    // 更新飞行动力学状态
    updateFlightDynamics(aircraft, dt);
    
    if (params.mode == UnifiedManeuverParameters::G_FORCE) {
        // 过载控制模式
        double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(currentTime / params.transitionTime, 1.0);
        if (!checkSafetyLimits(aircraft)) targetG = 1.0;
        
        Velocity3 control = calculateGForceControl(aircraft, targetG, dt);
        
        // 应用改进的控制输入
        double pitchRate = control.up / (calculateAirspeed(aircraft) * 0.5);
        applyControlInputs(aircraft, pitchRate, dt);
        
    } else {
        // 角度控制模式
        Velocity3 control = calculateAngleRateControl(aircraft, dt);
        
        // 应用控制输入（考虑阻力限制）
        double currentSpeed = calculateAirspeed(aircraft);
        double maxSpeed = aircraft.performance.maxSpeed;
        
        // 计算阻力减速（确保阻力不为零）
        double dragDeceleration = 0.0;
        if (flightState.currentDrag > 0.0) {
            dragDeceleration = flightState.currentDrag / aircraft.performance.mass;
        } else {
            // 如果阻力计算有问题，使用简化的阻力模型
            double dynamicPressure = 0.5 * 1.225 * currentSpeed * currentSpeed;
            double dragCoeff = 0.02; // 简化的阻力系数
            dragDeceleration = (dragCoeff * dynamicPressure * aircraft.performance.wingArea) / aircraft.performance.mass;
        }
        
        // 应用控制输入，但考虑速度限制
        double newNorthSpeed = aircraft.velocity.north + control.north * dt - dragDeceleration * dt;
        double newUpSpeed = aircraft.velocity.up + control.up * dt;
        double newEastSpeed = aircraft.velocity.east + control.east * dt;
        
        // 限制最大速度
        double newSpeed = sqrt(newNorthSpeed * newNorthSpeed + newUpSpeed * newUpSpeed + newEastSpeed * newEastSpeed);
        if (newSpeed > maxSpeed) {
            double scale = maxSpeed / newSpeed;
            newNorthSpeed *= scale;
            newUpSpeed *= scale;
            newEastSpeed *= scale;
        }
        
        aircraft.velocity.north = newNorthSpeed;
        aircraft.velocity.up = newUpSpeed;
        aircraft.velocity.east = newEastSpeed;
    }
}

void SimpleTurnModel::reset() {
    currentTime = 0.0;
}

// SimpleLoopModel 实现
void SimpleLoopModel::initialize(const UnifiedManeuverParameters& params) {
    this->params = params;
    currentTime = 0.0;
    loopPhase = 0.0;
}

void SimpleLoopModel::update(Aircraft& aircraft, double dt) {
    currentTime += dt;
    loopPhase += dt / params.duration;
    
    if (loopPhase >= 1.0) loopPhase = 0.0;
    
    // 更新飞行动力学状态
    updateFlightDynamics(aircraft, dt);
    
    if (params.mode == UnifiedManeuverParameters::G_FORCE) {
        // 筋斗中的过载变化
        double targetG;
        if (loopPhase < 0.25) {
            targetG = 1.0 + (params.targetGForce - 1.0) * (loopPhase / 0.25);
        } else if (loopPhase < 0.5) {
            double progress = (loopPhase - 0.25) / 0.25;
            targetG = params.targetGForce + (1.0 - params.targetGForce) * progress;
        } else if (loopPhase < 0.75) {
            double progress = (loopPhase - 0.5) / 0.25;
            targetG = 1.0 + (params.minAllowedG - 1.0) * progress;
        } else {
            double progress = (loopPhase - 0.75) / 0.25;
            targetG = params.minAllowedG + (1.0 - params.minAllowedG) * progress;
        }
        
        if (!checkSafetyLimits(aircraft)) targetG = 1.0;
        
        Velocity3 control = calculateGForceControl(aircraft, targetG, dt);
        
        // 应用改进的控制输入
        double pitchRate = control.up / (calculateAirspeed(aircraft) * 0.5);
        applyControlInputs(aircraft, pitchRate, dt);
        
    } else {
        Velocity3 control = calculateAngleRateControl(aircraft, dt);
        
        // 应用控制输入（考虑阻力限制）
        double currentSpeed = calculateAirspeed(aircraft);
        double maxSpeed = aircraft.performance.maxSpeed;
        
        // 计算阻力减速（确保阻力不为零）
        double dragDeceleration = 0.0;
        if (flightState.currentDrag > 0.0) {
            dragDeceleration = flightState.currentDrag / aircraft.performance.mass;
        } else {
            // 如果阻力计算有问题，使用简化的阻力模型
            double dynamicPressure = 0.5 * 1.225 * currentSpeed * currentSpeed;
            double dragCoeff = 0.02; // 简化的阻力系数
            dragDeceleration = (dragCoeff * dynamicPressure * aircraft.performance.wingArea) / aircraft.performance.mass;
        }
        
        // 应用控制输入，但考虑速度限制
        double newNorthSpeed = aircraft.velocity.north + control.north * dt - dragDeceleration * dt;
        double newUpSpeed = aircraft.velocity.up + control.up * dt;
        double newEastSpeed = aircraft.velocity.east + control.east * dt;
        
        // 限制最大速度
        double newSpeed = sqrt(newNorthSpeed * newNorthSpeed + newUpSpeed * newUpSpeed + newEastSpeed * newEastSpeed);
        if (newSpeed > maxSpeed) {
            double scale = maxSpeed / newSpeed;
            newNorthSpeed *= scale;
            newUpSpeed *= scale;
            newEastSpeed *= scale;
        }
        
        aircraft.velocity.north = newNorthSpeed;
        aircraft.velocity.up = newUpSpeed;
        aircraft.velocity.east = newEastSpeed;
    }
}

void SimpleLoopModel::reset() {
    currentTime = 0.0;
    loopPhase = 0.0;
}

// SimpleRollModel 实现
void SimpleRollModel::initialize(const UnifiedManeuverParameters& params) {
    this->params = params;
    currentTime = 0.0;
    rollAngle = 0.0;
}

void SimpleRollModel::update(Aircraft& aircraft, double dt) {
    currentTime += dt;
    
    // 更新飞行动力学状态
    updateFlightDynamics(aircraft, dt);
    
    Velocity3 control;
    if (params.mode == UnifiedManeuverParameters::G_FORCE) {
        double progress = std::min(currentTime / params.duration, 1.0);
        double targetG = 1.0 + (params.targetGForce - 1.0) * std::sin(progress * M_PI);
        
        if (!checkSafetyLimits(aircraft)) targetG = 1.0;
        control = calculateGForceControl(aircraft, targetG, dt);
    } else {
        control = calculateAngleRateControl(aircraft, dt);
    }
    
    // 应用控制输入（考虑阻力限制）
    double currentSpeed = calculateAirspeed(aircraft);
    double maxSpeed = aircraft.performance.maxSpeed;
    
    // 计算阻力减速（确保阻力不为零）
    double dragDeceleration = 0.0;
    if (flightState.currentDrag > 0.0) {
        dragDeceleration = flightState.currentDrag / aircraft.performance.mass;
    } else {
        // 如果阻力计算有问题，使用简化的阻力模型
        double dynamicPressure = 0.5 * 1.225 * currentSpeed * currentSpeed;
        double dragCoeff = 0.02; // 简化的阻力系数
        dragDeceleration = (dragCoeff * dynamicPressure * aircraft.performance.wingArea) / aircraft.performance.mass;
    }
    
    // 应用控制输入，但考虑速度限制
    double newNorthSpeed = aircraft.velocity.north + control.north * dt - dragDeceleration * dt;
    double newUpSpeed = aircraft.velocity.up + control.up * dt;
    double newEastSpeed = aircraft.velocity.east + control.east * dt;
    
    // 限制最大速度
    double newSpeed = sqrt(newNorthSpeed * newNorthSpeed + newUpSpeed * newUpSpeed + newEastSpeed * newEastSpeed);
    if (newSpeed > maxSpeed) {
        double scale = maxSpeed / newSpeed;
        newNorthSpeed *= scale;
        newUpSpeed *= scale;
        newEastSpeed *= scale;
    }
    
    aircraft.velocity.north = newNorthSpeed;
    aircraft.velocity.up = newUpSpeed;
    aircraft.velocity.east = newEastSpeed;
    
    // 更新滚转角
    rollAngle += rollDirection * aircraft.performance.maxRollRate * dt;
}

void SimpleRollModel::reset() {
	currentTime = 0.0;
	rollAngle = 0.0;
}

// SplitSManeuverModel 实现
void SplitSManeuverModel::initialize(const UnifiedManeuverParameters& params) {
	this->params = params;
	currentTime = 0.0;
	currentPhase = ROLL_PHASE;
	rollAngle = 0.0;
	loopPhase = 0.0;
	
	if (!params.validate()) {
		throw std::invalid_argument("Invalid Split-S parameters");
	}
}

void SplitSManeuverModel::update(Aircraft& aircraft, double dt) {
	currentTime += dt;
	
	// 更新飞行动力学状态
	updateFlightDynamics(aircraft, dt);
	
	Velocity3 control = {0.0, 0.0, 0.0};
	
	switch (currentPhase) {
		case ROLL_PHASE: {
			// 第一阶段：180度横滚
			double targetRollRate = aircraft.performance.maxRollRate * rollDirection;
			rollAngle += targetRollRate * dt;
			
			// 应用横滚控制
			if (params.mode == UnifiedManeuverParameters::G_FORCE) {
				// 过载控制：保持1G，专注于横滚
				double targetG = 1.0;
				if (!checkSafetyLimits(aircraft)) targetG = 1.0;
				control = calculateGForceControl(aircraft, targetG, dt);
			} else {
				// 角度控制：使用滚转率
				control = calculateAngleRateControl(aircraft, dt);
			}
			
			// 检查是否完成横滚阶段
			if (std::abs(rollAngle) >= M_PI) {  // 180度
				currentPhase = LOOP_PHASE;
				rollAngle = M_PI * rollDirection;  // 确保精确的180度
			}
			break;
		}
		
		case LOOP_PHASE: {
			// 第二阶段：半筋斗
			double phaseDuration = params.duration * 0.6;  // 60%时间用于半筋斗
			loopPhase += dt / phaseDuration;
			
			if (loopPhase >= 1.0) {
				loopPhase = 1.0;
				currentPhase = COMPLETE;
			}
			
			// 半筋斗中的过载变化
			if (params.mode == UnifiedManeuverParameters::G_FORCE) {
				double targetG;
				if (loopPhase < 0.5) {
					// 前半段：从1G到负过载
					targetG = 1.0 + (params.minAllowedG - 1.0) * (loopPhase / 0.5);
				} else {
					// 后半段：从负过载回到1G
					double progress = (loopPhase - 0.5) / 0.5;
					targetG = params.minAllowedG + (1.0 - params.minAllowedG) * progress;
				}
				
				if (!checkSafetyLimits(aircraft)) targetG = 1.0;
				control = calculateGForceControl(aircraft, targetG, dt);
			} else {
				// 角度控制：使用俯仰率进行半筋斗
				control = calculateAngleRateControl(aircraft, dt);
			}
			break;
		}
		
		case COMPLETE: {
			// 完成阶段：保持稳定飞行
			if (params.mode == UnifiedManeuverParameters::G_FORCE) {
				control = calculateGForceControl(aircraft, 1.0, dt);
			} else {
				control = calculateAngleRateControl(aircraft, dt);
			}
			break;
		}
	}
	
	// 应用控制输入（考虑阻力限制）
	double currentSpeed = calculateAirspeed(aircraft);
	double maxSpeed = aircraft.performance.maxSpeed;
	
	// 计算阻力减速（确保阻力不为零）
	double dragDeceleration = 0.0;
	if (flightState.currentDrag > 0.0) {
		dragDeceleration = flightState.currentDrag / aircraft.performance.mass;
	} else {
		// 如果阻力计算有问题，使用简化的阻力模型
		double dynamicPressure = 0.5 * 1.225 * currentSpeed * currentSpeed;
		double dragCoeff = 0.02; // 简化的阻力系数
		dragDeceleration = (dragCoeff * dynamicPressure * aircraft.performance.wingArea) / aircraft.performance.mass;
	}
	
	// 应用控制输入，但考虑速度限制
	double newNorthSpeed = aircraft.velocity.north + control.north * dt - dragDeceleration * dt;
	double newUpSpeed = aircraft.velocity.up + control.up * dt;
	double newEastSpeed = aircraft.velocity.east + control.east * dt;
	
	// 限制最大速度
	double newSpeed = sqrt(newNorthSpeed * newNorthSpeed + newUpSpeed * newUpSpeed + newEastSpeed * newEastSpeed);
	if (newSpeed > maxSpeed) {
		double scale = maxSpeed / newSpeed;
		newNorthSpeed *= scale;
		newUpSpeed *= scale;
		newEastSpeed *= scale;
	}
	
	aircraft.velocity.north = newNorthSpeed;
	aircraft.velocity.up = newUpSpeed;
	aircraft.velocity.east = newEastSpeed;
}

void SplitSManeuverModel::reset() {
	currentTime = 0.0;
	currentPhase = ROLL_PHASE;
	rollAngle = 0.0;
	loopPhase = 0.0;
}

// ManeuverFactory 实现
std::map<std::string, std::function<std::shared_ptr<UnifiedManeuverModel>()>> ManeuverFactory::creators = {
	{"turn", []() { return std::make_shared<SimpleTurnModel>(); }},
	{"loop", []() { return std::make_shared<SimpleLoopModel>(); }},
	{"roll", []() { return std::make_shared<SimpleRollModel>(); }},
	{"split-s", []() { return std::make_shared<SplitSManeuverModel>(); }},
	{"splits", []() { return std::make_shared<SplitSManeuverModel>(); }}
};

std::shared_ptr<UnifiedManeuverModel> ManeuverFactory::create(const std::string& name) {
    auto it = creators.find(name);
    if (it != creators.end()) {
        return it->second();
    }
    throw std::invalid_argument("Unknown maneuver type: " + name);
}

UnifiedManeuverParameters ManeuverFactory::getDefaultParams(const std::string& maneuverType) {
	UnifiedManeuverParameters params;
	
	if (maneuverType == "turn") {
		params.mode = UnifiedManeuverParameters::G_FORCE;
		params.targetGForce = 3.0;
		params.duration = 5.0;
	} else if (maneuverType == "loop") {
		params.mode = UnifiedManeuverParameters::G_FORCE;
		params.targetGForce = 4.0;
		params.duration = 8.0;
	} else if (maneuverType == "roll") {
		params.mode = UnifiedManeuverParameters::G_FORCE;
		params.targetGForce = 2.0;
		params.duration = 3.0;
	} else if (maneuverType == "split-s" || maneuverType == "splits") {
		params.mode = UnifiedManeuverParameters::G_FORCE;
		params.targetGForce = 3.0;
		params.minAllowedG = -2.0;  // 允许负过载
		params.duration = 6.0;      // 6秒完成整个机动
	}
	
	return params;
}
