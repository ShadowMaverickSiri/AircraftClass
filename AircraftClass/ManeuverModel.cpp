#include "ManeuverModel.h"
#include"EulerAngleCalculator.h"
#include <string>
#include <cmath>
#include <stdexcept>
#include <algorithm>

// ManeuverParameters实现
ManeuverParameters::ManeuverParameters()
	: turnRate(0.6), climbRate(0.2), rollRate(0.5), pitchRate(0.3),
	period(4.0), amplitude(1.0), altitudePeriod(3.0) {}

double ManeuverParameters::getActualTurnRate(const AircraftPerformance& perf) const {
	return turnRate * perf.maxTurnRate;
}
double ManeuverParameters::getActualClimbRate(const AircraftPerformance& perf) const {
	return climbRate * perf.maxClimbRate;
}
double ManeuverParameters::getActualRollRate(const AircraftPerformance& perf) const {
	return rollRate * perf.maxRollRate;
}
double ManeuverParameters::getActualPitchRate(const AircraftPerformance& perf) const {
	return pitchRate * perf.maxPitchRate;
}

// ManeuverState实现
ManeuverState::ManeuverState() : totalTime(0.0), currentPhase(0.0), isInitialized(false) {}
void ManeuverState::reset() {
	totalTime = 0.0;
	currentPhase = 0.0;
	isInitialized = false;
}

// 机动模型子类实现（以GeneralSManeuverModel为例，其它模型同理迁移...）
// ... 这里粘贴原有的GeneralSManeuverModel、LoopManeuverModel等所有实现 ...

// GeneralSManeuverModel
void GeneralSManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
}
void GeneralSManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualTurnRate = params.getActualTurnRate(perf);
	double actualClimbRate = params.getActualClimbRate(perf);
	double phase = 2.0 * M_PI * totalTime / params.period;
	double turnAngle = actualTurnRate * params.amplitude * std::sin(phase) * dt;
	if (params.climbRate != 0.0 && params.altitudePeriod > 0.0) {
		double altitudePhase = 2.0 * M_PI * totalTime / params.altitudePeriod;
		aircraft.velocity.up = actualClimbRate * std::sin(altitudePhase);
	}
	double speed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
		aircraft.velocity.east * aircraft.velocity.east);
	if (speed > 1e-3) {
		double cosTurn = std::cos(turnAngle);
		double sinTurn = std::sin(turnAngle);
		double newNorth = aircraft.velocity.north * cosTurn - aircraft.velocity.east * sinTurn;
		double newEast = aircraft.velocity.north * sinTurn + aircraft.velocity.east * cosTurn;
		aircraft.velocity.north = newNorth;
		aircraft.velocity.east = newEast;
	}
	aircraft.attitude = EulerAngleCalculator::calculateFromVelocity(aircraft.velocity);
}
void GeneralSManeuverModel::reset() { totalTime = 0.0; }

// LoopManeuverModel
void LoopManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
}
void LoopManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualClimbRate = params.getActualClimbRate(perf);
	aircraft.velocity.up += actualClimbRate * dt;
	aircraft.attitude = EulerAngleCalculator::calculateLoopManeuverAttitude(
		aircraft.velocity, actualClimbRate, dt);
}
void LoopManeuverModel::reset() { totalTime = 0.0; }

// RollManeuverModel
void RollManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
}
void RollManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualRollRate = params.getActualRollRate(perf);
	std::swap(aircraft.velocity.north, aircraft.velocity.east);
	aircraft.attitude = EulerAngleCalculator::calculateRollManeuverAttitude(
		aircraft.velocity, actualRollRate, dt);
}
void RollManeuverModel::reset() { totalTime = 0.0; }

// SplitSManeuverModel
void SplitSManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
	halfLoopDone = false;
}
void SplitSManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualTurnRate = params.getActualTurnRate(perf);
	double actualClimbRate = params.getActualClimbRate(perf);
	if (!halfLoopDone) {
		aircraft.velocity.up = actualClimbRate;
		aircraft.attitude.pitch = 0.3;
		if (aircraft.position.altitude > 1500.0 || totalTime > 3.0) {
			halfLoopDone = true;
		}
	}
	else {
		aircraft.velocity.up = -actualClimbRate * 0.5;
		aircraft.attitude.pitch = -0.2;
		double turnAngle = actualTurnRate * dt;
		double speed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
			aircraft.velocity.east * aircraft.velocity.east);
		if (speed > 1e-3) {
			double cosTurn = std::cos(turnAngle);
			double sinTurn = std::sin(turnAngle);
			double newNorth = aircraft.velocity.north * cosTurn - aircraft.velocity.east * sinTurn;
			double newEast = aircraft.velocity.north * sinTurn + aircraft.velocity.east * cosTurn;
			aircraft.velocity.north = newNorth;
			aircraft.velocity.east = newEast;
		}
	}
	aircraft.attitude = EulerAngleCalculator::calculateFromVelocity(aircraft.velocity);
}
void SplitSManeuverModel::reset() { totalTime = 0.0; halfLoopDone = false; }

// ImmelmannManeuverModel
void ImmelmannManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
	halfLoopDone = false;
}
void ImmelmannManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualPitchRate = params.getActualPitchRate(perf);
	double actualRollRate = params.getActualRollRate(perf);
	if (!halfLoopDone) {
		aircraft.attitude.pitch += actualPitchRate * dt;
		aircraft.velocity.up = params.getActualClimbRate(perf);
		if (aircraft.attitude.getPitchDegrees() >= 180.0) {
			aircraft.attitude.pitch = M_PI;
			halfLoopDone = true;
		}
	}
	else {
		aircraft.attitude.roll += actualRollRate * dt;
		aircraft.velocity.up = 0.0;
		if (aircraft.attitude.getRollDegrees() >= 180.0) {
			aircraft.attitude.roll = M_PI;
		}
	}
	aircraft.attitude = EulerAngleCalculator::limitAttitudeAngles(aircraft.attitude);
}
void ImmelmannManeuverModel::reset() { totalTime = 0.0; halfLoopDone = false; }

// BarrelRollManeuverModel
void BarrelRollManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
}
void BarrelRollManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualRollRate = params.getActualRollRate(perf);
	double actualPitchRate = params.getActualPitchRate(perf);
	aircraft.attitude.roll += actualRollRate * dt;
	aircraft.attitude.pitch += actualPitchRate * std::sin(totalTime) * dt;
	aircraft.attitude = EulerAngleCalculator::limitAttitudeAngles(aircraft.attitude);
}
void BarrelRollManeuverModel::reset() { totalTime = 0.0; }

// EvasiveDiveManeuverModel
void EvasiveDiveManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
	divePhase = true;
	turnPhase = false;
}
void EvasiveDiveManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualClimbRate = params.getActualClimbRate(perf);
	double actualTurnRate = params.getActualTurnRate(perf);
	if (divePhase) {
		aircraft.velocity.up = -actualClimbRate * 2.0;
		aircraft.attitude.pitch = -0.3;
		if (aircraft.position.altitude < 500.0 || totalTime > 3.0) {
			divePhase = false;
			turnPhase = true;
		}
	}
	else if (turnPhase) {
		double turnAngle = actualTurnRate * dt;
		double speed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
			aircraft.velocity.east * aircraft.velocity.east);
		if (speed > 1e-3) {
			double cosTurn = std::cos(turnAngle);
			double sinTurn = std::sin(turnAngle);
			double newNorth = aircraft.velocity.north * cosTurn - aircraft.velocity.east * sinTurn;
			double newEast = aircraft.velocity.north * sinTurn + aircraft.velocity.east * cosTurn;
			aircraft.velocity.north = newNorth;
			aircraft.velocity.east = newEast;
		}
		if (totalTime > 6.0) {
			aircraft.velocity.up = 0.0;
			aircraft.attitude.pitch = 0.0;
			turnPhase = false;
		}
	}
	aircraft.attitude = EulerAngleCalculator::calculateFromVelocity(aircraft.velocity);
}
void EvasiveDiveManeuverModel::reset() { totalTime = 0.0; divePhase = true; turnPhase = false; }

// LManeuverModel
void LManeuverModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
	turnPhase = false;
}
void LManeuverModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double actualTurnRate = params.getActualTurnRate(perf);
	if (!turnPhase && totalTime > 2.0) {
		turnPhase = true;
	}
	if (turnPhase) {
		double turnAngle = actualTurnRate * 2.0 * dt;
		double speed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
			aircraft.velocity.east * aircraft.velocity.east);
		if (speed > 1e-3) {
			double cosTurn = std::cos(turnAngle);
			double sinTurn = std::sin(turnAngle);
			double newNorth = aircraft.velocity.north * cosTurn - aircraft.velocity.east * sinTurn;
			double newEast = aircraft.velocity.north * sinTurn + aircraft.velocity.east * cosTurn;
			aircraft.velocity.north = newNorth;
			aircraft.velocity.east = newEast;
		}
	}
	aircraft.attitude = EulerAngleCalculator::calculateFromVelocity(aircraft.velocity);
}
void LManeuverModel::reset() { totalTime = 0.0; turnPhase = false; }

// ConstantFlightModel
void ConstantFlightModel::initialize(const ManeuverParameters& params) {
	this->params = params;
	totalTime = 0.0;
}
void ConstantFlightModel::update(Aircraft& aircraft, double dt) {
	totalTime += dt;
	const auto& perf = aircraft.getPerformance();
	double currentSpeed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
		aircraft.velocity.east * aircraft.velocity.east);
	double speedError = targetSpeed - currentSpeed;
	if (std::abs(speedError) > 1.0) {
		double speedAdjustment = speedControlGain * speedError * dt;
		double currentHeading = std::atan2(aircraft.velocity.east, aircraft.velocity.north);
		aircraft.velocity.north += speedAdjustment * std::cos(currentHeading);
		aircraft.velocity.east += speedAdjustment * std::sin(currentHeading);
	}
	double altitudeError = targetAltitude - aircraft.position.altitude;
	if (std::abs(altitudeError) > 10.0) {
		double climbRate = altitudeControlGain * altitudeError * dt;
		aircraft.velocity.up = climbRate;
	}
	else {
		aircraft.velocity.up = 0.0;
	}
	double currentHeading = std::atan2(aircraft.velocity.east, aircraft.velocity.north);
	double headingError = targetHeading - currentHeading;
	if (headingError > M_PI) headingError -= 2.0 * M_PI;
	if (headingError < -M_PI) headingError += 2.0 * M_PI;
	if (std::abs(headingError) > 0.1) {
		double turnRate = headingControlGain * headingError;
		double turnAngle = turnRate * dt;
		double speed = std::sqrt(aircraft.velocity.north * aircraft.velocity.north +
			aircraft.velocity.east * aircraft.velocity.east);
		if (speed > 1e-3) {
			double cosTurn = std::cos(turnAngle);
			double sinTurn = std::sin(turnAngle);
			double newNorth = aircraft.velocity.north * cosTurn - aircraft.velocity.east * sinTurn;
			double newEast = aircraft.velocity.north * sinTurn + aircraft.velocity.east * cosTurn;
			aircraft.velocity.north = newNorth;
			aircraft.velocity.east = newEast;
		}
	}
	aircraft.attitude = EulerAngleCalculator::calculateFromVelocity(aircraft.velocity);
}
void ConstantFlightModel::reset() { totalTime = 0.0; }

// 机动模型工厂实现
std::shared_ptr<ManeuverModel> ManeuverModelFactory::createManeuverModel(const std::string& name) {
	if (name == "s" || name == "S") {
		return std::make_shared<GeneralSManeuverModel>();
	}
	else if (name == "s_advanced" || name == "advanced_s") {
		return std::make_shared<GeneralSManeuverModel>();
	}
	else if (name == "snake") {
		return std::make_shared<GeneralSManeuverModel>();
	}
	else if (name == "loop") {
		return std::make_shared<LoopManeuverModel>();
	}
	else if (name == "roll") {
		return std::make_shared<RollManeuverModel>();
	}
	else if (name == "split_s") {
		return std::make_shared<SplitSManeuverModel>();
	}
	else if (name == "immelmann") {
		return std::make_shared<ImmelmannManeuverModel>();
	}
	else if (name == "barrel_roll") {
		return std::make_shared<BarrelRollManeuverModel>();
	}
	else if (name == "evasive_dive") {
		return std::make_shared<EvasiveDiveManeuverModel>();
	}
	else if (name == "l_maneuver") {
		return std::make_shared<LManeuverModel>();
	}
	else if (name == "constant" || name == "constant_flight") {
		return std::make_shared<ConstantFlightModel>();
	}
	else {
		throw std::invalid_argument("Unknown maneuver type: " + name);
	}
}

ManeuverParameters ManeuverModelFactory::getDefaultParameters(const std::string & maneuverType) {
	ManeuverParameters params;

	if (maneuverType == "s" || maneuverType == "S") {
		params.turnRate = 0.6;      // 60%的最大转向速率
		params.period = 4.0;
		params.amplitude = 1.0;
		params.climbRate = 0.0;     // 无爬升
		params.altitudePeriod = 0.0;
	}
	else if (maneuverType == "s_advanced" || maneuverType == "advanced_s") {
		params.turnRate = 0.5;      // 50%的最大转向速率
		params.period = 6.0;
		params.amplitude = 1.0;
		params.climbRate = 0.3;     // 30%的最大爬升速率
		params.altitudePeriod = 3.0;
	}
	else if (maneuverType == "snake") {
		params.turnRate = 0.8;      // 80%的最大转向速率
		params.period = 2.0;
		params.amplitude = 0.8;
		params.climbRate = 0.0;     // 无爬升
		params.altitudePeriod = 0.0;
	}
	else if (maneuverType == "loop") {
		params.turnRate = 0.0;      // 无转向
		params.climbRate = 0.8;     // 80%的最大爬升速率
		params.period = 10.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 5.0;
	}
	else if (maneuverType == "roll") {
		params.turnRate = 0.0;      // 无转向
		params.climbRate = 0.0;     // 无爬升
		params.rollRate = 0.8;      // 80%的最大滚转速率
		params.period = 2.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 0.0;
	}
	else if (maneuverType == "split_s") {
		params.turnRate = 0.7;      // 70%的最大转向速率
		params.climbRate = 0.6;     // 60%的最大爬升速率
		params.period = 8.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 4.0;
	}
	else if (maneuverType == "immelmann") {
		params.turnRate = 0.0;      // 无转向
		params.climbRate = 0.7;     // 70%的最大爬升速率
		params.rollRate = 0.8;      // 80%的最大滚转速率
		params.pitchRate = 0.6;     // 60%的最大俯仰速率
		params.period = 6.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 3.0;
	}
	else if (maneuverType == "barrel_roll") {
		params.turnRate = 0.0;      // 无转向
		params.climbRate = 0.0;     // 无爬升
		params.rollRate = 0.9;      // 90%的最大滚转速率
		params.pitchRate = 0.4;     // 40%的最大俯仰速率
		params.period = 4.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 0.0;
	}
	else if (maneuverType == "evasive_dive") {
		params.turnRate = 0.8;      // 80%的最大转向速率
		params.climbRate = -0.9;    // 90%的最大下降速率
		params.period = 6.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 3.0;
	}
	else if (maneuverType == "l_maneuver") {
		params.turnRate = 0.9;      // 90%的最大转向速率
		params.climbRate = 0.0;     // 无爬升
		params.period = 4.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 0.0;
	}
	else if (maneuverType == "constant" || maneuverType == "constant_flight") {
		params.turnRate = 0.0;      // 无转向
		params.climbRate = 0.0;     // 无爬升
		params.rollRate = 0.0;      // 无滚转
		params.pitchRate = 0.0;     // 无俯仰
		params.period = 0.0;        // 无周期
		params.amplitude = 0.0;     // 无振幅
		params.altitudePeriod = 0.0;
	}
	else {
		// 默认参数
		params.turnRate = 0.5;
		params.climbRate = 0.2;
		params.rollRate = 0.5;
		params.pitchRate = 0.3;
		params.period = 4.0;
		params.amplitude = 1.0;
		params.altitudePeriod = 3.0;
	}

	return params;
}