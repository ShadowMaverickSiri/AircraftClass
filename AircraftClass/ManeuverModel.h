#ifndef MANEUVER_MODEL_H
#define MANEUVER_MODEL_H

#include <string>
#include <memory>
#include <cmath>
#include "AircraftModelLibrary.h"

// 机动参数结构体
struct ManeuverParameters {
	double turnRate;
	double climbRate;
	double rollRate;
	double pitchRate;
	double period;
	double amplitude;
	double altitudePeriod;
	ManeuverParameters();
	double getActualTurnRate(const AircraftPerformance& perf) const;
	double getActualClimbRate(const AircraftPerformance& perf) const;
	double getActualRollRate(const AircraftPerformance& perf) const;
	double getActualPitchRate(const AircraftPerformance& perf) const;
};

// 机动状态结构体
struct ManeuverState {
	double totalTime;
	double currentPhase;
	bool isInitialized;
	ManeuverState();
	void reset();
};

// 机动模型基类
class ManeuverModel {
public:
	virtual ~ManeuverModel() = default;
	virtual void initialize(const ManeuverParameters& params) = 0;
	virtual void update(Aircraft& aircraft, double dt) = 0;
	virtual std::string getName() const = 0;
	virtual void reset() = 0;
};

// 机动模型工厂
class ManeuverModelFactory {
public:
	static std::shared_ptr<ManeuverModel> createManeuverModel(const std::string& name);
	static ManeuverParameters getDefaultParameters(const std::string& maneuverType);
};

// 具体机动模型子类声明
class GeneralSManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "General S Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
};
class LoopManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Loop Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
};
class RollManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Roll Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
};
class SplitSManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Split-S Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
	bool halfLoopDone = false;
};
class ImmelmannManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Immelmann Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
	bool halfLoopDone = false;
};
class BarrelRollManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Barrel Roll Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
};
class EvasiveDiveManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Evasive Dive Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
	bool divePhase = true;
	bool turnPhase = false;
};
class LManeuverModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "L Maneuver"; }
	void reset() override;
private:
	ManeuverParameters params;
	double totalTime = 0.0;
	bool turnPhase = false;
};
class ConstantFlightModel : public ManeuverModel {
public:
	void initialize(const ManeuverParameters& params) override;
	void update(Aircraft& aircraft, double dt) override;
	std::string getName() const override { return "Constant Speed & Altitude Flight"; }
	void reset() override;
	void setTargetSpeed(double speed) { targetSpeed = speed; }
	void setTargetAltitude(double altitude) { targetAltitude = altitude; }
	void setTargetHeading(double heading) { targetHeading = heading; }
private:
	ManeuverParameters params;
	double totalTime = 0.0;
	double targetSpeed = 200.0;
	double targetAltitude = 1000.0;
	double targetHeading = 0.0;
	double speedControlGain = 2.0;
	double altitudeControlGain = 1.0;
	double headingControlGain = 1.0;
};

#endif // MANEUVER_MODEL_H 