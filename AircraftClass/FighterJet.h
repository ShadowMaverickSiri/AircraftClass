#ifndef FIGHTER_JET_H
#define FIGHTER_JET_H

#include "AircraftModelLibrary.h"
#include <string>

// 战斗机类：继承自 Aircraft
class FighterJet : public Aircraft {
public:
	FighterJet(const std::string& modelName, const std::string& type);
	Velocity3 computeAcceleration() const override;

private:
	double maxThrust;   // 最大推力 (牛顿)
	double dragCoeff;   // 阻力系数
};

#endif // FIGHTER_JET_H
