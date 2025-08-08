#ifndef EULER_ANGLE_CALCULATION_H
#define EULER_ANGLE_CALCULATION_H

#include"AircraftModelLibrary.h"
#include <cmath>

// 欧拉角计算器：专门负责计算飞机的欧拉角（俯仰、滚转、偏航）
class EulerAngleCalculator {
public:
	// 从速度向量计算基本姿态角
	static AttitudeAngles calculateFromVelocity(const Velocity3& velocity);

	// S形机动的姿态角计算
	static AttitudeAngles calculateSManeuverAttitude(const Velocity3& velocity,
		double turnRate, double dt);

	// 筋斗机动的姿态角计算
	static AttitudeAngles calculateLoopManeuverAttitude(const Velocity3& velocity,
		double climbRate, double dt);

	// 横滚机动的姿态角计算
	static AttitudeAngles calculateRollManeuverAttitude(const Velocity3& velocity,
		double rollRate, double dt);

	// 蛇形机动的姿态角计算
	static AttitudeAngles calculateSnakeManeuverAttitude(const Velocity3& velocity,
		double turnRate, double amplitude, double dt);

	// 高级S机动的姿态角计算（带高度变化）
	static AttitudeAngles calculateAdvancedSAttitude(const Velocity3& velocity,
		double turnRate, double climbRate,
		double period, double totalTime);

	// 姿态角平滑插值
	static AttitudeAngles interpolateAttitude(const AttitudeAngles& current,
		const AttitudeAngles& target,
		double alpha);

	// 姿态角限制（防止过度旋转）
	static AttitudeAngles limitAttitudeAngles(const AttitudeAngles& attitude);

	// 姿态角微分（计算角速度）
	static AttitudeAngles calculateAngularVelocity(const AttitudeAngles& current,
		const AttitudeAngles& previous,
		double dt);

private:
	static constexpr double MAX_PITCH_ANGLE = M_PI / 3.0;    // 60度
	static constexpr double MAX_ROLL_ANGLE = M_PI / 2.0;     // 90度
	static constexpr double MAX_YAW_ANGLE = M_PI;            // 180度
};

#endif // EULER_ANGLE_CALCULATION_H 