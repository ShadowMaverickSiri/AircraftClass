
#include"EulerAngleCalculator.h"
#include <algorithm>

// 从速度向量计算基本姿态角
AttitudeAngles EulerAngleCalculator::calculateFromVelocity(const Velocity3& velocity) {
	AttitudeAngles attitude;

	double speed = sqrt(velocity.north * velocity.north +
		velocity.east * velocity.east +
		velocity.up * velocity.up);

	if (speed > 1e-3) {
		// 俯仰角：垂直速度分量与水平速度的比值
		double horizontalSpeed = sqrt(velocity.north * velocity.north +
			velocity.east * velocity.east);
		attitude.pitch = atan2(velocity.up, horizontalSpeed);

		// 偏航角：水平速度的方向
		attitude.yaw = atan2(velocity.east, velocity.north);

		// 滚转角：暂时设为0，需要根据机动类型计算
		attitude.roll = 0.0;
	}

	return attitude;
}

// S形机动的姿态角计算
AttitudeAngles EulerAngleCalculator::calculateSManeuverAttitude(const Velocity3 & velocity,
	double turnRate, double dt) {
	AttitudeAngles attitude = calculateFromVelocity(velocity);

	// S机动主要是偏航角变化
	double turnAngle = turnRate * dt;
	attitude.yaw += turnAngle;

	// 根据转向程度计算滚转角（模拟真实飞行）
	double rollAngle = turnAngle * 0.3; // 转向时会有一定滚转
	attitude.roll = rollAngle;

	return limitAttitudeAngles(attitude);
}

// 筋斗机动的姿态角计算
AttitudeAngles EulerAngleCalculator::calculateLoopManeuverAttitude(const Velocity3 & velocity,
	double climbRate, double dt) {
	AttitudeAngles attitude = calculateFromVelocity(velocity);

	// 筋斗机动主要是俯仰角变化
	double pitchRate = climbRate / 9.81; // 将爬升速率转换为俯仰角速率
	attitude.pitch += pitchRate * dt;

	// 筋斗时可能会有轻微的滚转
	attitude.roll = attitude.pitch * 0.1;

	return limitAttitudeAngles(attitude);
}

// 横滚机动的姿态角计算
AttitudeAngles EulerAngleCalculator::calculateRollManeuverAttitude(const Velocity3 & velocity,
	double rollRate, double dt) {
	AttitudeAngles attitude = calculateFromVelocity(velocity);

	// 横滚机动主要是滚转角变化
	attitude.roll += rollRate * dt;

	// 横滚时可能会有轻微的偏航变化
	attitude.yaw += rollRate * dt * 0.2;

	return limitAttitudeAngles(attitude);
}

// 蛇形机动的姿态角计算
AttitudeAngles EulerAngleCalculator::calculateSnakeManeuverAttitude(const Velocity3 & velocity,
	double turnRate, double amplitude, double dt) {
	AttitudeAngles attitude = calculateFromVelocity(velocity);

	// 蛇形机动是更剧烈的S形机动
	double turnAngle = turnRate * amplitude * dt;
	attitude.yaw += turnAngle;

	// 蛇形机动有更明显的滚转
	attitude.roll = turnAngle * 0.5;

	return limitAttitudeAngles(attitude);
}

// 高级S机动的姿态角计算（带高度变化）
AttitudeAngles EulerAngleCalculator::calculateAdvancedSAttitude(const Velocity3 & velocity,
	double turnRate, double climbRate,
	double period, double totalTime) {
	AttitudeAngles attitude = calculateFromVelocity(velocity);

	// 计算当前相位
	double phase = 2.0 * M_PI * totalTime / period;

	// 偏航角变化（S形）
	double yawChange = turnRate * sin(phase) * 0.1; // 0.1是时间步长
	attitude.yaw += yawChange;

	// 俯仰角变化（高度变化）
	double pitchChange = climbRate * sin(phase) * 0.1 / 9.81;
	attitude.pitch += pitchChange;

	// 滚转角（转向时的自然滚转）
	attitude.roll = yawChange * 0.3;

	return limitAttitudeAngles(attitude);
}

// 姿态角平滑插值
AttitudeAngles EulerAngleCalculator::interpolateAttitude(const AttitudeAngles & current,
	const AttitudeAngles & target,
	double alpha) {
	AttitudeAngles interpolated;

	// 线性插值
	interpolated.pitch = current.pitch + alpha * (target.pitch - current.pitch);
	interpolated.roll = current.roll + alpha * (target.roll - current.roll);
	interpolated.yaw = current.yaw + alpha * (target.yaw - current.yaw);

	return limitAttitudeAngles(interpolated);
}

// 姿态角限制（防止过度旋转）
AttitudeAngles EulerAngleCalculator::limitAttitudeAngles(const AttitudeAngles & attitude) {
	AttitudeAngles limited = attitude;

	// 限制俯仰角
	limited.pitch = std::max(-MAX_PITCH_ANGLE, std::min(MAX_PITCH_ANGLE, limited.pitch));

	// 限制滚转角
	limited.roll = std::max(-MAX_ROLL_ANGLE, std::min(MAX_ROLL_ANGLE, limited.roll));

	// 限制偏航角（保持连续性）
	while (limited.yaw > M_PI) limited.yaw -= 2.0 * M_PI;
	while (limited.yaw < -M_PI) limited.yaw += 2.0 * M_PI;

	return limited;
}

// 姿态角微分（计算角速度）
AttitudeAngles EulerAngleCalculator::calculateAngularVelocity(const AttitudeAngles & current,
	const AttitudeAngles & previous,
	double dt) {
	AttitudeAngles angularVelocity;

	if (dt > 1e-6) {
		angularVelocity.pitch = (current.pitch - previous.pitch) / dt;
		angularVelocity.roll = (current.roll - previous.roll) / dt;
		angularVelocity.yaw = (current.yaw - previous.yaw) / dt;
	}

	return angularVelocity;
}