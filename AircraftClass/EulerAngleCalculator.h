#ifndef EULER_ANGLE_CALCULATOR_H
#define EULER_ANGLE_CALCULATOR_H

#include "AircraftModelLibrary.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// 欧拉角计算器 - 内联实现
// ============================================================================
class EulerAngleCalculator {
public:
    // 从速度向量计算基本姿态角
    static AttitudeAngles calculateFromVelocity(const Velocity3& velocity) {
        AttitudeAngles attitude;
        double speed = std::sqrt(velocity.north * velocity.north +
                                velocity.east * velocity.east +
                                velocity.up * velocity.up);

        if (speed > 1e-3) {
            double horizontalSpeed = std::sqrt(velocity.north * velocity.north +
                                               velocity.east * velocity.east);
            attitude.pitch = std::atan2(velocity.up, horizontalSpeed);
            attitude.yaw = std::atan2(velocity.east, velocity.north);
            attitude.roll = 0.0;
        }
        return attitude;
    }

    // 姿态角限制
    static AttitudeAngles limit(const AttitudeAngles& attitude) {
        AttitudeAngles limited = attitude;
        limited.pitch = std::max(-MAX_PITCH, std::min(MAX_PITCH, limited.pitch));
        limited.roll = std::max(-MAX_ROLL, std::min(MAX_ROLL, limited.roll));
        // 偏航角保持连续性
        while (limited.yaw > M_PI) limited.yaw -= 2.0 * M_PI;
        while (limited.yaw < -M_PI) limited.yaw += 2.0 * M_PI;
        return limited;
    }

    // 姿态角插值
    static AttitudeAngles interpolate(const AttitudeAngles& current,
                                     const AttitudeAngles& target,
                                     double alpha) {
        AttitudeAngles result;
        result.pitch = current.pitch + alpha * (target.pitch - current.pitch);
        result.roll = current.roll + alpha * (target.roll - current.roll);
        result.yaw = current.yaw + alpha * (target.yaw - current.yaw);
        return limit(result);
    }

    // 计算角速度
    static AttitudeAngles calculateAngularVelocity(const AttitudeAngles& current,
                                                   const AttitudeAngles& previous,
                                                   double dt) {
        AttitudeAngles angularVelocity;
        if (dt > 1e-6) {
            angularVelocity.pitch = (current.pitch - previous.pitch) / dt;
            angularVelocity.roll = (current.roll - previous.roll) / dt;
            angularVelocity.yaw = (current.yaw - previous.yaw) / dt;
        }
        return angularVelocity;
    }

private:
    static constexpr double MAX_PITCH = M_PI / 3.0;   // 60度
    static constexpr double MAX_ROLL = M_PI / 2.0;    // 90度
};

#endif // EULER_ANGLE_CALCULATOR_H
