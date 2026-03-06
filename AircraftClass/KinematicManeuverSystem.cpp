#include "KinematicManeuverSystem.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>

using namespace KinematicManeuver;

// ============================================================================
// Quaternion 四元数类实现
// ============================================================================

// 从欧拉角构造四元数 (ZYX顺序: yaw->pitch->roll)
Quaternion Quaternion::fromEuler(double roll, double pitch, double yaw) {
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

Quaternion Quaternion::fromEuler(const AttitudeAngles& attitude) {
    return fromEuler(attitude.roll, attitude.pitch, attitude.yaw);
}

// 从旋转轴和角度构造四元数
Quaternion Quaternion::fromAxisAngle(double axisX, double axisY, double axisZ, double angle) {
    double halfAngle = angle * 0.5;
    double sinHalf = std::sin(halfAngle);
    double cosHalf = std::cos(halfAngle);

    // 归一化轴向量
    double len = std::sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (len < 1e-10) {
        return Quaternion();  // 返回单位四元数
    }

    Quaternion q;
    q.w = cosHalf;
    q.x = (axisX / len) * sinHalf;
    q.y = (axisY / len) * sinHalf;
    q.z = (axisZ / len) * sinHalf;
    return q;
}

Quaternion Quaternion::fromAxisAngle(const double axis[3], double angle) {
    return fromAxisAngle(axis[0], axis[1], axis[2], angle);
}

// 转换为欧拉角
void Quaternion::toEuler(double& roll, double& pitch, double& yaw) const {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(Constants::PI / 2, sinp);  // 使用90度如果超出范围
    } else {
        pitch = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

AttitudeAngles Quaternion::toEuler() const {
    AttitudeAngles attitude;
    toEuler(attitude.roll, attitude.pitch, attitude.yaw);
    return attitude;
}

// 四元数归一化
Quaternion Quaternion::normalized() const {
    double n = norm();
    if (n < 1e-10) {
        return Quaternion();  // 返回单位四元数
    }
    return Quaternion(w / n, x / n, y / n, z / n);
}

// 四元数共轭
Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

// 四元数求逆
Quaternion Quaternion::inverse() const {
    double n2 = normSquared();
    if (n2 < 1e-10) {
        return Quaternion();  // 返回单位四元数
    }
    Quaternion conj = conjugate();
    return Quaternion(conj.w / n2, conj.x / n2, conj.y / n2, conj.z / n2);
}

// 四元数乘法
Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w * q.w - x * q.x - y * q.y - z * q.z,
        w * q.x + x * q.w + y * q.z - z * q.y,
        w * q.y - x * q.z + y * q.w + z * q.x,
        w * q.z + x * q.y - y * q.x + z * q.w
    );
}

// 四元数与标量乘法
Quaternion Quaternion::operator*(double scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
}

// 四元数加法
Quaternion Quaternion::operator+(const Quaternion& q) const {
    return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
}

// 计算四元数的模
double Quaternion::norm() const {
    return std::sqrt(w * w + x * x + y * y + z * z);
}

// 计算四元数的模的平方
double Quaternion::normSquared() const {
    return w * w + x * x + y * y + z * z;
}

// 点积
double Quaternion::dot(const Quaternion& q) const {
    return w * q.w + x * q.x + y * q.y + z * q.z;
}

// 球面线性插值 (SLERP)
Quaternion Quaternion::slerp(const Quaternion& q0, const Quaternion& q1, double t) {
    // 计算两个四元数之间的夹角余弦值
    double cosOmega = q0.dot(q1);

    // 如果夹角余弦为负，取反一个四元数以选择最短路径
    Quaternion q1Temp = q1;
    if (cosOmega < 0.0) {
        q1Temp = Quaternion(-q1.w, -q1.x, -q1.y, -q1.z);
        cosOmega = -cosOmega;
    }

    // 如果四元数非常接近，使用线性插值
    if (cosOmega > 0.9999) {
        return Quaternion(
            q0.w + t * (q1Temp.w - q0.w),
            q0.x + t * (q1Temp.x - q0.x),
            q0.y + t * (q1Temp.y - q0.y),
            q0.z + t * (q1Temp.z - q0.z)
        ).normalized();
    }

    // 计算夹角
    double omega = std::acos(cosOmega);
    double sinOmega = std::sin(omega);

    // SLERP公式
    double scale0 = std::sin((1.0 - t) * omega) / sinOmega;
    double scale1 = std::sin(t * omega) / sinOmega;

    return Quaternion(
        scale0 * q0.w + scale1 * q1Temp.w,
        scale0 * q0.x + scale1 * q1Temp.x,
        scale0 * q0.y + scale1 * q1Temp.y,
        scale0 * q0.z + scale1 * q1Temp.z
    );
}

// 应用旋转向量
void Quaternion::rotateVector(double vx, double vy, double vz,
                              double& rx, double& ry, double& rz) const {
    // q * v * q^(-1)
    // 其中 v 作为纯四元数 (0, vx, vy, vz)
    // 简化公式（假设四元数已归一化）：
    // v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + q_w * v)

    double ux = x, uy = y, uz = z;

    double crossX = uy * vz - uz * vy;
    double crossY = uz * vx - ux * vz;
    double crossZ = ux * vy - uy * vx;

    double tempX = vx + 2.0 * (uy * crossZ - uz * crossY);
    double tempY = vy + 2.0 * (uz * crossX - ux * crossZ);
    double tempZ = vz + 2.0 * (ux * crossY - uy * crossX);

    rx = tempX;
    ry = tempY;
    rz = tempZ;
}

// 获取旋转矩阵 (3x3, 行主序)
void Quaternion::toRotationMatrix(double matrix[3][3]) const {
    double ww = w * w;
    double xx = x * x;
    double yy = y * y;
    double zz = z * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;

    matrix[0][0] = 1 - 2 * (yy + zz);
    matrix[0][1] = 2 * (xy - wz);
    matrix[0][2] = 2 * (xz + wy);

    matrix[1][0] = 2 * (xy + wz);
    matrix[1][1] = 1 - 2 * (xx + zz);
    matrix[1][2] = 2 * (yz - wx);

    matrix[2][0] = 2 * (xz - wy);
    matrix[2][1] = 2 * (yz + wx);
    matrix[2][2] = 1 - 2 * (xx + yy);
}

// ============================================================================
// Parameters 默认参数获取
// ============================================================================
Parameters Parameters::getDefault(Type type) {
    Parameters params;
    params.type = type;

    switch (type) {
        case Type::LEVEL_TURN:
            params.targetGForce = 3.0;
            params.duration = 15.0;
            params.turnDirection = 1.0;
            break;
        case Type::LOOP:
            params.targetGForce = 4.0;
            params.duration = 20.0;
            break;
        case Type::ROLL:
            params.targetGForce = 1.0;
            params.duration = 5.0;
            params.rollDirection = 1.0;
            break;
        case Type::SPLIT_S:
            params.targetGForce = 4.0;
            params.duration = 20.0;
            params.rollDirection = 1.0;
            break;
    }

    return params;
}

// ============================================================================
// Model 基类实现
// ============================================================================
bool Model::isActive(double currentTime) const {
    return currentTime >= params.startTime &&
           currentTime <= params.startTime + params.duration;
}

void Model::applyGForceConstraint(double& targetG) {
    targetG = std::max(1.0, std::min(9.0, targetG));
}

double Model::calculateTurnRadius(double speed, double gForce) const {
    if (gForce <= 1.0) return 1e6;  // 避免除零错误
    return (speed * speed) / (Constants::G * (gForce - 1.0));
}

double Model::calculateTurnRate(double speed, double gForce) const {
    if (speed <= 0.0 || gForce <= 1.0) return 0.0;
    return Constants::G * (gForce - 1.0) / speed;
}

void Model::updatePositionFromVelocity(double dt, GeoPosition& position, const Velocity3& velocity) {
    // 根据北天东速度更新经纬高位置
    // 纬度变化 = 北向速度 / 地球半径
    position.latitude += (velocity.north / Constants::EARTH_RADIUS) * Constants::RAD_TO_DEG * dt;

    // 经度变化 = 东向速度 / (地球半径 * cos(纬度))
    double radiusAtLat = Constants::EARTH_RADIUS * std::cos(position.latitude * Constants::DEG_TO_RAD);
    if (std::abs(radiusAtLat) > 1e-6)
        position.longitude += (velocity.east / radiusAtLat) * Constants::RAD_TO_DEG * dt;

    // 高度变化
    position.altitude += velocity.up * dt;
}

// ============================================================================
// LevelTurn 模型实现
// ============================================================================
void LevelTurn::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;

    // 计算初始航向角
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);

    // 计算初始速度大小
    double initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算转弯参数
    turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce) * params.turnDirection;

    // 计算转弯中心点
    double perpendicularAngle = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    turnCenterNorth = params.initialPosition.latitude + turnRadius * std::cos(perpendicularAngle) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    turnCenterEast = params.initialPosition.longitude + turnRadius * std::sin(perpendicularAngle) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
}

void LevelTurn::update(double currentTime, double dt,
                       GeoPosition& position,
                       Velocity3& velocity,
                       AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载（平滑过渡）
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    // 计算当前航向角
    double currentHeading = initialHeading + turnRate * maneuverTime;

    // 计算当前速度大小（保持恒定）
    double speed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 更新速度（北天东坐标系）
    velocity.north = speed * std::cos(currentHeading);
    velocity.east = speed * std::sin(currentHeading);
    velocity.up = 0.0;

    // 更新姿态
    attitude.yaw = currentHeading;
    attitude.pitch = 0.0;
    attitude.roll = std::atan2(speed * speed, Constants::G * turnRadius) * params.turnDirection;

    // 更新位置
    updatePositionFromVelocity(dt, position, velocity);
}

void LevelTurn::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
}

// ============================================================================
// Loop 模型实现
// ============================================================================
// 筋斗机动：在垂直平面内完成360°圆周运动
// 飞机从平飞开始（圆的左端），向上拉起，完成一圈回到平飞状态
// 现代战斗机在筋斗机动中，发动机推力补偿能量损失，速度基本恒定
// ============================================================================

// 计算筋斗理论完成时间
double Loop::calculateTheoreticalDuration(double speed, double gForce) {
    if (gForce <= 1.0 || speed <= 0.0) return 0.0;
    double radius = (speed * speed) / (Constants::G * (gForce - 1.0));
    double circumference = 2.0 * M_PI * radius;
    return circumference / speed;
}

void Loop::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;

    // 计算初始航向角和速度
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算筋斗半径（基于过载）
    loopRadius = (initialSpeed * initialSpeed) / (Constants::G * (params.targetGForce - 1.0));

    // 记录初始位置
    initialLatitude = params.initialPosition.latitude;
    initialLongitude = params.initialPosition.longitude;
    initialAltitude = params.initialPosition.altitude;

    // 计算圆心位置（在起始点前方 R 处，同一高度）
    loopCenterLatitude = initialLatitude + (loopRadius / Constants::EARTH_RADIUS) * std::cos(initialHeading) * Constants::RAD_TO_DEG;
    loopCenterLongitude = initialLongitude + (loopRadius / Constants::EARTH_RADIUS) * std::sin(initialHeading) * Constants::RAD_TO_DEG;
    loopCenterAltitude = initialAltitude;

    // 自动计算完成时间
    if (params.autoCalculateDuration) {
        const_cast<Parameters&>(params).duration = calculateTheoreticalDuration(initialSpeed, params.targetGForce);
    }
}

void Loop::update(double currentTime, double dt,
                  GeoPosition& position,
                  Velocity3& velocity,
                  AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    double maneuverTime = currentTime - params.startTime;
    this->currentTime = currentTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载（平滑过渡）
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    // 当前筋斗角度（从3π/2开始，转2π完成一圈）
    double loopAngle = 3.0 * M_PI / 2.0 + 2.0 * M_PI * progress;

    // 计算相对于圆心的位移
    double horizontalOffset = loopRadius * std::cos(loopAngle);
    double verticalOffset = loopRadius * std::sin(loopAngle);

    // 将水平位移转换为经纬度变化
    double horizontalDeg = horizontalOffset / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;

    // 更新位置（圆心 + 偏移）
    position.latitude = loopCenterLatitude + horizontalDeg * std::cos(initialHeading);
    position.longitude = loopCenterLongitude + horizontalDeg * std::sin(initialHeading);
    position.altitude = loopCenterAltitude + verticalOffset;

    // 速度沿圆周切线方向
    double horizontalSpeed = -initialSpeed * std::sin(loopAngle);
    double verticalSpeed = initialSpeed * std::cos(loopAngle);

    velocity.north = horizontalSpeed * std::cos(initialHeading);
    velocity.east = horizontalSpeed * std::sin(initialHeading);
    velocity.up = verticalSpeed;

    // 更新姿态角
    attitude.yaw = initialHeading;  // 航向保持不变

    // 俯仰角：飞机速度方向与水平面的夹角
    double pitchAngle = loopAngle - 3.0 * M_PI / 2.0;
    // 规范化到 -π 到 π
    while (pitchAngle > M_PI) pitchAngle -= 2 * M_PI;
    while (pitchAngle < -M_PI) pitchAngle += 2 * M_PI;
    attitude.pitch = pitchAngle;

    attitude.roll = 0.0;  // 筋斗时机翼始终水平
}

void Loop::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
}

// ============================================================================
// Roll 模型实现
// ============================================================================
void Roll::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;

    // 记录初始姿态
    initialRoll = 0.0;
    initialPitch = 0.0;
    initialYaw = std::atan2(params.initialVelocity.east, params.initialVelocity.north);

    // 计算滚转速率
    rollRate = 2.0 * M_PI * params.rollDirection / params.duration;
}

void Roll::update(double currentTime, double dt,
                  GeoPosition& position,
                  Velocity3& velocity,
                  AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;

    // 横滚时过载接近1G
    currentGForce = 1.0;

    // 计算滚转角度
    double rollAngle = rollRate * maneuverTime;

    // 计算当前航向角（保持不变）
    double currentHeading = initialYaw;

    // 计算飞行距离
    double speed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );
    double distance = speed * maneuverTime;

    // 更新位置（沿直线飞行）
    double distanceDeg = distance / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    position.latitude = params.initialPosition.latitude + distanceDeg * std::cos(currentHeading);
    position.longitude = params.initialPosition.longitude + distanceDeg * std::sin(currentHeading);
    position.altitude = params.initialPosition.altitude;

    // 速度保持不变
    velocity = params.initialVelocity;

    // 更新姿态
    attitude.yaw = currentHeading;
    attitude.pitch = initialPitch;
    attitude.roll = rollAngle;
}

void Roll::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
}

// ============================================================================
// SplitS 模型实现 - 使用四元数进行姿态解算
// ============================================================================
// 半滚倒转 = 滚转180° + 半筋斗向下
// 1. 滚转阶段：飞机平飞，滚转180°进入倒飞状态
// 2. 半筋斗阶段：在倒飞状态下推杆，完成向下的半圆，最终恢复正飞
void SplitS::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = ROLL_PHASE;

    // 记录初始状态
    initialYaw = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 记录初始位置
    initialLatitude = params.initialPosition.latitude;
    initialLongitude = params.initialPosition.longitude;
    initialAltitude = params.initialPosition.altitude;

    // 计算各阶段参数
    rollDuration = 2.0;  // 2秒完成滚转
    pitchDuration = params.duration - rollDuration;
    loopRadius = (initialSpeed * initialSpeed) / (Constants::G * (params.targetGForce - 1.0));

    // 初始化四元数为单位四元数（正飞状态）
    currentQuaternion = Quaternion();

    // 计算滚转结束时的姿态四元数（倒飞状态，roll = 180°，半筋斗起点）
    rollEndQuaternion = Quaternion::fromEuler(M_PI * params.rollDirection, 0.0, initialYaw);

    // 计算半筋斗结束时的姿态四元数（正飞状态，roll = 0°，航向反转180°）
    halfLoopEndQuaternion = Quaternion::fromEuler(0.0, 0.0, initialYaw + M_PI);
}

void SplitS::update(double currentTime, double dt,
                    GeoPosition& position,
                    Velocity3& velocity,
                    AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;

    if (maneuverTime < rollDuration) {
        // ========== 第一阶段：滚转180°进入倒飞 ==========
        currentPhase = ROLL_PHASE;
        currentGForce = 1.0;

        // 滚转进度
        double rollProgress = maneuverTime / rollDuration;

        // 使用SLERP插值计算当前姿态（从正飞到倒飞）
        Quaternion startQuaternion = Quaternion::fromEuler(0.0, 0.0, initialYaw);
        currentQuaternion = Quaternion::slerp(startQuaternion, rollEndQuaternion, rollProgress);

        // 沿直线飞行，高度不变
        double distance = initialSpeed * maneuverTime;
        double distanceDeg = distance / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
        position.latitude = initialLatitude + distanceDeg * std::cos(initialYaw);
        position.longitude = initialLongitude + distanceDeg * std::sin(initialYaw);
        position.altitude = initialAltitude;

        // 速度保持水平飞行
        velocity.north = initialSpeed * std::cos(initialYaw);
        velocity.east = initialSpeed * std::sin(initialYaw);
        velocity.up = 0.0;

        // 记录滚转结束时的位置，作为半筋斗阶段的起点
        rollEndLatitude = position.latitude;
        rollEndLongitude = position.longitude;
        rollEndAltitude = position.altitude;

    } else {
        // ========== 第二阶段：倒飞状态下的半筋斗向下 ==========
        currentPhase = PUSH_DOWN_PHASE;
        double pitchPhaseTime = maneuverTime - rollDuration;
        double pitchProgress = pitchPhaseTime / pitchDuration;

        // 过载：逐渐增加到目标过载
        double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(pitchProgress * 3.0, 1.0);
        applyGForceConstraint(targetG);
        currentGForce = targetG;

        // ========== SplitS 几何原理 ==========
        // 飞机在垂直平面（由初始航向定义）内完成半圆
        // 圆心在滚转结束点下方 R 处
        //
        // 角度定义（loopAngle）：从 π/2（起点）到 -π/2（终点），顺时针
        //
        // loopAngle  | 水平偏移      | 垂直偏移      | 速度方向        | 飞机状态
        // -----------|--------------|--------------|-----------------|----------
        // π/2 (起点) | 0 (前)       | +R (最高)    | 水平向前       | 倒飞平飞
        // 0          | +R (下)      | 0 (最低)    | 垂直向下       | 倒飞俯冲
        // -π/2 (终点)| 0 (后)       | -R (最低)   | 水平向后       | 正飞平飞
        //
        // 关键：航向反转180°是因为速度方向从向前变成向后
        //       但位置计算始终使用初始航向（在垂直平面内运动）

        double loopAngle = M_PI / 2.0 - M_PI * pitchProgress;

        // 计算相对于圆心的偏移
        double horizontalOffset = loopRadius * std::cos(loopAngle);
        double verticalOffset = loopRadius * std::sin(loopAngle);

        // 圆心位置（在滚转结束点下方 R 处）
        double loopCenterAltitude = rollEndAltitude - loopRadius;

        // ========== 位置计算 ==========
        // 关键：位置在垂直平面内变化，使用初始航向（不变）
        double horizontalDeg = horizontalOffset / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
        position.latitude = rollEndLatitude + horizontalDeg * std::cos(initialYaw);
        position.longitude = rollEndLongitude + horizontalDeg * std::sin(initialYaw);
        position.altitude = loopCenterAltitude + verticalOffset;

        // ========== 速度计算 ==========
        // 速度沿半圆切线方向
        double horizontalSpeed = initialSpeed * std::sin(loopAngle);
        double verticalSpeed = initialSpeed * std::cos(loopAngle);

        // 速度方向：水平分量使用初始航向
        // 当 horizontalSpeed 从 +V 变到 -V 时，自然实现航向反转
        velocity.north = horizontalSpeed * std::cos(initialYaw);
        velocity.east = horizontalSpeed * std::sin(initialYaw);
        velocity.up = verticalSpeed;

        // ========== 姿态计算（使用四元数SLERP插值）==========
        // 关键原理：使用SLERP在半筋斗的两个端点姿态之间进行平滑插值
        // 这样可以避免欧拉角的万向节锁问题和符号反转问题
        //
        // 半筋斗起点姿态（rollEndQuaternion）：倒飞(roll=180°), 平飞(pitch=0°), 初始航向
        // 半筋斗终点姿态（halfLoopEndQuaternion）：正飞(roll=0°), 平飞(pitch=0°), 反向航向(+180°)
        //
        // SLERP会自动处理中间所有姿态，包括最底点的俯冲姿态

        // 使用SLERP计算当前姿态（插值因子从0到1）
        currentQuaternion = Quaternion::slerp(rollEndQuaternion, halfLoopEndQuaternion, pitchProgress);
    }

    // 将四元数转换为欧拉角输出
    attitude = currentQuaternion.toEuler();
}

void SplitS::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = ROLL_PHASE;
    phaseTime = 0.0;
    currentQuaternion = Quaternion();  // 重置为单位四元数
}

// ============================================================================
// Factory 工厂类实现
// ============================================================================
std::shared_ptr<Model> Factory::create(Type type) {
    switch (type) {
        case Type::LEVEL_TURN:
            return std::make_shared<LevelTurn>();
        case Type::LOOP:
            return std::make_shared<Loop>();
        case Type::ROLL:
            return std::make_shared<Roll>();
        case Type::SPLIT_S:
            return std::make_shared<SplitS>();
        default:
            throw std::invalid_argument("Unknown maneuver type");
    }
}

Parameters Factory::getDefaultParams(Type type) {
    return Parameters::getDefault(type);
}
