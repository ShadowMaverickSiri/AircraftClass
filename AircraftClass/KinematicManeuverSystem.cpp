#include "KinematicManeuverSystem.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>

using namespace KinematicManeuver;

// ============================================================
// Quaternion 类已由 SimTools 提供
// 所有四元数运算（构造、转换、插值等）现在使用 SimTools::Vector4d
// ============================================================

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
        case Type::CONSTANT_TURN:
            params.targetGForce = 3.0;
            params.duration = 30.0;
            params.turnDirection = 1.0;
            params.numCircles = 2;
            break;
        case Type::RACETRACK_PATTERN:
            params.targetGForce = 3.0;
            params.duration = 60.0;
            params.turnDirection = 1.0;
            params.straightLength = 5000.0;
            params.turnRadius = 1000.0;
            params.numLaps = 1;
            break;
        case Type::EIGHT_PATTERN:
            params.targetGForce = 3.0;
            params.duration = 60.0;
            params.turnDirection = 1.0;
            params.patternWidth = 2000.0;
            params.patternHeight = 1000.0;
            break;
        case Type::IMMELMAN_ESCAPE:
            params.targetGForce = 4.0;
            params.duration = 30.0;
            params.turnDirection = 1.0;
            params.targetAltitude = 5000.0;
            params.descentRate = 50.0;
            params.maxTurnRate = 10.0;
            break;
        case Type::L_PATTERN:
            params.targetGForce = 3.0;
            params.duration = 30.0;
            params.turnDirection = 1.0;
            params.leg1Distance = 5000.0;
            params.turnAngle = 90.0;
            break;
        case Type::S_PATTERN:
            params.targetGForce = 3.0;
            params.duration = 40.0;
            params.turnDirection = 1.0;
            params.sTurnRadius = 1000.0;
            params.numTurns = 2;
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
    // 注意：必须修改 this->params.duration，而不是原始 params
    if (params.autoCalculateDuration) {
        this->params.duration = calculateTheoreticalDuration(initialSpeed, params.targetGForce);
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
    currentQuaternion = Quaternion::Identity();

    // 计算滚转结束时的姿态四元数（倒飞状态，roll = 180°，半筋斗起点）
    rollEndQuaternion = Quaternion::FromEuler(M_PI * params.rollDirection, 0.0, initialYaw);

    // 计算半筋斗结束时的姿态四元数（正飞状态，roll = 0°，航向反转180°）
    halfLoopEndQuaternion = Quaternion::FromEuler(0.0, 0.0, initialYaw + M_PI);
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
        Quaternion startQuaternion = Quaternion::FromEuler(0.0, 0.0, initialYaw);
        currentQuaternion = Quaternion::Slerp(startQuaternion, rollEndQuaternion, rollProgress);

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
        currentQuaternion = Quaternion::Slerp(rollEndQuaternion, halfLoopEndQuaternion, pitchProgress);
    }

    // 将四元数转换为欧拉角输出
    attitude = ToAttitudeAngles(currentQuaternion);
}

void SplitS::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = ROLL_PHASE;
    phaseTime = 0.0;
    currentQuaternion = Quaternion::Identity();  // 重置为单位四元数
}

// ============================================================================
// ConstantTurn 定速定高盘旋模型实现
// ============================================================================
// 定速定高盘旋：水平圆周运动，支持指定圈数
// 类似LevelTurn，但可以指定完成圈数
void ConstantTurn::initialize(const Parameters& params) {
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
    turnCenterNorth = params.initialPosition.latitude +
        turnRadius * std::cos(perpendicularAngle) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    turnCenterEast = params.initialPosition.longitude +
        turnRadius * std::sin(perpendicularAngle) / (Constants::EARTH_RADIUS *
        std::cos(params.initialPosition.latitude * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

    // 自动计算持续时间（基于圈数）
    // 注意：必须修改 this->params.duration，而不是原始 params
    if (params.autoCalculateDuration || params.numCircles > 1) {
        double circleDuration = (2.0 * M_PI) / std::abs(turnRate);
        this->params.duration = circleDuration * params.numCircles;
    }
}

void ConstantTurn::update(double currentTime, double dt,
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

    // 更新姿态 - 对航向角进行归一化到 [-π, π]
    attitude.yaw = currentHeading;
    while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
    while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;

    attitude.pitch = 0.0;
    // 滚转角：正值表示右转时机翼向右倾斜，负值表示左转
    double bankAngle = std::atan(speed * speed / (Constants::G * turnRadius));
    attitude.roll = bankAngle * params.turnDirection;

    // 更新位置
    updatePositionFromVelocity(dt, position, velocity);
}

void ConstantTurn::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
}

// ============================================================================
// RacetrackPattern 跑道型盘旋模型实现
// ============================================================================
// 跑道型盘旋：矩形路径 = 直道段 + 180°转弯 + 直道段 + 180°转弯
void RacetrackPattern::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = LEG1_STRAIGHT;
    currentLap = 0;

    // 计算初始航向角和速度
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算转弯参数
    if (params.turnRadius > 0) {
        turnRadius = params.turnRadius;
    } else {
        turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    }
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce);

    // 计算各阶段持续时间
    double straightDuration = straightLength / initialSpeed;
    double turnDuration = (M_PI) / (turnRate * params.turnDirection);  // 180度转弯
    double totalLapDuration = 2.0 * straightDuration + 2.0 * turnDuration;

    t1 = straightDuration;
    t2 = t1 + turnDuration;
    t3 = t2 + straightDuration;
    t4 = t3 + turnDuration;

    // 自动计算总持续时间
    // 注意：必须修改 this->params.duration，而不是原始 params
    if (params.autoCalculateDuration) {
        this->params.duration = totalLapDuration * params.numLaps;
    }

    // 计算第一个转弯圆心（在第一段直道终点左侧/右侧）
    double turn1Angle = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    turn1CenterNorth = params.initialPosition.latitude +
        turnRadius * std::cos(turn1Angle) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    turn1CenterEast = params.initialPosition.longitude +
        turnRadius * std::sin(turn1Angle) / (Constants::EARTH_RADIUS *
        std::cos(params.initialPosition.latitude * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

    // 计算第二个转弯圆心（在第二段直道终点左侧/右侧）
    turn2CenterNorth = turn1CenterNorth;
    turn2CenterEast = turn1CenterEast;
}

void RacetrackPattern::update(double currentTime, double dt,
                              GeoPosition& position,
                              Velocity3& velocity,
                              AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载（平滑过渡）
    double targetG = 1.0;
    if (maneuverTime < t1 || (maneuverTime > t2 && maneuverTime < t3)) {
        targetG = 1.0;  // 直道阶段
    } else {
        targetG = params.targetGForce;  // 转弯阶段
    }
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    // 计算当前在哪个圈
    int lap = static_cast<int>(maneuverTime / t4);
    double timeInLap = maneuverTime - lap * t4;

    double currentHeading = initialHeading;

    if (timeInLap <= t1) {
        // LEG1_STRAIGHT: 第一段直道
        currentPhase = LEG1_STRAIGHT;
        currentHeading = initialHeading;
        attitude.roll = 0.0;
    } else if (timeInLap <= t2) {
        // LEG2_TURN: 第一个180度转弯
        currentPhase = LEG2_TURN;
        double turnTime = timeInLap - t1;
        double turnAngle = turnRate * params.turnDirection * turnTime;
        currentHeading = initialHeading + turnAngle;
        attitude.roll = std::atan2(initialSpeed * initialSpeed, Constants::G * turnRadius) * params.turnDirection;
    } else if (timeInLap <= t3) {
        // LEG3_STRAIGHT: 第二段直道（反向）
        currentPhase = LEG3_STRAIGHT;
        currentHeading = initialHeading + M_PI * params.turnDirection;
        attitude.roll = 0.0;
    } else {
        // LEG4_TURN: 第二个180度转弯
        currentPhase = LEG4_TURN;
        double turnTime = timeInLap - t3;
        double turnAngle = turnRate * params.turnDirection * turnTime;
        currentHeading = initialHeading + M_PI * params.turnDirection + turnAngle;
        attitude.roll = std::atan2(initialSpeed * initialSpeed, Constants::G * turnRadius) * params.turnDirection;
    }

    // 更新速度
    velocity.north = initialSpeed * std::cos(currentHeading);
    velocity.east = initialSpeed * std::sin(currentHeading);
    velocity.up = 0.0;

    // 更新姿态 - 对航向角进行归一化
    attitude.yaw = currentHeading;
    while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
    while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
    attitude.pitch = 0.0;

    // 更新位置
    updatePositionFromVelocity(dt, position, velocity);
}

void RacetrackPattern::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = LEG1_STRAIGHT;
    currentLap = 0;
}

// ============================================================================
// EightPattern 8字型盘旋模型实现
// ============================================================================
// 8字型盘旋：两个相切圆形成的8字形
void EightPattern::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = FIRST_CIRCLE;

    // 计算初始航向角和速度
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算转弯半径
    turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce);

    // 计算一个圆的持续时间
    circleDuration = (2.0 * M_PI) / turnRate;

    // 自动计算总持续时间
    // 注意：必须修改 this->params.duration，而不是原始 params
    if (params.autoCalculateDuration) {
        this->params.duration = circleDuration * 2.0;  // 两个圆
    }

    // 计算两个圆的圆心
    // 第一个圆的圆心在起始点左侧/右侧
    double angle1 = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    circle1CenterNorth = params.initialPosition.latitude +
        turnRadius * std::cos(angle1) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    circle1CenterEast = params.initialPosition.longitude +
        turnRadius * std::sin(angle1) / (Constants::EARTH_RADIUS *
        std::cos(params.initialPosition.latitude * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

    // 第二个圆的圆心在第一个圆对面，相切于起始点
    double angle2 = initialHeading + (params.turnDirection > 0 ? -M_PI/2 : M_PI/2);
    circle2CenterNorth = params.initialPosition.latitude +
        turnRadius * std::cos(angle2) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    circle2CenterEast = params.initialPosition.longitude +
        turnRadius * std::sin(angle2) / (Constants::EARTH_RADIUS *
        std::cos(params.initialPosition.latitude * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;
}

void EightPattern::update(double currentTime, double dt,
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

    double currentHeading;
    double currentLat, currentLon;

    if (maneuverTime < circleDuration) {
        // 第一个圆
        currentPhase = FIRST_CIRCLE;
        double angleProgress = maneuverTime / circleDuration;
        double angle = (3.0 * M_PI / 2.0) + 2.0 * M_PI * angleProgress * params.turnDirection;

        // 相对于第一个圆心的位置
        double offsetNorth = turnRadius * std::cos(angle);
        double offsetEast = turnRadius * std::sin(angle);

        currentLat = circle1CenterNorth + offsetNorth / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
        currentLon = circle1CenterEast + offsetEast / (Constants::EARTH_RADIUS *
            std::cos(currentLat * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

        // 速度方向（切线方向）
        double tangentAngle = angle + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
        currentHeading = tangentAngle;
    } else {
        // 第二个圆
        currentPhase = SECOND_CIRCLE;
        double secondCircleTime = maneuverTime - circleDuration;
        double angleProgress = secondCircleTime / circleDuration;
        double angle = (M_PI / 2.0) + 2.0 * M_PI * angleProgress * params.turnDirection;

        // 相对于第二个圆心的位置
        double offsetNorth = turnRadius * std::cos(angle);
        double offsetEast = turnRadius * std::sin(angle);

        currentLat = circle2CenterNorth + offsetNorth / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
        currentLon = circle2CenterEast + offsetEast / (Constants::EARTH_RADIUS *
            std::cos(currentLat * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

        // 速度方向（切线方向）
        double tangentAngle = angle + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
        currentHeading = tangentAngle;
    }

    // 更新位置
    position.latitude = currentLat;
    position.longitude = currentLon;

    // 更新速度
    velocity.north = initialSpeed * std::cos(currentHeading);
    velocity.east = initialSpeed * std::sin(currentHeading);
    velocity.up = 0.0;

    // 更新姿态 - 对航向角进行归一化
    attitude.yaw = currentHeading;
    while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
    while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
    attitude.pitch = 0.0;
    attitude.roll = std::atan(initialSpeed * initialSpeed / (Constants::G * turnRadius)) * params.turnDirection;
}

void EightPattern::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = FIRST_CIRCLE;
}

// ============================================================================
// ImmelmanEscape 置尾降高逃逸模型实现
// ============================================================================
// 置尾降高逃逸：最大速率掉转机头180度 + 下降至目标高度 + 改平
void ImmelmanEscape::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = TURN_PHASE;

    // 记录初始状态
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialAltitude = params.initialPosition.altitude;
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算转弯速率（弧度/秒）
    turnRate = params.maxTurnRate * Constants::DEG_TO_RAD;

    // 计算各阶段持续时间
    turnDuration = M_PI / turnRate;  // 180度转弯时间
    descentDuration = (initialAltitude - params.targetAltitude) / params.descentRate;  // 下降时间
    levelDuration = params.duration - turnDuration - descentDuration;  // 改平时间

    // 确保改平时间不为负
    if (levelDuration < 0) {
        levelDuration = 0;
    }

    descentRate = params.descentRate;
    targetAltitude = params.targetAltitude;
}

void ImmelmanEscape::update(double currentTime, double dt,
                            GeoPosition& position,
                            Velocity3& velocity,
                            AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;

    if (maneuverTime < turnDuration) {
        // TURN_PHASE: 掉转机头180度
        currentPhase = TURN_PHASE;
        currentGForce = 2.0;  // 转弯时略大于1G

        double turnProgress = maneuverTime / turnDuration;
        double currentHeading = initialHeading + M_PI * turnProgress * params.turnDirection;

        // 记录转弯结束位置
        turnEndLat = position.latitude;
        turnEndLon = position.longitude;
        turnEndAlt = position.altitude;

        // 更新速度和姿态
        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = 30.0 * Constants::DEG_TO_RAD * params.turnDirection;  // 转弯坡度

        // 位置沿直线更新
        updatePositionFromVelocity(dt, position, velocity);

    } else if (maneuverTime < turnDuration + descentDuration) {
        // DESCENT_PHASE: 下降至目标高度
        currentPhase = DESCENT_PHASE;
        currentGForce = 1.0;  // 下降时1G

        double descentTime = maneuverTime - turnDuration;
        double currentAlt = turnEndAlt - descentRate * descentTime;

        // 确保不低于目标高度
        if (currentAlt < targetAltitude) {
            currentAlt = targetAltitude;
        }

        // 航向保持转弯后的方向
        double currentHeading = initialHeading + M_PI * params.turnDirection;

        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = -descentRate;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = -15.0 * Constants::DEG_TO_RAD;  // 俯冲角
        attitude.roll = 0.0;

        position.altitude = currentAlt;
        updatePositionFromVelocity(dt, position, velocity);

    } else {
        // LEVEL_PHASE: 改平保持高度
        currentPhase = LEVEL_PHASE;
        currentGForce = 1.0;

        double currentHeading = initialHeading + M_PI * params.turnDirection;

        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = 0.0;

        updatePositionFromVelocity(dt, position, velocity);
    }
}

void ImmelmanEscape::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = TURN_PHASE;
}

// ============================================================================
// LPattern L型机动模型实现
// ============================================================================
// L型机动：直飞 → 90度转弯 → 直飞
void LPattern::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = LEG1_STRAIGHT;

    // 记录初始状态
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    leg1Distance = params.leg1Distance;
    turnAngle = params.turnAngle * Constants::DEG_TO_RAD;

    // 计算转弯半径
    turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce);

    // 计算各阶段持续时间
    leg1Duration = leg1Distance / initialSpeed;
    turnDuration = turnAngle / turnRate;
    leg2Duration = params.duration - leg1Duration - turnDuration;

    // 初始化转弯结束位置
    turnEndLat = params.initialPosition.latitude;
    turnEndLon = params.initialPosition.longitude;
    turnEndAlt = params.initialPosition.altitude;
}

void LPattern::update(double currentTime, double dt,
                      GeoPosition& position,
                      Velocity3& velocity,
                      AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载
    double targetG = 1.0;
    if (maneuverTime >= leg1Duration && maneuverTime < leg1Duration + turnDuration) {
        targetG = params.targetGForce;
    }
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    if (maneuverTime < leg1Duration) {
        // LEG1_STRAIGHT: 第一段直飞
        currentPhase = LEG1_STRAIGHT;

        velocity.north = initialSpeed * std::cos(initialHeading);
        velocity.east = initialSpeed * std::sin(initialHeading);
        velocity.up = 0.0;

        attitude.yaw = initialHeading;
        attitude.pitch = 0.0;
        attitude.roll = 0.0;

        updatePositionFromVelocity(dt, position, velocity);

        // 记录第一段结束位置
        if (maneuverTime + dt >= leg1Duration) {
            turnEndLat = position.latitude;
            turnEndLon = position.longitude;
            turnEndAlt = position.altitude;
        }

    } else if (maneuverTime < leg1Duration + turnDuration) {
        // TURN_PHASE: 转弯
        currentPhase = TURN_PHASE;

        double turnTime = maneuverTime - leg1Duration;
        double angleTurned = turnRate * turnTime * params.turnDirection;
        double currentHeading = initialHeading + angleTurned;

        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = std::atan(initialSpeed * initialSpeed / (Constants::G * turnRadius)) * params.turnDirection;

        updatePositionFromVelocity(dt, position, velocity);

        // 记录转弯结束位置
        if (maneuverTime + dt >= leg1Duration + turnDuration) {
            turnEndLat = position.latitude;
            turnEndLon = position.longitude;
            turnEndAlt = position.altitude;
        }

    } else {
        // LEG2_STRAIGHT: 第二段直飞
        currentPhase = LEG2_STRAIGHT;

        double finalHeading = initialHeading + turnAngle * params.turnDirection;

        velocity.north = initialSpeed * std::cos(finalHeading);
        velocity.east = initialSpeed * std::sin(finalHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = finalHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = 0.0;

        updatePositionFromVelocity(dt, position, velocity);
    }
}

void LPattern::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = LEG1_STRAIGHT;
}

// ============================================================================
// SPattern S型机动模型实现
// ============================================================================
// S型机动：两次反向转弯形成S形航线
void SPattern::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = FIRST_TURN;
    currentTurn = 0;

    // 记录初始状态
    initialHeading = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算转弯半径
    turnRadius = params.sTurnRadius;
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce);

    // 计算一次转弯的持续时间（90度）
    turnDuration = (M_PI / 2.0) / turnRate;

    // 计算第一个转弯的圆心
    double angle1 = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    turn1CenterNorth = params.initialPosition.latitude +
        turnRadius * std::cos(angle1) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    turn1CenterEast = params.initialPosition.longitude +
        turnRadius * std::sin(angle1) / (Constants::EARTH_RADIUS *
        std::cos(params.initialPosition.latitude * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;

    // 初始化第一个转弯结束位置
    turn1EndLat = params.initialPosition.latitude;
    turn1EndLon = params.initialPosition.longitude;
    turn1EndAlt = params.initialPosition.altitude;
}

void SPattern::update(double currentTime, double dt,
                      GeoPosition& position,
                      Velocity3& velocity,
                      AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载
    double targetG = params.targetGForce;
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    double currentHeading;

    if (maneuverTime < turnDuration) {
        // FIRST_TURN: 第一次转弯（90度）
        currentPhase = FIRST_TURN;
        currentTurn = 1;

        double turnProgress = maneuverTime / turnDuration;
        double angleTurned = (M_PI / 2.0) * turnProgress * params.turnDirection;
        currentHeading = initialHeading + angleTurned;

        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = std::atan(initialSpeed * initialSpeed / (Constants::G * turnRadius)) * params.turnDirection;

        updatePositionFromVelocity(dt, position, velocity);

        // 记录第一个转弯结束位置
        if (maneuverTime + dt >= turnDuration) {
            turn1EndLat = position.latitude;
            turn1EndLon = position.longitude;
            turn1EndAlt = position.altitude;

            // 计算第二个转弯的圆心（在第一个转弯结束点的相反方向）
            double headingAfterTurn1 = initialHeading + (M_PI / 2.0) * params.turnDirection;
            double angle2 = headingAfterTurn1 + (params.turnDirection > 0 ? -M_PI/2 : M_PI/2);
            turn2CenterNorth = turn1EndLat +
                turnRadius * std::cos(angle2) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
            turn2CenterEast = turn1EndLon +
                turnRadius * std::sin(angle2) / (Constants::EARTH_RADIUS *
                std::cos(turn1EndLat * Constants::DEG_TO_RAD)) * Constants::RAD_TO_DEG;
        }

    } else if (maneuverTime < 2.0 * turnDuration) {
        // SECOND_TURN: 第二次转弯（反向90度）
        currentPhase = SECOND_TURN;
        currentTurn = 2;

        double turn2Time = maneuverTime - turnDuration;
        double headingAfterTurn1 = initialHeading + (M_PI / 2.0) * params.turnDirection;
        double angleTurned = (M_PI / 2.0) * (turn2Time / turnDuration) * (-params.turnDirection);
        currentHeading = headingAfterTurn1 + angleTurned;

        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = currentHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = -std::atan(initialSpeed * initialSpeed / (Constants::G * turnRadius)) * params.turnDirection;

        updatePositionFromVelocity(dt, position, velocity);

    } else {
        // 完成S形后继续直飞
        double finalHeading = initialHeading;  // 两次反向90度转弯后回到原航向

        velocity.north = initialSpeed * std::cos(finalHeading);
        velocity.east = initialSpeed * std::sin(finalHeading);
        velocity.up = 0.0;

        // 对航向角进行归一化
        attitude.yaw = finalHeading;
        while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
        while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
        attitude.pitch = 0.0;
        attitude.roll = 0.0;

        updatePositionFromVelocity(dt, position, velocity);
    }
}

void SPattern::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = FIRST_TURN;
    currentTurn = 0;
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
        case Type::CONSTANT_TURN:
            return std::make_shared<ConstantTurn>();
        case Type::RACETRACK_PATTERN:
            return std::make_shared<RacetrackPattern>();
        case Type::EIGHT_PATTERN:
            return std::make_shared<EightPattern>();
        case Type::IMMELMAN_ESCAPE:
            return std::make_shared<ImmelmanEscape>();
        case Type::L_PATTERN:
            return std::make_shared<LPattern>();
        case Type::S_PATTERN:
            return std::make_shared<SPattern>();
        default:
            throw std::invalid_argument("Unknown maneuver type");
    }
}

Parameters Factory::getDefaultParams(Type type) {
    return Parameters::getDefault(type);
}
