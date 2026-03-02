#include "KinematicManeuverSystem.h"
#include <algorithm>
#include <stdexcept>

using namespace KinematicManeuver;

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
            params.targetGForce = 2.0;
            params.duration = 12.0;
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

    // 计算筋斗半径
    loopRadius = (initialSpeed * initialSpeed) / (Constants::G * (params.targetGForce - 1.0));

    // 计算筋斗中心点
    loopCenterNorth = params.initialPosition.latitude + loopRadius * std::cos(initialHeading) / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    loopCenterUp = params.initialPosition.altitude;
}

void Loop::update(double currentTime, double dt,
                  GeoPosition& position,
                  Velocity3& velocity,
                  AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    // 计算当前过载
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);
    currentGForce = targetG;

    // 计算筋斗角度（0到2π）
    double loopAngle = 2.0 * M_PI * progress;

    // 更新位置（简化圆弧轨迹）
    double angleOffset = loopRadius / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;
    position.latitude = loopCenterNorth + angleOffset * std::cos(initialHeading + loopAngle);
    position.longitude = params.initialPosition.longitude + angleOffset * std::sin(initialHeading + loopAngle);
    position.altitude = loopCenterUp + loopRadius * std::sin(loopAngle);

    // 更新速度
    double currentHeading = initialHeading + loopAngle;
    velocity.north = initialSpeed * std::cos(currentHeading);
    velocity.east = initialSpeed * std::sin(currentHeading);
    velocity.up = initialSpeed * std::cos(loopAngle);

    // 更新姿态
    attitude.yaw = currentHeading;
    attitude.pitch = loopAngle;
    attitude.roll = 0.0;
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
// SplitS 模型实现
// ============================================================================
void SplitS::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;

    // 记录初始状态
    initialRoll = 0.0;
    initialPitch = 0.0;
    initialYaw = std::atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = std::sqrt(
        params.initialVelocity.north * params.initialVelocity.north +
        params.initialVelocity.east * params.initialVelocity.east
    );

    // 计算各阶段参数
    rollRate = M_PI * params.rollDirection / (params.duration * 0.3);  // 30%时间用于滚转
    pushDownRate = M_PI / (params.duration * 0.7);  // 70%时间用于推杆
}

void SplitS::update(double currentTime, double dt,
                    GeoPosition& position,
                    Velocity3& velocity,
                    AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;

    this->currentTime = currentTime;
    double maneuverTime = currentTime - params.startTime;
    double progress = maneuverTime / params.duration;

    if (progress < 0.3) {
        // ========== 第一阶段：滚转 ==========
        currentPhase = ROLL_PHASE;
        phaseTime = maneuverTime;
        currentGForce = 1.0;

        double rollAngle = rollRate * phaseTime;
        double currentHeading = initialYaw;

        // 计算飞行距离
        double distance = initialSpeed * phaseTime;
        double distanceDeg = distance / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;

        // 更新位置（沿直线飞行）
        position.latitude = params.initialPosition.latitude + distanceDeg * std::cos(currentHeading);
        position.longitude = params.initialPosition.longitude + distanceDeg * std::sin(currentHeading);
        position.altitude = params.initialPosition.altitude;

        // 速度保持不变
        velocity = params.initialVelocity;

        // 更新姿态
        attitude.yaw = currentHeading;
        attitude.pitch = initialPitch;
        attitude.roll = rollAngle;

    } else if (progress < 1.0) {
        // ========== 第二阶段：推杆 ==========
        currentPhase = PUSH_DOWN_PHASE;
        phaseTime = maneuverTime - params.duration * 0.3;
        double pushDownProgress = phaseTime / (params.duration * 0.7);

        // 计算过载（推杆时过载减小）
        currentGForce = 1.0 - 0.5 * pushDownProgress;

        // 计算俯仰角（从0度到-180度）
        double pitchAngle = -M_PI * pushDownProgress;

        // 计算航向角（180度转向）
        double headingChange = M_PI * pushDownProgress;
        double currentHeading = initialYaw + headingChange;

        // 计算转弯半径（简化模型）
        double turnRadius = (initialSpeed * initialSpeed) / (Constants::G * 0.5);
        double radiusDeg = turnRadius / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;

        // 更新位置（圆弧轨迹）
        position.latitude = params.initialPosition.latitude + radiusDeg * (std::cos(currentHeading) - std::cos(initialYaw));
        position.longitude = params.initialPosition.longitude + radiusDeg * (std::sin(currentHeading) - std::sin(initialYaw));
        position.altitude = params.initialPosition.altitude - turnRadius * (1.0 - std::cos(pitchAngle));

        // 更新速度
        velocity.north = initialSpeed * std::cos(currentHeading);
        velocity.east = initialSpeed * std::sin(currentHeading);
        velocity.up = -initialSpeed * std::sin(pitchAngle);

        // 更新姿态
        attitude.yaw = currentHeading;
        attitude.pitch = pitchAngle;
        attitude.roll = M_PI * params.rollDirection;  // 保持180度滚转
    }
}

void SplitS::reset() {
    currentTime = 0.0;
    currentGForce = 1.0;
    currentPhase = ROLL_PHASE;
    phaseTime = 0.0;
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
