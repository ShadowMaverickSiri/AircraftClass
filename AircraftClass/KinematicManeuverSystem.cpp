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
// 筋斗机动：在垂直平面内完成360°圆周运动
// 飞机从平飞开始（圆的左端），向上拉起，完成一圈回到平飞状态
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

    // 记录初始位置（筋斗起点：圆的左端，平飞状态）
    initialLatitude = params.initialPosition.latitude;
    initialLongitude = params.initialPosition.longitude;
    initialAltitude = params.initialPosition.altitude;

    // 计算圆心位置（在起始点前方 R 处，同一高度）
    loopCenterLatitude = initialLatitude + (loopRadius / Constants::EARTH_RADIUS) * std::cos(initialHeading) * Constants::RAD_TO_DEG;
    loopCenterLongitude = initialLongitude + (loopRadius / Constants::EARTH_RADIUS) * std::sin(initialHeading) * Constants::RAD_TO_DEG;
    loopCenterAltitude = initialAltitude;
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

    // 筋斗角度：从 3π/2 开始（圆的左端，平飞），逆时针旋转到 3π/2 + 2π
    double loopAngle = 3.0 * M_PI / 2.0 + 2.0 * M_PI * progress;

    // 筋斗是垂直平面内的圆周运动
    // 圆心在 loopCenterLatitude/Longitude/Altitude
    // 圆周上点的位置（相对于圆心）：
    //   水平方向：R * cos(loopAngle)
    //   垂直方向：R * sin(loopAngle)
    //
    // loopAngle = 3π/2: 水平位移=0, 垂直位移=-R  （起点，平飞）
    // loopAngle = 0:     水平位移=R, 垂直位移=0   （最高点，平飞）
    // loopAngle = π/2:   水平位移=0, 垂直位移=R  （最低点，平飞）
    // loopAngle = 3π/2: 水平位移=0, 垂直位移=-R  （完成一圈，回到起点）

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
    //   水平速度 = -initialSpeed * sin(loopAngle)
    //   垂直速度 = initialSpeed * cos(loopAngle)
    //
    // loopAngle = 3π/2: 水平速度=max, 垂直速度=0    （平飞，开始向上拉起）
    // loopAngle = 0:     水平速度=0, 垂直速度=max  （最高点，垂直向上）
    // loopAngle = π/2:   水平速度=-max, 垂直速度=0  （平飞，开始俯冲）
    // loopAngle = π:     水平速度=0, 垂直速度=-max （最低点，垂直向下）

    double horizontalSpeed = -initialSpeed * std::sin(loopAngle);
    double verticalSpeed = initialSpeed * std::cos(loopAngle);

    velocity.north = horizontalSpeed * std::cos(initialHeading);
    velocity.east = horizontalSpeed * std::sin(initialHeading);
    velocity.up = verticalSpeed;

    // 更新姿态角
    attitude.yaw = initialHeading;  // 航向保持不变

    // 俯仰角：飞机速度方向与水平面的夹角
    //   loopAngle = 3π/2: 俯仰角=0°   （平飞）
    //   loopAngle = 0:     俯仰角=90°  （垂直向上）
    //   loopAngle = π/2:   俯仰角=0°   （平飞）
    //   loopAngle = π:     俯仰角=-90° （垂直向下）
    double pitchAngle = loopAngle - 3.0 * M_PI / 2.0;
    // 规范化到 -π 到 π
    while (pitchAngle > M_PI) pitchAngle -= 2 * M_PI;
    while (pitchAngle < -M_PI) pitchAngle += 2 * M_PI;
    attitude.pitch = pitchAngle;

    // 滚转角：保持0（筋斗时机翼始终水平）
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
// 半滚倒转 = 滚转180° + 半筋斗向下
// 1. 滚转阶段：飞机平飞，滚转180°进入倒飞状态
// 2. 半筋斗阶段：在倒飞状态下推杆，完成向下的半圆，最终恢复正飞
void SplitS::initialize(const Parameters& params) {
    this->params = params;
    currentTime = 0.0;
    currentGForce = 1.0;

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
    // 滚转阶段：快速滚转180°
    rollDuration = 2.0;  // 2秒完成滚转
    rollRate = M_PI * params.rollDirection / rollDuration;  // 180度 / 2秒

    // 半筋斗阶段：在倒飞状态下完成半圆
    pitchDuration = params.duration - rollDuration;
    // 计算半筋斗半径（基于过载）
    loopRadius = (initialSpeed * initialSpeed) / (Constants::G * (params.targetGForce - 1.0));
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

        // 滚转角度：从0到180度（π）
        double rollAngle = rollRate * maneuverTime;
        // 规范化滚转角到 -π 到 π
        if (rollAngle > M_PI) rollAngle -= 2 * M_PI;
        if (rollAngle < -M_PI) rollAngle += 2 * M_PI;

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

        // 姿态：滚转，俯仰和航向不变
        attitude.yaw = initialYaw;
        attitude.pitch = 0.0;
        attitude.roll = rollAngle;

        // 记录滚转结束时的位置，作为半筋斗阶段的起点
        rollEndLatitude = position.latitude;
        rollEndLongitude = position.longitude;
        rollEndAltitude = position.altitude;

    } else {
        // ========== 第二阶段：倒飞状态下的半筋斗向下 ==========
        currentPhase = PUSH_DOWN_PHASE;
        double pitchPhaseTime = maneuverTime - rollDuration;
        double pitchProgress = pitchPhaseTime / pitchDuration;

        // 半筋斗角度：从 π/2（圆的顶端，倒飞平飞）顺时针到 -π/2（圆的底端，正飞平飞）
        // 这个半圆轨迹使得飞机从倒飞状态通过下半圆回到正飞状态
        double loopAngle = M_PI / 2.0 - M_PI * pitchProgress;

        // 过载：逐渐增加到目标过载
        double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(pitchProgress * 3.0, 1.0);
        applyGForceConstraint(targetG);
        currentGForce = targetG;

        // Split-S 半筋斗几何：
        // 圆心在起点下方 R 处（飞机在圆心上方 R 处，倒飞平飞）
        // 从 π/2 开始（顶端，倒飞），顺时针到 -π/2（底端，正飞）
        //
        // loopAngle = π/2:  水平偏移=0,   垂直偏移=R   （起点，圆心上方R，倒飞平飞）
        // loopAngle = 0:    水平偏移=R,   垂直偏移=0   （最低点，圆心处，垂直向下）
        // loopAngle = -π/2: 水平偏移=0,   垂直偏移=-R  （终点，圆心下方R，正飞平飞）
        //
        // 最终效果：高度损失 = 2R，航向反转 180°

        // 计算相对于圆心的偏移
        double horizontalOffset = loopRadius * std::cos(loopAngle);
        double verticalOffset = loopRadius * std::sin(loopAngle);

        // 圆心位置（在滚转结束点下方 R 处）
        double loopCenterAltitude = rollEndAltitude - loopRadius;

        // 位置计算
        // 水平偏移转换为经纬度（沿新航向）
        double horizontalDeg = horizontalOffset / Constants::EARTH_RADIUS * Constants::RAD_TO_DEG;

        // 新航向 = 初始航向 + 180°（反向）
        double reverseHeading = initialYaw + M_PI;
        if (reverseHeading > M_PI) reverseHeading -= 2 * M_PI;
        if (reverseHeading < -M_PI) reverseHeading += 2 * M_PI;

        // 位置 = 滚转结束点 + 水平偏移（沿反向航向）+ 高度偏移
        position.latitude = rollEndLatitude + horizontalDeg * std::cos(reverseHeading);
        position.longitude = rollEndLongitude + horizontalDeg * std::sin(reverseHeading);
        position.altitude = loopCenterAltitude + verticalOffset;

        // 速度沿半圆切线方向（顺时针）
        //   水平速度 = initialSpeed * sin(loopAngle)
        //   垂直速度 = -initialSpeed * cos(loopAngle)
        //
        // loopAngle = π/2:  水平速度=max,    垂直速度=0    （起点，倒飞平飞，向前）
        // loopAngle = 0:    水平速度=0,     垂直速度=max  （最低点，垂直向下）
        // loopAngle = -π/2: 水平速度=-max,  垂直速度=0    （终点，正飞平飞，向前）
        //
        // 注意：终点的水平速度为负，但这是相对于切线方向的。
        // 由于我们使用 reverseHeading，负的水平速度实际表示正向飞行

        double horizontalSpeed = initialSpeed * std::sin(loopAngle);
        double verticalSpeed = -initialSpeed * std::cos(loopAngle);

        // 使用原始航向计算速度分量（因为反向已经体现在水平速度的符号中）
        velocity.north = horizontalSpeed * std::cos(initialYaw);
        velocity.east = horizontalSpeed * std::sin(initialYaw);
        velocity.up = verticalSpeed;

        // 姿态角更新
        attitude.yaw = reverseHeading;  // 航向反向

        // Split-S 的姿态角：
        // 在半筋斗阶段，飞机保持倒飞状态（roll=180°），只是pitch角变化
        // 从倒飞平飞（pitch=π）→ 垂直向下（pitch=3π/2或-π/2）→ 正飞平飞（pitch=0或2π）
        //
        // 使用欧拉角表示：pitch 从 π 到 0（经过 3π/2）
        double pitchAngle = M_PI - M_PI * pitchProgress;  // π → 0
        // 规范化到 -π 到 π
        while (pitchAngle > M_PI) pitchAngle -= 2 * M_PI;
        while (pitchAngle < -M_PI) pitchAngle += 2 * M_PI;
        attitude.pitch = pitchAngle;

        // 滚转角：在整个半筋斗阶段保持倒飞状态（180°）
        // 在终点时，pitch已经到了0，roll也应该回到0表示正飞
        // 但实际上，我们用pitch=0来表示"完成了360°的俯仰旋转"
        // 所以roll应该保持180°，或者逐渐过渡到0
        //
        // 简化处理：roll随pitchProgress线性变化，从180°到0°
        double rollAngle = M_PI * params.rollDirection * (1.0 - pitchProgress);
        // 规范化到 -π 到 π
        if (rollAngle > M_PI) rollAngle -= 2 * M_PI;
        if (rollAngle < -M_PI) rollAngle += 2 * M_PI;
        attitude.roll = rollAngle;
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
