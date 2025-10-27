#include "KinematicManeuverSystem.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

// 基类方法实现
bool KinematicManeuverModel::isActive(double currentTime) const {
    return currentTime >= params.startTime && 
           currentTime <= params.startTime + params.duration;
}

void KinematicManeuverModel::applyGForceConstraint(double& targetG) {
    targetG = std::max(1.0, std::min(9.0, targetG));
}

double KinematicManeuverModel::calculateTurnRadius(double speed, double gForce) const {
    if (gForce <= 1.0) return 1e6; // 避免除零错误
    return (speed * speed) / (9.81 * (gForce - 1.0));
}

double KinematicManeuverModel::calculateTurnRate(double speed, double gForce) const {
    if (speed <= 0.0 || gForce <= 1.0) return 0.0; // 避免除零错误
    return 9.81 * (gForce - 1.0) / speed;
}

void KinematicManeuverModel::LLAcalculate(double dt, GeoPosition& position, const Velocity3& velocity)
{
	double vx = velocity.north;
	double vy = velocity.up;
	double vz = velocity.east;
	double longt = position.longitude;
	double lati = position.latitude;
	double ht = position.altitude;
	double d_lonti = vz * Rad / ((Re + ht) * cos(lati / Rad));
	double d_lati = vx * Rad / (Re + ht);
	double d_ht = vy;
	position.longitude += d_lonti * dt;
	position.latitude += d_lati * dt;
	position.altitude += d_ht * dt;
}

// 水平转弯模型实现
void LevelTurnModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 计算初始航向角（弧度）
    initialHeading = atan2(params.initialVelocity.east, params.initialVelocity.north);
    
    // 计算初始速度大小（m/s）
    double initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                              params.initialVelocity.east * params.initialVelocity.east);
    
    // 计算转弯半径（米）和转弯速率（弧度/秒）
    turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce) * params.turnDirection;
    
    // 计算转弯中心点坐标（北向和东向）
    double perpendicularAngle = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    turnCenterNorth = params.initialPosition.longitude + turnRadius * cos(perpendicularAngle);
    turnCenterEast = params.initialPosition.latitude + turnRadius * sin(perpendicularAngle);
}

void LevelTurnModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 计算当前过载（平滑过渡）
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);
    currentGForce = targetG;
    
    // 计算当前航向角（弧度）
    double currentHeading = initialHeading + turnRate * maneuverTime;
    
    // 计算当前速度大小（保持恒定，m/s）
    double speed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    
    // 更新速度（北天东坐标系，m/s）
    velocity.north = speed * cos(currentHeading);
    velocity.east = speed * sin(currentHeading);
    velocity.up = 0.0; // 水平飞行，垂直速度为
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;
    attitude.pitch = 0.0; // 水平飞行，俯仰角为0
    attitude.roll = atan2(speed * speed, 9.81 * turnRadius) * params.turnDirection; // 倾斜角

    // 更新位置(经纬高)
    LLAcalculate(dt, position, velocity); 
    // 更新位置（沿圆弧运动）
    // position.longitude = turnCenterNorth + turnRadius * cos(currentHeading);
    // position.latitude = turnCenterEast + turnRadius * sin(currentHeading);
    // position.altitude = params.initialPosition.altitude; // 保持高度不变
    
}

void LevelTurnModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// 筋斗模型实现
void LoopModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 计算初始航向角（弧度）和初始速度大小（m/s）
    initialHeading = atan2(params.initialVelocity.east, params.initialVelocity.north);
    initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    
    // 计算筋斗半径（米）
    loopRadius = (initialSpeed * initialSpeed) / (9.81 * (params.targetGForce - 1.0));
    
    // 计算筋斗中心点坐标
    loopCenterNorth = params.initialPosition.longitude + loopRadius * cos(initialHeading);
    loopCenterUp = params.initialPosition.altitude;
}

void LoopModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 计算当前过载
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);
    currentGForce = targetG;
    
    // 计算筋斗角度（0到2π弧度）
    double loopAngle = 2.0 * M_PI * progress;
    
    // 更新位置
    position.longitude = loopCenterNorth + loopRadius * cos(initialHeading + loopAngle);
    position.latitude = params.initialPosition.latitude + loopRadius * sin(initialHeading + loopAngle);
    position.altitude = loopCenterUp + loopRadius * sin(loopAngle);
    
    // 更新速度（北天东坐标系，m/s）
    double currentHeading = initialHeading + loopAngle;
    velocity.north = initialSpeed * cos(currentHeading);
    velocity.east = initialSpeed * sin(currentHeading);
    velocity.up = initialSpeed * cos(loopAngle); // 垂直速度分量
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;
    attitude.pitch = loopAngle; // 俯仰角等于筋斗角度
    attitude.roll = 0.0; // 无滚转
}

void LoopModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// 横滚模型实现
void RollModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 记录初始姿态（弧度）
    initialRoll = 0.0;                        // 初始滚转角
    initialPitch = 0.0;                       // 初始俯仰角
    initialYaw = atan2(params.initialVelocity.east, params.initialVelocity.north);  // 初始偏航角
    
    // 计算滚转速率（完成360度滚转，弧度/秒）
    rollRate = 2.0 * M_PI * params.rollDirection / params.duration;
}

void RollModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 横滚时过载接近1G
    currentGForce = 1.0;
    
    // 计算滚转角度（弧度）
    double rollAngle = rollRate * maneuverTime;
    
    // 计算当前航向角（保持不变，弧度）
    double currentHeading = initialYaw;
    
    // 计算飞行距离（速度 × 时间，米）
    double speed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    double distance = speed * maneuverTime;
    
    // 更新位置（沿直线飞行）
    position.longitude = params.initialPosition.longitude + distance * cos(currentHeading);
    position.latitude = params.initialPosition.latitude + distance * sin(currentHeading);
    position.altitude = params.initialPosition.altitude; // 保持高度不变
    
    // 速度保持不变
    velocity = params.initialVelocity;
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;
    attitude.pitch = initialPitch;
    attitude.roll = rollAngle;
}

void RollModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// 半滚倒转模型实现
void SplitSModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 记录初始状态
    initialRoll = 0.0;                        // 初始滚转角（弧度）
    initialPitch = 0.0;                       // 初始俯仰角（弧度）
    initialYaw = atan2(params.initialVelocity.east, params.initialVelocity.north);  // 初始偏航角（弧度）
    initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);  // 初始速度大小（m/s）
    
    // 计算各阶段参数
    rollRate = M_PI * params.rollDirection / (params.duration * 0.3); // 30%时间用于滚转（弧度/秒）
    pushDownRate = M_PI / (params.duration * 0.7); // 70%时间用于推杆（弧度/秒）
}

void SplitSModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    // 确定当前阶段
    if (progress < 0.3) {
        // 第一阶段：滚转
        currentPhase = ROLL_PHASE;            // 设置当前阶段为滚转阶段
        phaseTime = maneuverTime;             // 当前阶段的时间
        currentGForce = 1.0;                  // 滚转时过载为1G
        
        double rollAngle = rollRate * phaseTime;  // 计算滚转角度（弧度）
        double currentHeading = initialYaw;       // 当前航向角（保持不变）
        
        // 计算飞行距离（速度 × 时间，米）
        double distance = initialSpeed * phaseTime;
        
        // 更新位置（沿直线飞行）
        position.longitude = params.initialPosition.longitude + distance * cos(currentHeading);
        position.latitude = params.initialPosition.latitude + distance * sin(currentHeading);
        position.altitude = params.initialPosition.altitude; // 保持高度不变
        
        // 速度保持不变
        velocity = params.initialVelocity;
        
        // 更新姿态（弧度）
        attitude.yaw = currentHeading;
        attitude.pitch = initialPitch;
        attitude.roll = rollAngle;
        
    } else if (progress < 1.0) {
        // 第二阶段：推杆
        currentPhase = PUSH_DOWN_PHASE;       // 设置当前阶段为推杆阶段
        phaseTime = maneuverTime - params.duration * 0.3;  // 当前阶段的时间
        double pushDownProgress = phaseTime / (params.duration * 0.7);  // 推杆阶段进度（0-1）
        
        // 计算过载（推杆时过载减小）
        currentGForce = 1.0 - 0.5 * pushDownProgress;
        
        // 计算俯仰角（从0度到-180度，弧度）
        double pitchAngle = -M_PI * pushDownProgress;
        
        // 计算航向角（180度转向，弧度）
        double headingChange = M_PI * pushDownProgress;
        double currentHeading = initialYaw + headingChange;
        
        // 更新位置（简化的圆弧轨迹）
        double turnRadius = (initialSpeed * initialSpeed) / (9.81 * 0.5); // 使用0.5G计算转弯半径（米）
        position.longitude = params.initialPosition.longitude + turnRadius * (cos(currentHeading) - cos(initialYaw));
        position.latitude = params.initialPosition.latitude + turnRadius * (sin(currentHeading) - sin(initialYaw));
        position.altitude = params.initialPosition.altitude - turnRadius * (1.0 - cos(pitchAngle));
        

        // 更新速度（北天东坐标系，m/s）
        velocity.north = initialSpeed * cos(currentHeading);
        velocity.east = initialSpeed * sin(currentHeading);
        velocity.up = -initialSpeed * sin(pitchAngle);  // 负值表示下降
        
        // 更新姿态（弧度）
        attitude.yaw = currentHeading;
        attitude.pitch = pitchAngle;
        attitude.roll = M_PI * params.rollDirection; // 保持180度滚转
    }
}

void SplitSModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
    currentPhase = ROLL_PHASE;                // 重置当前阶段为滚转阶段
    phaseTime = 0.0;                          // 重置阶段时间为0
}

// 工厂类实现
std::shared_ptr<KinematicManeuverModel> KinematicManeuverFactory::create(ManeuverType type) {
    switch (type) {
        case ManeuverType::LEVEL_TURN:
            return std::make_shared<LevelTurnModel>();
        case ManeuverType::LOOP:
            return std::make_shared<LoopModel>();
        case ManeuverType::ROLL:
            return std::make_shared<RollModel>();
        case ManeuverType::SPLIT_S:
            return std::make_shared<SplitSModel>();
        default:
            throw std::invalid_argument("Unknown maneuver type");
    }
}

KinematicManeuverParameters KinematicManeuverFactory::getDefaultParams(ManeuverType type) {
    KinematicManeuverParameters params;       // 创建默认参数结构体
    params.type = type;                       // 设置机动类型
    
    switch (type) {
        case ManeuverType::LEVEL_TURN:
            params.targetGForce = 3.0;        // 目标过载3G
            params.duration = 15.0;           // 持续时间15秒
            params.turnDirection = 1.0;       // 右转
            break;
        case ManeuverType::LOOP:
            params.targetGForce = 4.0;        // 目标过载4G
            params.duration = 20.0;           // 持续时间20秒
            break;
        case ManeuverType::ROLL:
            params.targetGForce = 1.0;        // 目标过载1G
            params.duration = 5.0;            // 持续时间5秒
            params.rollDirection = 1.0;       // 右滚
            break;
        case ManeuverType::SPLIT_S:
            params.targetGForce = 2.0;        // 目标过载2G
            params.duration = 12.0;           // 持续时间12秒
            params.rollDirection = 1.0;       // 右滚
            break;
    }
    
    return params;
}
