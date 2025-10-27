/**
 * @file KinematicManeuverSystem.cpp
 * @brief 运动学机动系统实现文件
 * @details 实现了各种飞机机动飞行的运动学模型，包括水平转弯、筋斗、横滚、半滚倒转等
 * @author 飞机仿真系统
 * @date 2024
 */

#include "KinematicManeuverSystem.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream> // Added for debugging

// ============================================================================
// 基类方法实现
// ============================================================================

/**
 * @brief 检查机动是否处于活跃状态
 * @param currentTime 当前仿真时间 (秒)
 * @return true表示机动正在进行中，false表示机动未开始或已结束
 * @details 通过比较当前时间与机动的开始时间和结束时间来判断机动状态
 */
bool KinematicManeuverModel::isActive(double currentTime) const {
    return currentTime >= params.startTime && 
           currentTime <= params.startTime + params.duration;
}

/**
 * @brief 应用过载约束
 * @param targetG 目标过载值 (输入输出参数)
 * @details 将过载值限制在合理范围内，防止过载过大或过小
 *          最小过载为1.0G（平飞），最大过载为9.0G（人体承受极限）
 */
void KinematicManeuverModel::applyGForceConstraint(double& targetG) {
    targetG = std::max(1.0, std::min(9.0, targetG));
}

/**
 * @brief 计算转弯半径
 * @param speed 飞行速度 (m/s)
 * @param gForce 过载值 (G)
 * @return 转弯半径 (m)
 * @details 基于向心力公式计算转弯半径：R = V²/(g*(n-1))
 *          其中V为速度，g为重力加速度，n为过载值
 *          当过载≤1G时返回极大值，避免除零错误
 */
double KinematicManeuverModel::calculateTurnRadius(double speed, double gForce) const {
    if (gForce <= 1.0) return 1e6; // 避免除零错误，返回极大半径
    return (speed * speed) / (9.81 * (gForce - 1.0));
}

/**
 * @brief 计算转弯角速度
 * @param speed 飞行速度 (m/s)
 * @param gForce 过载值 (G)
 * @return 转弯角速度 (rad/s)
 * @details 基于向心力公式计算转弯角速度：ω = g*(n-1)/V
 *          当速度≤0或过载≤1G时返回0，避免除零错误
 */
double KinematicManeuverModel::calculateTurnRate(double speed, double gForce) const {
    if (speed <= 0.0 || gForce <= 1.0) return 0.0; // 避免除零错误
    return 9.81 * (gForce - 1.0) / speed;
}

/**
 * @brief 根据北天东速度计算经纬高变化
 * @param dt 时间步长 (秒)
 * @param position 位置信息 (经纬高，输入输出参数)
 * @param velocity 速度信息 (北天东坐标系，m/s)
 * @details 基于地球椭球模型计算位置变化，考虑地球曲率影响
 * 
 * 算法原理：
 * 1. 将北天东速度分解为北向、天向、东向分量
 * 2. 基于地球半径和当前位置计算经纬度变化率
 * 3. 考虑纬度对经度变化的影响（纬度越高，相同东向速度对应的经度变化越小）
 * 4. 积分更新位置信息
 * 
 * 注意事项：
 * - 经度变化需要考虑纬度的影响，使用cos(latitude)修正
 * - 地球半径需要加上当前高度
 * - 所有角度计算使用弧度，最终结果转换为度
 */
void KinematicManeuverModel::LLAcalculate(double dt, GeoPosition& position, const Velocity3& velocity)
{
	// 提取速度分量（北天东坐标系）
	double vx = velocity.north;  // 北向速度 (m/s)
	double vy = velocity.up;     // 天向速度 (m/s)
	double vz = velocity.east;   // 东向速度 (m/s)
	
	// 提取当前位置信息
	double longt = position.longitude;  // 经度（度）
	double lati = position.latitude;    // 纬度（度）
	double ht = position.altitude;      // 高度（米）
	
	// 将纬度转换为弧度进行计算
	double lati_rad = lati * Constants::DEG_TO_RAD;
	
	// 计算经纬度变化率（度/秒）
	// 经度变化率：考虑纬度影响，纬度越高相同东向速度对应的经度变化越小
	double d_lonti = vz * Constants::RAD_TO_DEG / ((Constants::EARTH_RADIUS + ht) * cos(lati_rad));
	// 纬度变化率：北向速度直接对应纬度变化
	double d_lati = vx * Constants::RAD_TO_DEG / (Constants::EARTH_RADIUS + ht);
	// 高度变化率：天向速度直接对应高度变化
	double d_ht = vy;
	
	// 积分更新位置（度）
	position.longitude += d_lonti * dt;  // 更新经度
	position.latitude += d_lati * dt;    // 更新纬度
	position.altitude += d_ht * dt;      // 更新高度
}

// ============================================================================
// 水平转弯模型实现
// ============================================================================

/**
 * @brief 初始化水平转弯模型
 * @param params 机动参数，包含目标过载、转弯方向、初始状态等
 * @details 根据给定参数计算转弯的关键参数，包括转弯半径、转弯速率、转弯中心等
 * 
 * 初始化过程：
 * 1. 计算初始航向角和速度大小
 * 2. 基于向心力公式计算转弯半径和转弯速率
 * 3. 根据转弯方向计算转弯中心点坐标
 * 4. 设置初始状态变量
 */
void LevelTurnModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 计算初始航向角（弧度）
    // 使用atan2函数计算航向角，确保角度在正确象限
    initialHeading = atan2(params.initialVelocity.east, params.initialVelocity.north);
    
    // 计算初始速度大小（m/s）
    // 只考虑水平速度分量，忽略垂直速度
    double initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                              params.initialVelocity.east * params.initialVelocity.east);
    
    // 计算转弯半径（米）和转弯速率（弧度/秒）
    // 基于向心力公式：R = V²/(g*(n-1)), ω = g*(n-1)/V
    turnRadius = calculateTurnRadius(initialSpeed, params.targetGForce);
    turnRate = calculateTurnRate(initialSpeed, params.targetGForce) * params.turnDirection;
    
    // 计算转弯中心点坐标（北向和东向）
    // 转弯中心位于初始位置的法向方向，距离为转弯半径
    double perpendicularAngle = initialHeading + (params.turnDirection > 0 ? M_PI/2 : -M_PI/2);
    turnCenterNorth = params.initialPosition.longitude + turnRadius * cos(perpendicularAngle);
    turnCenterEast = params.initialPosition.latitude + turnRadius * sin(perpendicularAngle);
}

/**
 * @brief 更新水平转弯模型状态
 * @param currentTime 当前仿真时间 (秒)
 * @param dt 仿真时间步长 (秒)
 * @param position 飞机位置 (经纬高，输入输出参数)
 * @param velocity 飞机速度 (北天东坐标系，输入输出参数)
 * @param attitude 飞机姿态角 (俯仰、滚转、偏航，输入输出参数)
 * @details 根据当前时间更新飞机的运动状态，实现水平转弯机动
 * 
 * 更新过程：
 * 1. 检查机动是否处于活跃状态
 * 2. 计算机动进度和当前过载（平滑过渡）
 * 3. 根据转弯速率更新航向角
 * 4. 计算速度分量和姿态角
 * 5. 使用LLA计算更新位置
 */
void LevelTurnModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    // 检查机动是否处于活跃状态，如果不在活跃期则直接返回
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 计算当前过载（平滑过渡）
    // 过载从1G逐渐增加到目标过载，前1/3时间内完成过渡
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);  // 应用过载约束
    currentGForce = targetG;
    
    // 计算当前航向角（弧度）
    // 航向角随时间线性变化，变化速率为turnRate
    double currentHeading = initialHeading + turnRate * maneuverTime;
    
    // 计算当前速度大小（保持恒定，m/s）
    // 水平转弯中速度大小保持不变，只改变方向
    double speed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    
    // 更新速度（北天东坐标系，m/s）
    // 根据当前航向角计算速度分量
    velocity.north = speed * cos(currentHeading);  // 北向速度分量
    velocity.east = speed * sin(currentHeading);   // 东向速度分量
    velocity.up = 0.0; // 水平飞行，垂直速度为0
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;  // 偏航角等于当前航向角
    attitude.pitch = 0.0; // 水平飞行，俯仰角为0
    // 滚转角基于向心力计算，用于产生转弯所需的向心力
    attitude.roll = atan2(speed * speed, 9.81 * turnRadius) * params.turnDirection;

    // 使用LLA计算更新位置(经纬高)
    // 基于速度积分计算位置变化，考虑地球曲率影响
    LLAcalculate(dt, position, velocity); 
    
    // 注释掉的代码：直接基于圆弧运动的位置更新方法
    // position.longitude = turnCenterNorth + turnRadius * cos(currentHeading);
    // position.latitude = turnCenterEast + turnRadius * sin(currentHeading);
    // position.altitude = params.initialPosition.altitude; // 保持高度不变
}

/**
 * @brief 重置水平转弯模型
 * @details 将模型重置到初始状态，清除所有内部状态变量
 */
void LevelTurnModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// ============================================================================
// 筋斗模型实现
// ============================================================================

/**
 * @brief 初始化筋斗模型
 * @param params 机动参数，包含目标过载、持续时间、初始状态等
 * @details 根据给定参数计算筋斗的关键参数，包括筋斗半径、初始状态等
 * 
 * 初始化过程：
 * 1. 计算初始航向角和速度大小
 * 2. 基于向心力公式计算筋斗半径
 * 3. 设置筋斗中心点坐标
 * 4. 设置初始状态变量
 */
void LoopModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 计算初始航向角（弧度）和初始速度大小（m/s）
    // 使用atan2函数计算航向角，确保角度在正确象限
    initialHeading = atan2(params.initialVelocity.east, params.initialVelocity.north);
    // 只考虑水平速度分量，忽略垂直速度
    initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    
    // 计算筋斗半径（米）
    // 基于向心力公式：R = V²/(g*(n-1))，其中n为过载值
    loopRadius = (initialSpeed * initialSpeed) / (9.81 * (params.targetGForce - 1.0));
    
    // 计算筋斗中心点坐标
    // 筋斗中心位于初始位置前方，距离为筋斗半径
    loopCenterNorth = params.initialPosition.longitude + loopRadius * cos(initialHeading);
    loopCenterUp = params.initialPosition.altitude;  // 筋斗中心高度与初始高度相同
}

/**
 * @brief 更新筋斗模型状态
 * @param currentTime 当前仿真时间 (秒)
 * @param dt 仿真时间步长 (秒)
 * @param position 飞机位置 (经纬高，输入输出参数)
 * @param velocity 飞机速度 (北天东坐标系，输入输出参数)
 * @param attitude 飞机姿态角 (俯仰、滚转、偏航，输入输出参数)
 * @details 根据当前时间更新飞机的运动状态，实现筋斗机动
 * 
 * 筋斗机动特点：
 * 1. 在垂直平面内画完整圆形轨迹
 * 2. 航向角保持不变
 * 3. 俯仰角从0度变化到360度
 * 4. 速度大小保持不变，方向沿圆形轨迹切线
 * 
 * 更新过程：
 * 1. 检查机动开始时间
 * 2. 计算机动进度和当前过载
 * 3. 计算筋斗角度和俯仰角
 * 4. 计算速度分量
 * 5. 更新位置和姿态
 */
void LoopModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    // 只检查开始时间，筋斗机动一旦开始就持续到完成
    if (currentTime < params.startTime) return;
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 计算当前过载（平滑过渡）
    // 过载从1G逐渐增加到目标过载，前1/3时间内完成过渡
    double targetG = 1.0 + (params.targetGForce - 1.0) * std::min(progress * 3.0, 1.0);
    applyGForceConstraint(targetG);  // 应用过载约束
    currentGForce = targetG;
    
    // 关键修正：筋斗是在垂直平面内画一个完整的圆
    // 航向角保持不变，因为筋斗在垂直平面内
    double currentHeading = initialHeading;  // 航向角不变
    
    // 计算筋斗角度（0到2π弧度，完整的圆）
    // 筋斗角度从0度开始，随时间线性增加到360度
    double loopAngle = 2.0 * M_PI * progress;
    
    // 计算俯仰角：基于筋斗角度，在垂直平面内画圆
    // 从0°开始，逆时针画圆：0° → 90° → 180° → 270° → 360°
    // 0°: 水平飞行，90°: 垂直向上，180°: 倒飞，270°: 垂直向下，360°: 恢复水平
    double pitchAngle = loopAngle;
    
    // 计算速度分量（沿着圆的切线方向）
    // 1. 水平速度分量（在垂直平面内，水平速度基于俯仰角）
    // 当俯仰角为0°或180°时，水平速度最大；当俯仰角为90°或270°时，水平速度为0
    double horizontalSpeed = initialSpeed * cos(pitchAngle);
    
    // 2. 垂直速度分量（在垂直平面内，垂直速度基于俯仰角）
    // 当俯仰角为90°时，垂直速度最大（向上）；当俯仰角为270°时，垂直速度最大（向下）
    double verticalSpeed = initialSpeed * sin(pitchAngle);
    
    // 3. 将速度分解到地理坐标系（航向角不变）
    // 将垂直平面内的速度分量投影到北天东坐标系
    velocity.north = horizontalSpeed * cos(currentHeading);  // 北向速度分量
    velocity.east = horizontalSpeed * sin(currentHeading);   // 东向速度分量
    velocity.up = verticalSpeed;                             // 天向速度分量
    
    // 使用LLAcalculate函数更新位置
    // 基于速度积分计算位置变化，考虑地球曲率影响
    LLAcalculate(dt, position, velocity);
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;        // 航向角保持不变
    attitude.pitch = pitchAngle;          // 俯仰角等于筋斗角度
    attitude.roll = 0.0;                 // 无滚转，筋斗在垂直平面内进行
}

/**
 * @brief 重置筋斗模型
 * @details 将模型重置到初始状态，清除所有内部状态变量
 */
void LoopModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// ============================================================================
// 横滚模型实现
// ============================================================================

/**
 * @brief 初始化横滚模型
 * @param params 机动参数，包含滚转方向、持续时间、初始状态等
 * @details 根据给定参数计算横滚的关键参数，包括滚转速率、初始姿态等
 * 
 * 初始化过程：
 * 1. 记录初始姿态角
 * 2. 计算滚转速率（完成360度滚转）
 * 3. 设置初始状态变量
 */
void RollModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 记录初始姿态（弧度）
    initialRoll = 0.0;                        // 初始滚转角
    initialPitch = 0.0;                       // 初始俯仰角
    // 使用atan2函数计算初始偏航角，确保角度在正确象限
    initialYaw = atan2(params.initialVelocity.east, params.initialVelocity.north);
    
    // 计算滚转速率（完成360度滚转，弧度/秒）
    // 滚转速率 = 2π / 持续时间，考虑滚转方向
    rollRate = 2.0 * M_PI * params.rollDirection / params.duration;
}

/**
 * @brief 更新横滚模型状态
 * @param currentTime 当前仿真时间 (秒)
 * @param dt 仿真时间步长 (秒)
 * @param position 飞机位置 (经纬高，输入输出参数)
 * @param velocity 飞机速度 (北天东坐标系，输入输出参数)
 * @param attitude 飞机姿态角 (俯仰、滚转、偏航，输入输出参数)
 * @details 根据当前时间更新飞机的运动状态，实现横滚机动
 * 
 * 横滚机动特点：
 * 1. 飞机绕纵轴旋转360度
 * 2. 航向角和俯仰角保持不变
 * 3. 速度大小和方向保持不变
 * 4. 过载接近1G（平飞状态）
 * 
 * 更新过程：
 * 1. 检查机动是否处于活跃状态
 * 2. 计算机动时间
 * 3. 计算滚转角度
 * 4. 更新位置和姿态
 */
void RollModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    // 检查机动是否处于活跃状态
    if (!isActive(currentTime)) return;
    
    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 横滚时过载接近1G（平飞状态）
    currentGForce = 1.0;
    
    // 计算滚转角度（弧度）
    // 滚转角度随时间线性变化，变化速率为rollRate
    double rollAngle = rollRate * maneuverTime;
    
    // 计算当前航向角（保持不变，弧度）
    double currentHeading = initialYaw;
    
    // 使用LLAcalculate函数更新位置
    // 基于速度积分计算位置变化，考虑地球曲率影响
    LLAcalculate(dt, position, velocity);
    
    // 速度保持不变（横滚机动不改变飞行速度）
    velocity = params.initialVelocity;
    
    // 更新姿态（弧度）
    attitude.yaw = currentHeading;    // 偏航角保持不变
    attitude.pitch = initialPitch;    // 俯仰角保持不变
    attitude.roll = rollAngle;        // 滚转角随时间变化
}

/**
 * @brief 重置横滚模型
 * @details 将模型重置到初始状态，清除所有内部状态变量
 */
void RollModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
}

// ============================================================================
// 半滚倒转模型实现
// ============================================================================

/**
 * @brief 半滚倒转（Split-S）机动说明
 * @details 半滚倒转是一种典型的战斗机机动，分为两个阶段：
 * 
 * 第一阶段（横滚阶段）：
 * - 飞机先横滚180°进入倒飞状态
 * - 持续时间：总时间的30%
 * - 过载：1G（平飞状态）
 * 
 * 第二阶段（筋斗阶段）：
 * - 在倒飞状态下拉杆做半个筋斗，使机头向下转过180°
 * - 持续时间：总时间的70%
 * - 过载：从1G逐渐增加到3G
 * 
 * 机动效果：
 * - 迅速将高度转化为速度
 * - 改变航向180°
 * - 是空战中的常用机动
 */

/**
 * @brief 初始化半滚倒转模型
 * @param params 机动参数，包含目标过载、持续时间、滚转方向、初始状态等
 * @details 根据给定参数计算半滚倒转的关键参数，包括各阶段时间分配、速率等
 * 
 * 初始化过程：
 * 1. 记录初始状态（姿态、速度）
 * 2. 计算各阶段时间分配
 * 3. 计算横滚速率和筋斗速率
 * 4. 设置初始状态变量
 */
void SplitSModel::initialize(const KinematicManeuverParameters& params) {
    this->params = params;                    // 存储机动参数
    currentTime = 0.0;                        // 初始化当前时间为0
    currentGForce = 1.0;                      // 初始化当前过载为1G
    
    // 记录初始状态
    initialRoll = 0.0;                        // 初始滚转角（弧度）
    initialPitch = 0.0;                       // 初始俯仰角（弧度）
    // 使用atan2函数计算初始偏航角，确保角度在正确象限
    initialYaw = atan2(params.initialVelocity.east, params.initialVelocity.north);
    // 计算初始速度大小，只考虑水平速度分量
    initialSpeed = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                       params.initialVelocity.east * params.initialVelocity.east);
    
    // 计算各阶段参数
    // 横滚速率：在30%的时间内完成180度横滚
    rollRate = M_PI * params.rollDirection / (params.duration * 0.3);
    // 筋斗速率：在70%的时间内完成180度筋斗（半个筋斗）
    pushDownRate = M_PI / (params.duration * 0.7);
}

/**
 * @brief 更新半滚倒转模型状态
 * @param currentTime 当前仿真时间 (秒)
 * @param dt 仿真时间步长 (秒)
 * @param position 飞机位置 (经纬高，输入输出参数)
 * @param velocity 飞机速度 (北天东坐标系，输入输出参数)
 * @param attitude 飞机姿态角 (俯仰、滚转、偏航，输入输出参数)
 * @details 根据当前时间更新飞机的运动状态，实现半滚倒转机动
 * 
 * 半滚倒转机动分为两个阶段：
 * 1. 横滚阶段（0-30%时间）：飞机横滚180度进入倒飞
 * 2. 筋斗阶段（30-100%时间）：在倒飞状态下拉杆做半个筋斗
 * 
 * 更新过程：
 * 1. 检查机动开始时间
 * 2. 计算机动进度
 * 3. 根据进度确定当前阶段
 * 4. 执行相应阶段的运动计算
 * 5. 更新位置和姿态
 */
void SplitSModel::update(double currentTime, double dt, GeoPosition& position, Velocity3& velocity, AttitudeAngles& attitude) {
    // 只检查开始时间，半滚倒转机动一旦开始就持续到完成
	if (currentTime < params.startTime) return;

    this->currentTime = currentTime;          // 更新当前时间
    double maneuverTime = currentTime - params.startTime;  // 计算机动开始后的时间
    double progress = maneuverTime / params.duration;      // 计算机动进度（0-1）
    
    // dt: 仿真步长（秒），可用于内部模型解算的精度控制
    
    // 根据机动进度确定当前阶段
    if (progress < 0.3) {
        // ========================================
        // 第一阶段：横滚阶段（0-30%时间）
        // ========================================
        currentPhase = ROLL_PHASE;            // 设置当前阶段为滚转阶段
        phaseTime = maneuverTime;             // 当前阶段的时间
        currentGForce = 1.0;                  // 滚转时过载为1G（平飞状态）
        
        // 计算滚转角度（弧度）
        // 滚转角度从0度线性增加到180度，完成倒飞
        double rollAngle = rollRate * phaseTime;
        
        // 横滚阶段特点：
        // - 保持航向和俯仰角不变
        // - 只改变滚转角
        // - 速度保持不变
        // - 过载为1G
        
        // 速度保持不变，但确保坐标系一致
        velocity.north = params.initialVelocity.north;
        velocity.east = params.initialVelocity.east;
        velocity.up = params.initialVelocity.up;
        
        // 使用LLAcalculate函数更新位置
        // 基于速度积分计算位置变化，考虑地球曲率影响
        LLAcalculate(dt, position, velocity);
        
        // 更新姿态（弧度）
        attitude.yaw = initialYaw;        // 滚转阶段航向不变
        attitude.pitch = initialPitch;    // 俯仰角不变
        attitude.roll = rollAngle;        // 滚转角从0度逐渐增加到180度（倒飞）
        
    } else if (progress <= 0.999999) {
        // ========================================
        // 第二阶段：筋斗阶段（30-100%时间）
        // ========================================
        currentPhase = PUSH_DOWN_PHASE;        // 设置当前阶段为筋斗阶段
        phaseTime = maneuverTime - params.duration * 0.3;  // 当前阶段的时间
        double loopProgress = phaseTime / (params.duration * 0.7);  // 筋斗阶段进度（0-1）
        
        // 计算过载（筋斗时过载增大）
        // 过载从1G逐渐增加到3G，模拟拉杆做筋斗时的过载变化
        currentGForce = 1.0 + 2.0 * loopProgress;
        
        // 关键修正：这是半个筋斗，不是简单的俯仰角变化
        // 筋斗角度从0到π弧度（半个筋斗，180度）
        double loopAngle = M_PI * loopProgress;
        
        // 在倒飞状态下，拉杆做向下的筋斗
        // 俯仰角从0度变化到-π弧度（-180°），表示向下筋斗
        // 负号表示在倒飞状态下，拉杆使机头向下
        double pitchAngle = -loopAngle;
        
        // 航向角瞬间跳变180度，不是逐渐变化
        // 当筋斗角度达到90度时，航向角跳变180度
        double currentHeading = initialYaw;
        if(loopAngle >= M_PI/2 && isFirst == 0){
            currentHeading = initialYaw + M_PI;  // 航向角跳变180度
            isFirst = true;  // 设置标志位，避免重复跳变
        }
        
        // 计算速度分量（基于筋斗运动）
        // 1. 计算水平速度分量
        // 当俯仰角为0°或180°时，水平速度最大；当俯仰角为90°或270°时，水平速度为0
        double horizontalSpeed = initialSpeed * cos(pitchAngle);
        
        // 2. 计算垂直速度分量
        // 当俯仰角为-90°时，垂直速度最大（向下）；当俯仰角为90°时，垂直速度最大（向上）
        double verticalSpeed = initialSpeed * sin(pitchAngle);
        
        // 3. 将速度分解到地理坐标系
        // 将垂直平面内的速度分量投影到北天东坐标系
        velocity.north = horizontalSpeed * cos(currentHeading);  // 北向速度分量
        velocity.east = horizontalSpeed * sin(currentHeading);   // 东向速度分量
        velocity.up = verticalSpeed;                             // 天向速度分量
        
        // 调试信息：打印关键参数（每秒打印一次）
        if (fmod(phaseTime, 1.0) < dt) {
            std::cout << "筋斗阶段 - 时间: " << phaseTime 
                      << ", 筋斗角度: " << loopAngle * 180.0 / M_PI 
                      << "°, 俯仰角: " << pitchAngle * 180.0 / M_PI 
                      << "°, 航向角: " << currentHeading * 180.0 / M_PI 
                      << "°, 垂直速度: " << velocity.up 
                      << " m/s, 高度: " << position.altitude << " m" << std::endl;
        }
        
        // 使用LLAcalculate函数更新位置
        // 基于速度积分计算位置变化，考虑地球曲率影响
        LLAcalculate(dt, position, velocity);
        
        // 更新姿态（弧度）
        attitude.yaw = currentHeading;        // 偏航角（可能发生跳变）
        attitude.pitch = pitchAngle;          // 俯仰角基于筋斗运动
        attitude.roll = M_PI * params.rollDirection; // 保持180度滚转（倒飞状态）
    } else {
        // ========================================
        // 完成阶段：机动完成
        // ========================================
		currentPhase = COMPLETE;              // 设置完成机动标志位
		LLAcalculate(dt, position, velocity); // 保持当前状态飞行
	}
}

/**
 * @brief 重置半滚倒转模型
 * @details 将模型重置到初始状态，清除所有内部状态变量
 */
void SplitSModel::reset() {
    currentTime = 0.0;                        // 重置当前时间
    currentGForce = 1.0;                      // 重置当前过载
    currentPhase = ROLL_PHASE;                // 重置当前阶段为滚转阶段
    phaseTime = 0.0;                          // 重置阶段时间为0
}

// ============================================================================
// 工厂类实现
// ============================================================================

/**
 * @brief 创建机动模型实例
 * @param type 机动类型
 * @return 指向机动模型的智能指针
 * @throws std::invalid_argument 当机动类型未知时抛出异常
 * @details 根据机动类型创建相应的机动模型实例，使用工厂模式
 */
std::shared_ptr<KinematicManeuverModel> KinematicManeuverFactory::create(ManeuverType type) {
    switch (type) {
        case ManeuverType::LEVEL_TURN:
            return std::make_shared<LevelTurnModel>();  // 创建水平转弯模型
        case ManeuverType::LOOP:
            return std::make_shared<LoopModel>();       // 创建筋斗模型
        case ManeuverType::ROLL:
            return std::make_shared<RollModel>();       // 创建横滚模型
        case ManeuverType::SPLIT_S:
            return std::make_shared<SplitSModel>();     // 创建半滚倒转模型
        default:
            throw std::invalid_argument("Unknown maneuver type");  // 抛出异常
    }
}

/**
 * @brief 获取指定机动类型的默认参数
 * @param type 机动类型
 * @return 包含默认参数的KinematicManeuverParameters结构体
 * @details 为每种机动类型提供合理的默认参数配置
 * 
 * 默认参数说明：
 * - 水平转弯：3G过载，15秒持续时间，右转
 * - 筋斗：4G过载，20秒持续时间
 * - 横滚：1G过载，5秒持续时间，右滚
 * - 半滚倒转：2G过载，12秒持续时间，右滚
 */
KinematicManeuverParameters KinematicManeuverFactory::getDefaultParams(ManeuverType type) {
    KinematicManeuverParameters params;       // 创建默认参数结构体
    params.type = type;                       // 设置机动类型
    
    switch (type) {
        case ManeuverType::LEVEL_TURN:
            params.targetGForce = 3.0;        // 目标过载3G（中等过载）
            params.duration = 15.0;           // 持续时间15秒
            params.turnDirection = 1.0;       // 右转（1.0=右转，-1.0=左转）
            break;
        case ManeuverType::LOOP:
            params.targetGForce = 4.0;        // 目标过载4G（较高过载）
            params.duration = 20.0;           // 持续时间20秒
            break;
        case ManeuverType::ROLL:
            params.targetGForce = 1.0;        // 目标过载1G（平飞状态）
            params.duration = 5.0;            // 持续时间5秒（快速机动）
            params.rollDirection = 1.0;       // 右滚（1.0=右滚，-1.0=左滚）
            break;
        case ManeuverType::SPLIT_S:
            params.targetGForce = 2.0;        // 目标过载2G（中等过载）
            params.duration = 12.0;           // 持续时间12秒
            params.rollDirection = 1.0;       // 右滚（1.0=右滚，-1.0=左滚）
            break;
    }
    
    return params;
}
