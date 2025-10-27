#include <iostream>
#include <iomanip>
#include <memory>
#include "KinematicManeuverSystem.h"

int main() {
    std::cout << "=== 运动学机动系统测试 ===" << std::endl;
    
    // 设置初始条件
    KinematicManeuverParameters params;
    params.initialPosition = {0.0, 0.0, 10000.0};  // 初始位置：经度0，纬度0，高度10000米
    params.initialVelocity = {250.0, 0.0, 0.0};    // 初始速度：250m/s向北
    params.startTime = 5.0;                         // 5秒后开始机动
    params.duration = 15.0;                         // 机动持续15秒
    
    // 测试水平转弯
    std::cout << "\n--- 水平转弯测试 ---" << std::endl;
    auto levelTurn = KinematicManeuverFactory::create(ManeuverType::LEVEL_TURN);
    params.type = ManeuverType::LEVEL_TURN;
    params.targetGForce = 3.0;
    params.turnDirection = 1.0;  // 右转
    levelTurn->initialize(params);
    
    // 仿真参数
    double dt = 0.5;  // 时间步长0.5秒
    double totalTime = 0.0;
    double endTime = 25.0;  // 仿真25秒
    
    std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
    std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------" << std::endl;
    
    GeoPosition position = params.initialPosition;
    Velocity3 velocity = params.initialVelocity;
    AttitudeAngles attitude;
    
    while (totalTime <= endTime) {
        // 更新机动
        levelTurn->update(totalTime, dt, position, velocity, attitude);
        
        // 打印状态（每2秒打印一次）
        if (fmod(totalTime, 2.0) < dt) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << std::setw(8) << totalTime 
                      << std::setw(9) << position.longitude * 180.0 / M_PI
                      << std::setw(9) << position.latitude * 180.0 / M_PI
                      << std::setw(10) << position.altitude
                      << std::setw(11) << velocity.north
                      << std::setw(11) << velocity.up
                      << std::setw(11) << velocity.east
                      << std::setw(8) << levelTurn->getCurrentGForce()
                      << std::setw(9) << attitude.getPitchDegrees()
                      << std::setw(9) << attitude.getRollDegrees()
                      << std::setw(9) << attitude.getYawDegrees() << std::endl;
        }
        
        totalTime += dt;
    }
    
    // 测试筋斗
    std::cout << "\n--- 筋斗测试 ---" << std::endl;
    auto loop = KinematicManeuverFactory::create(ManeuverType::LOOP);
    params.type = ManeuverType::LOOP;
    params.targetGForce = 4.0;
    params.duration = 20.0;
    loop->initialize(params);
    
    // 重置状态
    position = params.initialPosition;
    velocity = params.initialVelocity;
    totalTime = 0.0;
    endTime = 30.0;
    
    std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
    std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------" << std::endl;
    
    while (totalTime <= endTime) {
        loop->update(totalTime, dt, position, velocity, attitude);
        
        if (fmod(totalTime, 2.0) < dt) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << std::setw(8) << totalTime 
                      << std::setw(9) << position.longitude * 180.0 / M_PI
                      << std::setw(9) << position.latitude * 180.0 / M_PI
                      << std::setw(10) << position.altitude
                      << std::setw(11) << velocity.north
                      << std::setw(11) << velocity.up
                      << std::setw(11) << velocity.east
                      << std::setw(8) << loop->getCurrentGForce()
                      << std::setw(9) << attitude.getPitchDegrees()
                      << std::setw(9) << attitude.getRollDegrees()
                      << std::setw(9) << attitude.getYawDegrees() << std::endl;
        }
        
        totalTime += dt;
    }
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    return 0;
}
