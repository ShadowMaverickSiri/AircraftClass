#include <iostream>
#include <iomanip>
#include <memory>
#include <fstream>
#include <algorithm>
#include "KinematicManeuverSystem.h"

int main() {
    std::cout << "=== 战斗机半滚倒转机动测试 ===" << std::endl;
    
    // 设置战斗机初始条件
    KinematicManeuverParameters params;
    params.initialPosition = {0.0, 0.0, 8000.0};    // 初始位置：经度0，纬度0，高度8000米
    params.initialVelocity = {300.0, 0.0, 0.0};    // 初始速度：300m/s向北（战斗机速度）
    params.startTime = 2.0;                         // 2秒后开始机动
    params.duration = 12.0;                         // 机动持续12秒
    params.type = ManeuverType::SPLIT_S;
    params.targetGForce = 2.5;                      // 目标过载2.5G
    params.rollDirection = 1.0;                     // 右滚转
    
    // 创建半滚倒转机动模型
    auto splitS = KinematicManeuverFactory::create(ManeuverType::SPLIT_S);
    splitS->initialize(params);
    
    // 仿真参数
    double dt = 0.1;        // 时间步长0.1秒（更精细的仿真）
    double totalTime = 0.0;
    double endTime = 20.0;  // 仿真20秒
    
    // 创建输出文件
    std::ofstream outFile("split_s_maneuver.csv");
    outFile << "时间(s),经度(°),纬度(°),高度(m),北速(m/s),天速(m/s),东速(m/s),过载(G),俯仰(°),滚转(°),偏航(°),阶段" << std::endl;
    
    std::cout << "初始条件：" << std::endl;
    std::cout << "  初始位置: 经度 " << params.initialPosition.longitude * 180.0 / M_PI 
              << "°, 纬度 " << params.initialPosition.latitude * 180.0 / M_PI 
              << "°, 高度 " << params.initialPosition.altitude << "m" << std::endl;
    std::cout << "  初始速度: 北向 " << params.initialVelocity.north 
              << "m/s, 天向 " << params.initialVelocity.up 
              << "m/s, 东向 " << params.initialVelocity.east << "m/s" << std::endl;
    std::cout << "  目标过载: " << params.targetGForce << "G" << std::endl;
    std::cout << "  机动持续时间: " << params.duration << "秒" << std::endl;
    std::cout << std::endl;
    
    // 表头
    std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°) | 阶段" << std::endl;
    std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------|------" << std::endl;
    
    GeoPosition position = params.initialPosition;
    Velocity3 velocity = params.initialVelocity;
    AttitudeAngles attitude;
    
    // 记录关键时间点
    double maxAltitude = params.initialPosition.altitude;
    double minAltitude = params.initialPosition.altitude;
    double maxGForce = 1.0;
    double minGForce = 1.0;
    
    while (totalTime <= endTime) {
        // 更新机动
        splitS->update(totalTime, dt, position, velocity, attitude);
        
        // 记录极值
        maxAltitude = std::max(maxAltitude, position.altitude);
        minAltitude = std::min(minAltitude, position.altitude);
        maxGForce = std::max(maxGForce, splitS->getCurrentGForce());
        minGForce = std::min(minGForce, splitS->getCurrentGForce());
        
        // 确定当前阶段
        std::string phase;
        double progress = (totalTime - params.startTime) / params.duration;
        if (totalTime < params.startTime) {
            phase = "准备";
        } else if (progress < 0.3) {
            phase = "滚转";
        } else if (progress < 1.0) {
            phase = "推杆";
        } else {
            phase = "完成";
        }
        
        // 写入CSV文件
        outFile << std::fixed << std::setprecision(3);
        outFile << totalTime << ","
                << position.longitude * 180.0 / M_PI << ","
                << position.latitude * 180.0 / M_PI << ","
                << position.altitude << ","
                << velocity.north << ","
                << velocity.up << ","
                << velocity.east << ","
                << splitS->getCurrentGForce() << ","
                << attitude.getPitchDegrees() << ","
                << attitude.getRollDegrees() << ","
                << attitude.getYawDegrees() << ","
                << phase << std::endl;
        
        // 打印状态（每0.5秒打印一次）
        if (fmod(totalTime, 0.5) < dt) {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << std::setw(8) << totalTime 
                      << std::setw(9) << position.longitude * 180.0 / M_PI
                      << std::setw(9) << position.latitude * 180.0 / M_PI
                      << std::setw(10) << position.altitude
                      << std::setw(11) << velocity.north
                      << std::setw(11) << velocity.up
                      << std::setw(11) << velocity.east
                      << std::setw(8) << splitS->getCurrentGForce()
                      << std::setw(9) << attitude.getPitchDegrees()
                      << std::setw(9) << attitude.getRollDegrees()
                      << std::setw(9) << attitude.getYawDegrees()
                      << std::setw(6) << phase << std::endl;
        }
        
        totalTime += dt;
    }
    
    outFile.close();
    
    // 输出机动分析结果
    std::cout << std::endl;
    std::cout << "=== 半滚倒转机动分析 ===" << std::endl;
    std::cout << "高度变化:" << std::endl;
    std::cout << "  最大高度: " << std::fixed << std::setprecision(1) << maxAltitude << "m" << std::endl;
    std::cout << "  最小高度: " << std::fixed << std::setprecision(1) << minAltitude << "m" << std::endl;
    std::cout << "  高度损失: " << std::fixed << std::setprecision(1) << (maxAltitude - minAltitude) << "m" << std::endl;
    
    std::cout << "过载变化:" << std::endl;
    std::cout << "  最大过载: " << std::fixed << std::setprecision(2) << maxGForce << "G" << std::endl;
    std::cout << "  最小过载: " << std::fixed << std::setprecision(2) << minGForce << "G" << std::endl;
    
    // 计算最终状态
    double finalSpeed = sqrt(velocity.north * velocity.north + 
                           velocity.up * velocity.up + 
                           velocity.east * velocity.east);
    double speedLoss = sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                           params.initialVelocity.east * params.initialVelocity.east) - finalSpeed;
    
    std::cout << "速度分析:" << std::endl;
    std::cout << "  初始速度: " << std::fixed << std::setprecision(1) 
              << sqrt(params.initialVelocity.north * params.initialVelocity.north + 
                     params.initialVelocity.east * params.initialVelocity.east) << "m/s" << std::endl;
    std::cout << "  最终速度: " << std::fixed << std::setprecision(1) << finalSpeed << "m/s" << std::endl;
    std::cout << "  速度损失: " << std::fixed << std::setprecision(1) << speedLoss << "m/s" << std::endl;
    
    // 计算航向变化
    double initialHeading = atan2(params.initialVelocity.east, params.initialVelocity.north) * 180.0 / M_PI;
    double finalHeading = attitude.getYawDegrees();
    double headingChange = finalHeading - initialHeading;
    
    // 标准化航向变化到-180到180度范围
    while (headingChange > 180.0) headingChange -= 360.0;
    while (headingChange < -180.0) headingChange += 360.0;
    
    std::cout << "航向变化:" << std::endl;
    std::cout << "  初始航向: " << std::fixed << std::setprecision(1) << initialHeading << "°" << std::endl;
    std::cout << "  最终航向: " << std::fixed << std::setprecision(1) << finalHeading << "°" << std::endl;
    std::cout << "  航向变化: " << std::fixed << std::setprecision(1) << headingChange << "°" << std::endl;
    
    std::cout << std::endl;
    std::cout << "数据已保存到 split_s_maneuver.csv 文件" << std::endl;
    std::cout << "=== 测试完成 ===" << std::endl;
    
    return 0;
}
