#include <iostream>
#include <memory>
#include <iomanip>
#include <cmath>
#include "FighterJet.h"
#include "UnifiedManeuverSystem.h"

void printStatus(const FighterJet& fighter, double time, const std::string& phase) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "时间:" << std::setw(6) << time << "s | " 
              << "阶段:" << std::setw(8) << phase << " | "
              << "高度:" << std::setw(8) << fighter.position.altitude << "m | "
              << "速度:" << std::setw(6) << sqrt(fighter.velocity.north*fighter.velocity.north + 
                                                 fighter.velocity.up*fighter.velocity.up + 
                                                 fighter.velocity.east*fighter.velocity.east) << "m/s | "
              << "俯仰:" << std::setw(6) << fighter.attitude.getPitchDegrees() << "° | "
              << "滚转:" << std::setw(6) << fighter.attitude.getRollDegrees() << "° | "
              << "偏航:" << std::setw(6) << fighter.attitude.getYawDegrees() << "°" << std::endl;
}

int main() {
    std::cout << "=== 战斗机半滚倒转机动演示 ===" << std::endl;
    
    // 创建F-16战斗机
    FighterJet fighter(FighterType::F16_FALCON, 
                      {0.0, 0.0, 10000.0},    // 位置
                      {250.0, 0.0, 0.0},      // 速度
                      AttitudeAngles());       // 姿态
    
    std::cout << "战斗机: F-16 Falcon" << std::endl;
    std::cout << "最大过载: " << fighter.performance.maxGForce << "G" << std::endl;
    std::cout << std::endl;
    
    // 创建半滚倒转机动
    auto splitSManeuver = ManeuverFactory::create("split-s");
    if (!splitSManeuver) {
        std::cout << "错误：无法创建半滚倒转机动模型！" << std::endl;
        return -1;
    }
    
    // 配置参数
    UnifiedManeuverParameters params = ManeuverFactory::getDefaultParams("split-s");
    params.mode = UnifiedManeuverParameters::G_FORCE;
    params.targetGForce = 4.0;
    params.duration = 12.0;
    
    std::cout << "目标过载: " << params.targetGForce << "G" << std::endl;
    std::cout << "持续时间: " << params.duration << "秒" << std::endl;
    std::cout << std::endl;
    
    // 初始化机动
    fighter.setManeuverModel(splitSManeuver);
    fighter.initializeManeuver(params);
    
    // 仿真参数
    double dt = 0.1;
    double totalTime = 0.0;
    double maneuverStartTime = 2.0;
    
    std::cout << "时间(s) | 阶段     | 高度(m)  | 速度(m/s)| 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
    std::cout << "--------|----------|----------|----------|---------|---------|--------" << std::endl;
    
    // 仿真循环
    while (totalTime <= maneuverStartTime + params.duration + 2.0) {
        // 打印状态
        if (fmod(totalTime, 0.5) < dt) {
            std::string phase = (totalTime < maneuverStartTime) ? "准备" : 
                               (totalTime < maneuverStartTime + params.duration) ? "机动中" : "完成";
            printStatus(fighter, totalTime, phase);
        }
        
        // 执行机动
        if (totalTime >= maneuverStartTime && totalTime < maneuverStartTime + params.duration) {
            fighter.updateManeuver(dt);
        }
        
        // 更新飞机状态
        fighter.updateKinematics(dt);
        totalTime += dt;
    }
    
    // 结果分析
    std::cout << std::endl;
    std::cout << "=== 机动完成 ===" << std::endl;
    std::cout << "最终高度: " << fighter.position.altitude << "m" << std::endl;
    std::cout << "最终滚转角: " << fighter.attitude.getRollDegrees() << "°" << std::endl;
    std::cout << "最终俯仰角: " << fighter.attitude.getPitchDegrees() << "°" << std::endl;
    
    return 0;
}
