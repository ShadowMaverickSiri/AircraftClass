#include <iostream>
#include <memory>
#include "TacviewTelemetry.h"
#include <thread>
#include <chrono>
#include <cmath>

int main() {
    std::cout << "=== Tacview Circle Test ===" << std::endl;
    
    // 创建遥测对象
    auto telemetry = std::make_unique<TacviewTelemetry>(42674, "CIRCLE-01");
    
    // 启动遥测
    if (!telemetry->start()) {
        std::cerr << "Failed to start Tacview telemetry server" << std::endl;
        return 1;
    }
    
    std::cout << "Tacview telemetry server started!" << std::endl;
    std::cout << "Please connect to 127.0.0.1:42674 in Tacview" << std::endl;
    
    // 等待连接
    std::cout << "Waiting for Tacview connection..." << std::endl;
    int waitCount = 0;
    while (!telemetry->isRunning() && waitCount < 30) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        waitCount++;
        std::cout << "Waiting... (" << waitCount << "/30)" << std::endl;
    }
    
    if (!telemetry->isRunning()) {
        std::cerr << "Timeout: Tacview not connected" << std::endl;
        telemetry->stop();
        return 1;
    }
    
    std::cout << "Tacview connected! Starting circle flight..." << std::endl;
    
    // 发送圆形飞行轨迹
    double time = 0.0;
    double dt = 0.1;  // 100ms更新间隔
    double radius = 0.01;  // 约1km半径
    double centerLon = 120.0;
    double centerLat = 30.0;
    double altitude = 10000.0;
    
    for (int i = 0; i < 100; ++i) {
        // 计算圆形轨迹
        double angle = time * 0.5;  // 角速度
        
        double longitude = centerLon + radius * cos(angle);
        double latitude = centerLat + radius * sin(angle);
        
        // 计算速度（切线方向）
        double speed = 200.0;  // 200 m/s
        double northVel = -speed * sin(angle);
        double eastVel = speed * cos(angle);
        double upVel = 0.0;
        
        // 计算姿态
        double pitch = 0.0;
        double roll = 0.0;
        double yaw = angle * 180.0 / 3.1415926;  // 转换为度
        
        // 发送数据
        AttitudeAngles attitude;
        attitude.setPitchDegrees(pitch);
        attitude.setRollDegrees(roll);
        attitude.setYawDegrees(yaw);
        
        telemetry->sendSimulationData(time,
            { longitude, latitude, altitude },
            { northVel, upVel, eastVel },
            attitude,
            1.0,  // G力
            "F-22");
        
        // 每10帧显示一次状态
        if (i % 10 == 0) {
            std::cout << "Time: " << std::fixed << std::setprecision(1) << time
                << "s, Position: (" << longitude << ", " << latitude << "), "
                << "Altitude: " << altitude << "m, Yaw: " << yaw << "deg" << std::endl;
        }
        
        time += dt;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    std::cout << "Circle test completed! Total frames sent: " << telemetry->getSentFrames() << std::endl;
    
    // 停止遥测
    telemetry->stop();
    
    return 0;
}
