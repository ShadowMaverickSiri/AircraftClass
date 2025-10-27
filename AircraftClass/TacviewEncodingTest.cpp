#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include "TacviewTelemetry.h"

int main() {
    std::cout << "=== Tacview Encoding Test ===" << std::endl;
    
    // 测试不同的编码格式
    std::cout << "Testing different encoding formats..." << std::endl;
    
    // 1. 测试标准ASCII格式
    std::string testData1 = "FileType=text/acmi/tacview\nFileVersion=2.2\n";
    std::cout << "ASCII format test:" << std::endl;
    std::cout << "Length: " << testData1.length() << " bytes" << std::endl;
    std::cout << "Content: " << testData1 << std::endl;
    
    // 2. 测试UTF-8格式（如果可能）
    std::string testData2 = "FileType=text/acmi/tacview\r\nFileVersion=2.2\r\n";
    std::cout << "Windows format test:" << std::endl;
    std::cout << "Length: " << testData2.length() << " bytes" << std::endl;
    std::cout << "Content: " << testData2 << std::endl;
    
    // 3. 创建最小化的Tacview测试
    auto telemetry = std::make_unique<TacviewTelemetry>(42674, "ENCODING-TEST");
    
    if (!telemetry->start()) {
        std::cerr << "Failed to start Tacview telemetry server" << std::endl;
        return 1;
    }
    
    std::cout << "Tacview telemetry server started!" << std::endl;
    std::cout << "Please connect to 127.0.0.1:42674 in Tacview" << std::endl;
    
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
    
    std::cout << "Tacview connected! Sending encoding test data..." << std::endl;
    
    // 发送最简单的测试数据
    for (int i = 0; i < 5; ++i) {
        double time = i * 1.0;
        double longitude = 120.0 + i * 0.001;
        double latitude = 30.0;
        double altitude = 10000.0;
        
        AttitudeAngles attitude;
        attitude.setPitchDegrees(0.0);
        attitude.setRollDegrees(0.0);
        attitude.setYawDegrees(0.0);
        
        telemetry->sendSimulationData(time,
            { longitude, latitude, altitude },
            { 200.0, 0.0, 0.0 },
            attitude,
            1.0,
            "F-16");
        
        std::cout << "Sent frame " << (i + 1) << " at time " << time 
                  << "s, position (" << longitude << ", " << latitude << ")" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 2 second delay
    }
    
    std::cout << "Encoding test completed! Total frames sent: " << telemetry->getSentFrames() << std::endl;
    
    telemetry->stop();
    
    return 0;
}
