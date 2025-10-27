#pragma once
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include"AircraftModelLibrary.h"

// 前向声明
struct GeoPosition;
struct Velocity3;
struct AttitudeAngles;

/**
 * Tacview实时遥测数据发送器
 * 支持通过UDP协议向Tacview发送实时飞行数据
 */
class TacviewTelemetry {
public:
    struct TelemetryData {
        double timestamp;          // 时间戳(秒)
        double longitude;          // 经度(度)
        double latitude;           // 纬度(度)
        double altitude;           // 高度(米)
        double northVelocity;      // 北向速度(m/s)
        double upVelocity;         // 天向速度(m/s)
        double eastVelocity;       // 东向速度(m/s)
        double gForce;             // 过载(G)
        double pitch;              // 俯仰角(度)
        double roll;               // 滚转角(度)
        double yaw;                // 偏航角(度)
        std::string aircraftType;  // 飞机类型
        std::string callsign;      // 呼号
    };

    // 构造函数
    TacviewTelemetry(int listenPort = 42674,
                     const std::string& callsign = "F-22-01");
    
    // 析构函数
    ~TacviewTelemetry();

    // 启动/停止遥测
    bool start();
    void stop();
    bool isRunning() const { return running_ && clientConnected_; }
    bool isServerRunning() const { return running_; }

    // 发送单帧数据
    void sendFrame(const TelemetryData& data);
    
    // 便捷方法：从仿真数据结构发送
    void sendSimulationData(double time, 
                           const GeoPosition& position,
                           const Velocity3& velocity,
                           const AttitudeAngles& attitude,
                           double gForce,
                           const std::string& aircraftType = "F-22");

    // 配置方法
    void setTarget(const std::string& ip, int port);
    void setCallsign(const std::string& callsign);
    void setUpdateRate(double rateHz);  // 设置更新频率

    // 状态查询
    size_t getSentFrames() const { return sentFrames_; }
    double getLastSendTime() const { return lastSendTime_; }
    
    // 连接状态检查
    void checkConnectionStatus();
    
    // 真正的连接验证
    bool verifyTacviewConnection();

private:
    // 网络相关
    int listenPort_;
    int serverSocket_;           // TCP server socket
    int clientSocket_;           // TCP client socket (Tacview)
    
    // 数据相关
    std::string callsign_;
    double updateRate_;           // 更新频率(Hz)
    double lastSendTime_;
    
    // 连接状态
    std::atomic<bool> clientConnected_;
    std::atomic<bool> running_;
    std::thread serverThread_;
    std::thread sendThread_;
    std::mutex dataMutex_;
    
    // 统计信息
    std::atomic<size_t> sentFrames_;
    
    // 内部方法
    bool initializeTCPServer();
    void serverThreadFunction();
    void sendThreadFunction();
    void cleanupTCPServer();
    std::string encodeTelemetryData(const TelemetryData& data);
    bool sendTCPData(const std::string& data);
    
    // Tacview协议相关
    static constexpr int TACVIEW_DEFAULT_PORT = 42674;
    static constexpr double DEFAULT_UPDATE_RATE = 10.0;  // 10Hz
    static constexpr size_t MAX_PACKET_SIZE = 1024;
};

