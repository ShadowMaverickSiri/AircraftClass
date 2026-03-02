#pragma once

#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <atomic>
#include <fstream>

#pragma comment(lib, "Ws2_32.lib")

// ============================================================================
// Tacview 实时遥测 TCP 服务器
// 用于向 Tacview 软件实时发送飞行数据
// ============================================================================
class TacviewTelemetry {
public:
    TacviewTelemetry();
    ~TacviewTelemetry();

    // 启动服务器，监听端口（默认 42674）
    bool start(unsigned short port = 42674);

    // 停止服务器并断开所有客户端
    void stop();

    // 向客户端广播 ACMI 数据行
    void broadcastLine(const std::string& line);

    // 状态查询
    bool isRunning() const { return running_; }
    bool hasClients() const { return !clients_.empty(); }

    // 启用/禁用日志记录
    void enableLogging(const std::string& logFile = "telemetry_log.txt");

private:
    void acceptLoop();
    bool sendAll(SOCKET s, const char* buf, int len);
    void removeClientLocked(size_t idx);
    void sendHandshake(SOCKET client);

private:
    SOCKET listenSock_;
    std::vector<SOCKET> clients_;
    std::mutex clientsMutex_;
    std::thread acceptThread_;
    std::atomic_bool running_;
    std::ofstream logFile_;
    std::mutex logMutex_;
};
