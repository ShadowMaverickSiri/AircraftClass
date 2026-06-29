#include "TacviewTelemetry.h"
#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>

// ============================================================================
// TacviewTelemetry 类实现
// ============================================================================

// 构造函数
// 初始化 Winsock 库，为网络通信做准备
TacviewTelemetry::TacviewTelemetry()
    : listenSock_(INVALID_SOCKET), running_(false)
{
    // 初始化 Windows Sockets 库
    // 参数：MAKEWORD(2, 2) 请求版本 2.2
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
    }
}

// 析构函数
// 自动停止服务器并清理 Winsock 资源
TacviewTelemetry::~TacviewTelemetry()
{
    stop();              // 停止服务器，断开所有客户端
    WSACleanup();        // 清理 Winsock 资源
}

// 启动 TCP 服务器
// 参数：port - 监听端口号，默认为 42674（Tacview 标准端口）
// 返回值：成功返回 true，失败返回 false
bool TacviewTelemetry::start(unsigned short port)
{
    // 如果服务器已经在运行，直接返回成功
    if (running_) return true;

    // 创建 TCP 套接字
    // AF_INET: IPv4 地址族
    // SOCK_STREAM: 面向连接的套接字（TCP）
    // IPPROTO_TCP: TCP 协议
    listenSock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock_ == INVALID_SOCKET) {
        std::cerr << "socket() failed\n";
        return false;
    }

    // 设置套接字选项：允许地址重用
    // 这样可以在服务器重启后立即绑定同一端口
    char opt = 1;
    setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 绑定套接字到指定端口
    sockaddr_in addr{};
    addr.sin_family = AF_INET;           // IPv4
    addr.sin_addr.s_addr = INADDR_ANY;   // 监听所有网络接口
    addr.sin_port = htons(port);         // 绑定端口（转换为网络字节序）

    if (bind(listenSock_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "bind() failed, errno=" << WSAGetLastError() << "\n";
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
        return false;
    }

    // 开始监听，最大挂起连接队列为 4
    if (listen(listenSock_, 4) == SOCKET_ERROR) {
        std::cerr << "listen() failed\n";
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
        return false;
    }

    // 设置运行标志并启动接受连接的线程
    running_ = true;
    acceptThread_ = std::thread(&TacviewTelemetry::acceptLoop, this);
    std::cout << "[Tacview] Listening on port " << port << "...\n";
    return true;
}

// 停止服务器并断开所有客户端连接
void TacviewTelemetry::stop()
{
    // 如果服务器已经停止，直接返回
    if (!running_) return;

    // 设置运行标志为 false，通知 acceptLoop 线程退出
    running_ = false;

    // 关闭监听套接字
    // 这会解除 accept() 的阻塞，使线程能够退出
    if (listenSock_ != INVALID_SOCKET) {
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
    }

    // 等待接受线程结束
    if (acceptThread_.joinable()) acceptThread_.join();

    // 关闭所有客户端套接字
    std::lock_guard<std::mutex> lk(clientsMutex_);
    for (SOCKET s : clients_) {
        if (s != INVALID_SOCKET) {
            closesocket(s);
        }
    }
    clients_.clear();
    std::cout << "[Tacview] Telemetry server stopped\n";

    // 关闭日志文件
    if (logFile_.is_open()) {
        logFile_.close();
    }
}

// 启用日志记录功能
// 参数：logFile - 日志文件路径，默认为 "telemetry_log.txt"
void TacviewTelemetry::enableLogging(const std::string& logFile)
{
    std::lock_guard<std::mutex> lk(logMutex_);
    // 如果已有日志文件打开，先关闭
    if (logFile_.is_open()) {
        logFile_.close();
    }
    // 打开新的日志文件（覆盖模式）
    logFile_.open(logFile, std::ios::out | std::ios::trunc);
    if (logFile_.is_open()) {
        std::cout << "[Tacview] Logging enabled: " << logFile << "\n";
    } else {
        std::cerr << "[Tacview] Failed to open log file: " << logFile << "\n";
    }
}

// 接受客户端连接的循环函数（在独立线程中运行）
void TacviewTelemetry::acceptLoop()
{
    while (running_) {
        // 等待客户端连接
        sockaddr_in clientAddr{};
        int addrLen = sizeof(clientAddr);
        SOCKET client = accept(listenSock_, (sockaddr*)&clientAddr, &addrLen);

        if (client == INVALID_SOCKET) {
            // 如果服务器已停止，正常退出
            if (!running_) break;
            // 否则打印错误并继续
            std::cerr << "[Tacview] accept() failed, errno=" << WSAGetLastError() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // 获取客户端 IP 地址并打印连接信息
        char ip[INET_ADDRSTRLEN] = "n/a";
        inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip));
        std::cout << "[Tacview] Client connected: " << ip << ":"
                  << ntohs(clientAddr.sin_port) << "\n";

        // 向新客户端发送握手协议
        sendHandshake(client);

        // 将客户端加入列表（需要加锁保护）
        {
            std::lock_guard<std::mutex> lk(clientsMutex_);
            clients_.push_back(client);
        }
    }
}

// 向客户端发送 Tacview 握手协议
// 参数：client - 客户端套接字
void TacviewTelemetry::sendHandshake(SOCKET client)
{
    // Tacview 实时遥测协议握手格式
    // 必须严格按照以下格式发送，最后需要一个空字符终止符
    //
    // 格式：
    //   XtraLib.Stream.0\n
    //   Tacview.RealTimeTelemetry.0\n
    //   <服务器名称>\n
    //   \0（空字符终止符）

    std::string handshake =
        "XtraLib.Stream.0\n"
        "Tacview.RealTimeTelemetry.0\n"
        "AircraftClass Server\n";

    // 发送握手内容（不含终止符）
    if (!sendAll(client, handshake.c_str(), (int)handshake.size())) {
        std::cerr << "[Tacview] Failed to send handshake\n";
        return;
    }

    // 发送终止空字符（Tacview 协议必需）
    char terminator = '\0';
    if (!sendAll(client, &terminator, 1)) {
        std::cerr << "[Tacview] Failed to send handshake terminator\n";
        return;
    }

    std::cout << "[Tacview] Handshake sent successfully\n";
}

// 可靠发送函数，确保所有数据都发送完成
// 参数：
//   s   - 目标套接字
//   buf - 要发送的数据缓冲区
//   len - 要发送的数据长度
// 返回值：成功返回 true，失败返回 false
bool TacviewTelemetry::sendAll(SOCKET s, const char* buf, int len)
{
    int sentTotal = 0;
    // 循环发送，直到所有数据发送完毕或发生错误
    while (sentTotal < len) {
        int n = send(s, buf + sentTotal, len - sentTotal, 0);
        if (n == SOCKET_ERROR) {
            std::cerr << "[Tacview] send() failed to client, errno="
                      << WSAGetLastError() << "\n";
            return false;
        }
        if (n == 0) return false;  // 连接已关闭
        sentTotal += n;
    }
    return true;
}

// 移除客户端（调用前必须持有 clientsMutex_ 锁）
// 参数：idx - 客户端在列表中的索引
void TacviewTelemetry::removeClientLocked(size_t idx)
{
    if (idx >= clients_.size()) return;
    SOCKET s = clients_[idx];
    if (s != INVALID_SOCKET) closesocket(s);
    clients_.erase(clients_.begin() + idx);
}

// 向所有连接的客户端广播 ACMI 数据行
// 参数：line - 要发送的数据行（不包含换行符，函数会自动添加）
void TacviewTelemetry::broadcastLine(const std::string& line)
{
    std::lock_guard<std::mutex> lk(clientsMutex_);

    // 如果没有客户端连接，直接返回
    if (clients_.empty()) return;

    // 添加换行符
    std::string msg = line + "\n";

    // 记录到日志文件（如果启用）
    {
        std::lock_guard<std::mutex> logLock(logMutex_);
        if (logFile_.is_open()) {
            logFile_ << msg << std::flush;
        }
    }

    // 向所有客户端发送数据
    for (size_t i = 0; i < clients_.size(); ++i) {
        SOCKET s = clients_[i];
        if (!sendAll(s, msg.c_str(), (int)msg.size())) {
            // 发送失败，移除断开的客户端
            std::cerr << "[Tacview] Dropping disconnected client\n";
            removeClientLocked(i);
            --i;  // 调整索引，因为列表已变化
        }
    }
}
