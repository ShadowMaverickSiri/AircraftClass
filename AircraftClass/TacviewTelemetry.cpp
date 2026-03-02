#include "TacviewTelemetry.h"
#include <iostream>
#include <chrono>
#include <sstream>
#include <algorithm>

TacviewTelemetry::TacviewTelemetry()
    : listenSock_(INVALID_SOCKET), running_(false)
{
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
    }
}

TacviewTelemetry::~TacviewTelemetry()
{
    stop();
    WSACleanup();
}

bool TacviewTelemetry::start(unsigned short port)
{
    if (running_) return true;

    listenSock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock_ == INVALID_SOCKET) {
        std::cerr << "socket() failed\n";
        return false;
    }

    // 允许地址重用
    char opt = 1;
    setsockopt(listenSock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(listenSock_, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        std::cerr << "bind() failed, errno=" << WSAGetLastError() << "\n";
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
        return false;
    }

    if (listen(listenSock_, 4) == SOCKET_ERROR) {
        std::cerr << "listen() failed\n";
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
        return false;
    }

    running_ = true;
    acceptThread_ = std::thread(&TacviewTelemetry::acceptLoop, this);
    std::cout << "[Tacview] Listening on port " << port << "...\n";
    return true;
}

void TacviewTelemetry::stop()
{
    if (!running_) return;
    running_ = false;

    // 关闭监听套接字，解除 accept() 阻塞
    if (listenSock_ != INVALID_SOCKET) {
        closesocket(listenSock_);
        listenSock_ = INVALID_SOCKET;
    }

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

void TacviewTelemetry::enableLogging(const std::string& logFile)
{
    std::lock_guard<std::mutex> lk(logMutex_);
    if (logFile_.is_open()) {
        logFile_.close();
    }
    logFile_.open(logFile, std::ios::out | std::ios::trunc);
    if (logFile_.is_open()) {
        std::cout << "[Tacview] Logging enabled: " << logFile << "\n";
    } else {
        std::cerr << "[Tacview] Failed to open log file: " << logFile << "\n";
    }
}

void TacviewTelemetry::acceptLoop()
{
    while (running_) {
        sockaddr_in clientAddr{};
        int addrLen = sizeof(clientAddr);
        SOCKET client = accept(listenSock_, (sockaddr*)&clientAddr, &addrLen);
        if (client == INVALID_SOCKET) {
            if (!running_) break;
            std::cerr << "[Tacview] accept() failed, errno=" << WSAGetLastError() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        char ip[INET_ADDRSTRLEN] = "n/a";
        inet_ntop(AF_INET, &clientAddr.sin_addr, ip, sizeof(ip));
        std::cout << "[Tacview] Client connected: " << ip << ":" << ntohs(clientAddr.sin_port) << "\n";

        // 向新客户端发送握手协议
        sendHandshake(client);

        {
            std::lock_guard<std::mutex> lk(clientsMutex_);
            clients_.push_back(client);
        }
    }
}

void TacviewTelemetry::sendHandshake(SOCKET client)
{
    // Tacview 实时遥测协议握手
    // 格式: XtraLib.Stream.0\nTacview.RealTimeTelemetry.0\n<Host Username>\n\0

    std::string handshake =
        "XtraLib.Stream.0\n"
        "Tacview.RealTimeTelemetry.0\n"
        "AircraftClass Server\n";

    // 发送所有行
    if (!sendAll(client, handshake.c_str(), (int)handshake.size())) {
        std::cerr << "[Tacview] Failed to send handshake\n";
        return;
    }

    // 发送终止空字符
    char terminator = '\0';
    if (!sendAll(client, &terminator, 1)) {
        std::cerr << "[Tacview] Failed to send handshake terminator\n";
        return;
    }

    std::cout << "[Tacview] Handshake sent successfully\n";
}

bool TacviewTelemetry::sendAll(SOCKET s, const char* buf, int len)
{
    int sentTotal = 0;
    while (sentTotal < len) {
        int n = send(s, buf + sentTotal, len - sentTotal, 0);
        if (n == SOCKET_ERROR) {
            std::cerr << "[Tacview] send() failed to client, errno=" << WSAGetLastError() << "\n";
            return false;
        }
        if (n == 0) return false;
        sentTotal += n;
    }
    return true;
}

void TacviewTelemetry::removeClientLocked(size_t idx)
{
    if (idx >= clients_.size()) return;
    SOCKET s = clients_[idx];
    if (s != INVALID_SOCKET) closesocket(s);
    clients_.erase(clients_.begin() + idx);
}

void TacviewTelemetry::broadcastLine(const std::string& line)
{
    std::lock_guard<std::mutex> lk(clientsMutex_);
    if (clients_.empty()) return;

    std::string msg = line + "\n";

    // 记录到日志
    {
        std::lock_guard<std::mutex> logLock(logMutex_);
        if (logFile_.is_open()) {
            logFile_ << msg << std::flush;
        }
    }

    // 向所有客户端发送
    for (size_t i = 0; i < clients_.size(); ++i) {
        SOCKET s = clients_[i];
        if (!sendAll(s, msg.c_str(), (int)msg.size())) {
            std::cerr << "[Tacview] Dropping disconnected client\n";
            removeClientLocked(i);
            --i;
        }
    }
}
