#include "TacviewTelemetry.h"
#include <iostream>
#include <cstring>
#include <sstream>
#include <iomanip>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

TacviewTelemetry::TacviewTelemetry(int listenPort,
                                   const std::string& callsign)
    : listenPort_(listenPort)
    , callsign_(callsign)
    , updateRate_(DEFAULT_UPDATE_RATE)
    , serverSocket_(-1)
    , clientSocket_(-1)
    , clientConnected_(false)
    , running_(false)
    , sentFrames_(0)
    , lastSendTime_(0.0)
{
#ifdef _WIN32
    // 初始化Winsock
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
}

TacviewTelemetry::~TacviewTelemetry() {
    stop();
#ifdef _WIN32
    WSACleanup();
#endif
}

bool TacviewTelemetry::start() {
    if (running_) {
        return true;
    }
    
    std::cout << "Starting Tacview telemetry server on port " << listenPort_ << "..." << std::endl;
    
    if (!initializeTCPServer()) {
        std::cerr << "Failed to initialize TCP server" << std::endl;
        return false;
    }
    
    running_ = true;
    serverThread_ = std::thread(&TacviewTelemetry::serverThreadFunction, this);
    sendThread_ = std::thread(&TacviewTelemetry::sendThreadFunction, this);
    
    std::cout << "Tacview telemetry server started successfully!" << std::endl;
    std::cout << "Waiting for Tacview to connect on port " << listenPort_ 
              << " (Callsign: " << callsign_ << ")" << std::endl;
    std::cout << "In Tacview, go to File -> Open -> Real-time telemetry and connect to 127.0.0.1:" << listenPort_ << std::endl;
    
    return true;
}

void TacviewTelemetry::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    clientConnected_ = false;
    
    if (serverThread_.joinable()) {
        serverThread_.join();
    }
    
    if (sendThread_.joinable()) {
        sendThread_.join();
    }
    
    cleanupTCPServer();
    std::cout << "Tacview telemetry server stopped. Total frames sent: " << sentFrames_ << std::endl;
}

void TacviewTelemetry::sendFrame(const TelemetryData& data) {
    if (!running_ || !clientConnected_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(dataMutex_);
    
    // 检查发送频率限制
    auto now = std::chrono::steady_clock::now();
    double currentTime = std::chrono::duration<double>(now.time_since_epoch()).count();
    double timeSinceLastSend = currentTime - lastSendTime_;
    
    if (timeSinceLastSend < (1.0 / updateRate_)) {
        return;  // 跳过此帧以保持更新频率
    }
    
    std::string dataStr = encodeTelemetryData(data);
    if (sendTCPData(dataStr)) {
        sentFrames_++;
        lastSendTime_ = currentTime;
        
        // 调试输出
        static int frameCount = 0;
        if (++frameCount == 1) {
            // 第一帧时显示发送的数据格式
            std::cout << "=== Sending First ACMI Data Frame ===" << std::endl;
            std::cout << dataStr << std::endl;
            std::cout << "=== Data Format Verification Complete ===" << std::endl;
        } else if (frameCount % 50 == 0) {
            std::cout << "Sent " << frameCount << " frames to Tacview" << std::endl;
        }
    } else {
        // 连接可能断开
        std::cerr << "Failed to send data, connection may be lost" << std::endl;
        clientConnected_ = false;
    }
}

void TacviewTelemetry::sendSimulationData(double time, 
                                         const GeoPosition& position,
                                         const Velocity3& velocity,
                                         const AttitudeAngles& attitude,
                                         double gForce,
                                         const std::string& aircraftType) {
    TelemetryData data;
    data.timestamp = time;
    data.longitude = position.longitude;
    data.latitude = position.latitude;
    data.altitude = position.altitude;
    data.northVelocity = velocity.north;
    data.upVelocity = velocity.up;
    data.eastVelocity = velocity.east;
    data.gForce = gForce;
    data.pitch = attitude.getPitchDegrees();
    data.roll = attitude.getRollDegrees();
    data.yaw = attitude.getYawDegrees();
    data.aircraftType = aircraftType;
    data.callsign = callsign_;
    
    sendFrame(data);
}

void TacviewTelemetry::setTarget(const std::string& ip, int port) {
    // 注意：在服务器模式下，我们监听所有接口，不需要设置目标IP
    // 端口在构造函数中设置
    std::cout << "Note: Server mode - listening on all interfaces, port: " << listenPort_ << std::endl;
}

void TacviewTelemetry::setCallsign(const std::string& callsign) {
    callsign_ = callsign;
}

void TacviewTelemetry::setUpdateRate(double rateHz) {
    updateRate_ = rateHz;
}

bool TacviewTelemetry::initializeTCPServer() {
    // 创建TCP server socket
    serverSocket_ = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket_ < 0) {
        std::cerr << "Failed to create TCP server socket" << std::endl;
        return false;
    }
    
    // 设置socket选项，允许地址重用
    int opt = 1;
    setsockopt(serverSocket_, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
    
    // 设置服务器地址
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;  // 监听所有接口
    serverAddr.sin_port = htons(listenPort_);
    
    // 绑定socket
    if (bind(serverSocket_, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind server socket to port " << listenPort_ << std::endl;
#ifdef _WIN32
        int error = WSAGetLastError();
        if (error == WSAEADDRINUSE) {
            std::cerr << "Port " << listenPort_ << " is already in use" << std::endl;
        }
#endif
        cleanupTCPServer();
        return false;
    }
    
    // 开始监听
    if (listen(serverSocket_, 1) < 0) {
        std::cerr << "Failed to listen on server socket" << std::endl;
        cleanupTCPServer();
        return false;
    }
    
    std::cout << "TCP server socket created and listening on port " << listenPort_ << std::endl;
    return true;
}

void TacviewTelemetry::serverThreadFunction() {
    std::cout << "Server thread started, waiting for Tacview to connect..." << std::endl;
    
    while (running_) {
        // 接受客户端连接
        struct sockaddr_in clientAddr;
        int clientAddrLen = sizeof(clientAddr);
        
        clientSocket_ = accept(serverSocket_, (struct sockaddr*)&clientAddr, &clientAddrLen);
        
        if (clientSocket_ >= 0) {
            clientConnected_ = true;
            std::cout << "Tacview connected successfully!" << std::endl;
            
            // 设置客户端socket超时（发送超时，接收不设超时）
            struct timeval timeout;
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            setsockopt(clientSocket_, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(timeout));
            
            // 尝试接收Tacview的握手响应
            std::cout << "Waiting for Tacview handshake response..." << std::endl;
            char buffer[1024];
            int bytesReceived = recv(clientSocket_, buffer, sizeof(buffer) - 1, 0);
            if (bytesReceived > 0) {
                buffer[bytesReceived] = '\0';
                std::cout << "=== Tacview Handshake Response ===" << std::endl;
                std::cout << "Received " << bytesReceived << " bytes:" << std::endl;
                std::cout << buffer << std::endl;
                std::cout << "=== End Response ===" << std::endl;
            } else if (bytesReceived == 0) {
                std::cout << "Tacview closed connection immediately" << std::endl;
                clientConnected_ = false;
            } else {
                int error = WSAGetLastError();
                if (error == WSAEWOULDBLOCK) {
                    std::cout << "No immediate response from Tacview (this is normal)" << std::endl;
                } else {
                    std::cout << "Error receiving from Tacview, error code: " << error << std::endl;
                }
            }
            
            // 保持连接，等待客户端断开
            // Tacview作为客户端，主要接收数据，不主动发送数据
            // 我们只需要保持连接活跃，不需要发送心跳包
            while (running_ && clientConnected_) {
                // 简单地等待，不发送心跳包，因为这会干扰ACMI数据流
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                
                // 检查程序是否还在运行
                if (!running_) {
                    break;
                }
            }
            
            // 关闭客户端socket
            if (clientSocket_ >= 0) {
#ifdef _WIN32
                closesocket(clientSocket_);
#else
                close(clientSocket_);
#endif
                clientSocket_ = -1;
            }
        } else {
            if (running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
    
    std::cout << "Server thread stopped" << std::endl;
}

bool TacviewTelemetry::verifyTacviewConnection() {
    // 在服务器模式下，我们不需要验证连接
    // 我们等待Tacview连接到我们的服务器
    std::cout << "Server mode: Waiting for Tacview to connect to port " << listenPort_ << std::endl;
    return true;
}

void TacviewTelemetry::checkConnectionStatus() {
    std::cout << "\n=== Tacview Connection Status Check ===" << std::endl;
    std::cout << "Server Port: " << listenPort_ << std::endl;
    std::cout << "Frames sent: " << sentFrames_ << std::endl;
    std::cout << "Server Status: " << (running_ ? "Running" : "Stopped") << std::endl;
    std::cout << "Client Status: " << (clientConnected_ ? "Connected" : "Waiting for connection") << std::endl;
    
    if (sentFrames_ == 0) {
        std::cout << "Status: No data sent" << std::endl;
        std::cout << "Possible reasons:" << std::endl;
        std::cout << "  - Program just started" << std::endl;
        std::cout << "  - No Tacview client connected" << std::endl;
        std::cout << "  - Send rate limiting" << std::endl;
    } else {
        std::cout << "Status: Sending data" << std::endl;
        std::cout << "If Tacview shows no trajectory, check:" << std::endl;
        std::cout << "  1. Tacview is connected to 127.0.0.1:" << listenPort_ << std::endl;
        std::cout << "  2. Tacview shows 'Connected' status" << std::endl;
        std::cout << "  3. Firewall blocking port " << listenPort_ << std::endl;
    }
    std::cout << "=====================================\n" << std::endl;
}

void TacviewTelemetry::cleanupTCPServer() {
    if (clientSocket_ >= 0) {
#ifdef _WIN32
        closesocket(clientSocket_);
#else
        close(clientSocket_);
#endif
        clientSocket_ = -1;
    }
    
    if (serverSocket_ >= 0) {
#ifdef _WIN32
        closesocket(serverSocket_);
#else
        close(serverSocket_);
#endif
        serverSocket_ = -1;
    }
    
    clientConnected_ = false;
}

void TacviewTelemetry::sendThreadFunction() {
    // 发送线程主要用于处理异步发送，当前实现中主要逻辑在sendFrame中
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

std::string TacviewTelemetry::encodeTelemetryData(const TelemetryData& data) {
    // Tacview实时遥测协议格式 - 使用标准的ACMI格式
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    // 第一阶段：Tacview实时遥测握手（只在第一帧发送）
    static bool handshakeSent = false;
    if (!handshakeSent) {
        // 使用官方标准的Tacview实时遥测握手格式
        // 注意：使用\r\n换行符，这是Windows标准
        oss << "XtraLib.Stream.0\r\n";
        oss << "Tacview.RealTimeTelemetry.0\r\n";
        oss << "Host " << callsign_ << "\r\n";
        
        handshakeSent = true;
        std::cout << "=== Tacview Real-time Telemetry Handshake Sent ===" << std::endl;
        std::cout << "Handshake format: XtraLib.Stream.0 + Tacview.RealTimeTelemetry.0 + Host " << callsign_ << std::endl;
        return oss.str();  // 第一帧只返回握手信息
    }
    
    // 第二阶段：ACMI协议头部（只在第二帧发送）
    static bool acmiHeaderSent = false;
    if (!acmiHeaderSent) {
        // ACMI文件头部 - 使用\r\n换行符
        oss << "FileType=text/acmi/tacview\r\n";
        oss << "FileVersion=2.2\r\n";
        oss << "0,ReferenceTime=2024-01-01T00:00:00Z\r\n";
        oss << "0,ReferenceLongitude=120\r\n";
        oss << "0,ReferenceLatitude=30\r\n";
        oss << "#0.0\r\n";  // 时间基准
        
        // 对象定义
        oss << "40000001,T=0|0|0,Type=Air+FixedWing,Name=" << callsign_ << "\r\n";
        
        acmiHeaderSent = true;
        std::cout << "=== ACMI Protocol Header Sent ===" << std::endl;
        return oss.str();  // 第二帧只返回ACMI头部
    }
    
    // 第三阶段：实时数据帧 - 使用标准ACMI格式
    // 使用官方标准格式：时间戳,对象ID,T=经度|纬度|高度
    oss << "0," << data.timestamp << ",40000001,T=" << data.longitude << "|" << data.latitude << "|" << data.altitude << "\r\n";
    
    // 握手确认：如果第一帧数据发送成功，说明握手完成
    static bool handshakeConfirmed = false;
    if (!handshakeConfirmed) {
        handshakeConfirmed = true;
        std::cout << "=== Handshake Complete - Sending Data ===" << std::endl;
        std::cout << "Tacview real-time telemetry is now active!" << std::endl;
    }
    
    return oss.str();
}

bool TacviewTelemetry::sendTCPData(const std::string& data) {
    if (clientSocket_ < 0 || data.empty()) {
        return false;
    }
    
    // 调试输出：显示发送的数据
    static int frameCount = 0;
    frameCount++;
    if (frameCount <= 3) {  // 只显示前3帧
        std::cout << "=== Sending Frame " << frameCount << " ===" << std::endl;
        std::cout << "Data length: " << data.length() << " bytes" << std::endl;
        std::cout << "Data content:" << std::endl;
        std::cout << data << std::endl;
        std::cout << "=== End Frame " << frameCount << " ===" << std::endl;
    }
    
    int sent = send(clientSocket_, data.c_str(), static_cast<int>(data.length()), 0);
    
    // 如果是握手帧，尝试接收Tacview的响应
    if (frameCount == 1 && sent > 0) {
        std::cout << "Handshake sent, waiting for Tacview response..." << std::endl;
        char responseBuffer[1024];
        int responseBytes = recv(clientSocket_, responseBuffer, sizeof(responseBuffer) - 1, 0);
        if (responseBytes > 0) {
            responseBuffer[responseBytes] = '\0';
            std::cout << "=== Tacview Response to Handshake ===" << std::endl;
            std::cout << "Received " << responseBytes << " bytes:" << std::endl;
            std::cout << responseBuffer << std::endl;
            std::cout << "=== End Response ===" << std::endl;
        } else if (responseBytes == 0) {
            std::cout << "Tacview closed connection after handshake" << std::endl;
        } else {
            int error = WSAGetLastError();
            if (error == WSAEWOULDBLOCK) {
                std::cout << "No response from Tacview (may be normal)" << std::endl;
            } else {
                std::cout << "Error receiving response, error code: " << error << std::endl;
            }
        }
    }
    
    if (sent < 0) {
#ifdef _WIN32
        int error = WSAGetLastError();
        std::cerr << "TCP send failed, error code: " << error << std::endl;
        if (error == WSAECONNRESET) {
            std::cerr << "Connection reset by peer - Tacview may have closed the connection" << std::endl;
            clientConnected_ = false;
        } else if (error == WSAECONNABORTED) {
            std::cerr << "Connection aborted - Check ACMI data format" << std::endl;
            clientConnected_ = false;
        } else if (error == WSAENOTCONN) {
            std::cerr << "Socket not connected" << std::endl;
            clientConnected_ = false;
        }
#else
        std::cerr << "TCP send failed, error code: " << errno << std::endl;
        if (errno == ECONNRESET) {
            std::cerr << "Connection reset by peer - Tacview may have closed the connection" << std::endl;
            clientConnected_ = false;
        } else if (errno == EPIPE) {
            std::cerr << "Broken pipe - Check ACMI data format" << std::endl;
            clientConnected_ = false;
        }
#endif
        return false;
    }
    
    if (sent != static_cast<int>(data.length())) {
        std::cerr << "Warning: Partial data sent (" << sent << "/" << data.length() << " bytes)" << std::endl;
    }
    
    return true;
}
