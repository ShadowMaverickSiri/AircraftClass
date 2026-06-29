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
// ============================================================================
//
// 功能说明：
// ---------
// 这个类实现了 Tacview 软件的实时遥测 TCP 服务器功能，用于向 Tacview 客户端
// 实时发送飞行仿真数据。Tacview 是一款通用的飞行数据分析软件，支持实时
// 接收和显示飞行轨迹、姿态等信息。
//
// 通信协议：
// ---------
// 1. TCP 服务器监听端口 42674（Tacview 默认端口）
// 2. 客户端连接后，服务器发送握手协议
// 3. 握手成功后，服务器发送 ACMI 格式的数据流
//
// 握手协议格式：
// ------------
// XtraLib.Stream.0\n
// Tacview.RealTimeTelemetry.0\n
// <服务器名称>\n
// \0（空字符终止符）
//
// ACMI 数据格式：
// --------------
// 数据以行为单位，每行以 \n 结尾。主要包含：
// - 文件头：FileType, FileVersion 等元数据
// - 对象定义：对象ID,Type=xxx,Name=xxx,Color=xxx,T=位置
// - 状态更新：#时间戳，对象ID,T=位置|姿态信息
//
// 线程安全：
// --------
// - 使用独立的 acceptThread_ 接受客户端连接
// - 使用 clientsMutex_ 保护客户端列表
// - broadcastLine() 方法内部已加锁，外部调用无需同步
//
// 使用示例：
// --------
// TacviewTelemetry telemetry;
// telemetry.start(42674);
// telemetry.broadcastLine("FileType=text/acmi/tacview\nFileVersion=2.2");
// telemetry.broadcastLine("#0.1");
// telemetry.broadcastLine("3EC,T=116.0|39.0|9000.0|0.0|0.0|0.0");
// telemetry.stop();
//
// ============================================================================

class TacviewTelemetry {
public:
    // 构造函数
    // 初始化 Winsock 库，创建监听套接字
    TacviewTelemetry();

    // 析构函数
    // 自动停止服务器并清理 Winsock 资源
    ~TacviewTelemetry();

    // 启动 TCP 服务器
    //
    // 参数：
    //   port - 监听端口号，默认为 42674（Tacview 标准端口）
    //
    // 返回值：
    //   成功返回 true，失败返回 false
    //
    // 注意：
    //   - 如果服务器已经运行，直接返回 true
    //   - 此方法会创建一个独立的线程接受客户端连接
    //   - 调用后需等待 hasClients() 返回 true 再发送数据
    bool start(unsigned short port = 42674);

    // 停止服务器并断开所有客户端连接
    //
    // 功能：
    //   - 设置运行标志为 false
    //   - 关闭监听套接字，解除 accept() 阻塞
    //   - 等待接受线程结束
    //   - 关闭所有客户端套接字
    //   - 关闭日志文件
    void stop();

    // 向所有连接的客户端广播 ACMI 数据行
    //
    // 参数：
    //   line - 要发送的数据行（不包含换行符，函数会自动添加）
    //
    // 功能：
    //   - 在 line 末尾添加 \n 换行符
    //   - 将数据写入日志文件（如果启用）
    //   - 向所有客户端发送数据
    //   - 如果发送失败，自动移除断开的客户端
    //
    // 线程安全：内部已加锁，外部调用无需同步
    void broadcastLine(const std::string& line);

    // 查询服务器是否正在运行
    //
    // 返回值：true 表示服务器正在运行，false 表示已停止
    bool isRunning() const { return running_; }

    // 查询是否有客户端连接
    //
    // 返回值：true 表示至少有一个客户端连接，false 表示无连接
    //
    // 用途：在发送数据前检查，确保有客户端接收
    bool hasClients() const { return !clients_.empty(); }

    // 启用日志记录功能
    //
    // 参数：
    //   logFile - 日志文件路径，默认为 "telemetry_log.txt"
    //
    // 功能：
    //   - 打开指定的日志文件
    //   - 所有通过 broadcastLine() 发送的数据都会写入日志
    //   - 如果已有日志文件打开，会先关闭它
    void enableLogging(const std::string& logFile = "telemetry_log.txt");

private:
    // 接受客户端连接的循环函数
    //
    // 功能：
    //   - 在独立线程中运行
    //   - 循环调用 accept() 接受客户端连接
    //   - 对每个新客户端发送握手协议
    //   - 将客户端加入客户端列表
    //
    // 注意：此方法在 acceptThread_ 线程中运行，不要直接调用
    void acceptLoop();

    // 可靠发送函数，确保所有数据都发送完成
    //
    // 参数：
    //   s    - 目标套接字
    //   buf  - 要发送的数据缓冲区
    //   len  - 要发送的数据长度
    //
    // 返回值：成功返回 true，失败返回 false
    //
    // 功能：循环调用 send() 直到所有数据发送完毕或发生错误
    bool sendAll(SOCKET s, const char* buf, int len);

    // 移除客户端（调用前必须持有 clientsMutex_ 锁）
    //
    // 参数：
    //   idx - 客户端在列表中的索引
    //
    // 功能：关闭指定客户端的套接字并从列表中移除
    //
    // 注意：调用此函数前必须先获取 clientsMutex_ 锁
    void removeClientLocked(size_t idx);

    // 向客户端发送 Tacview 握手协议
    //
    // 参数：
    //   client - 客户端套接字
    //
    // 握手协议格式：
    //   XtraLib.Stream.0\n
    //   Tacview.RealTimeTelemetry.0\n
    //   AircraftClass Server\n
    //   \0（空字符终止符）
    //
    // 注意：最后的 \0 空字符是必需的，Tacview 协议要求
    void sendHandshake(SOCKET client);

private:
    // 监听套接字，用于接受客户端连接
    SOCKET listenSock_;

    // 客户端套接字列表，存储所有已连接的客户端
    std::vector<SOCKET> clients_;

    // 保护客户端列表的互斥锁
    std::mutex clientsMutex_;

    // 接受连接的独立线程
    std::thread acceptThread_;

    // 服务器运行标志，原子操作保证线程安全
    std::atomic_bool running_;

    // 日志文件输出流
    std::ofstream logFile_;

    // 保护日志文件的互斥锁
    std::mutex logMutex_;
};
