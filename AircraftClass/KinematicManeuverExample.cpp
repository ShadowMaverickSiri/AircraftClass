// ============================================================================
// KinematicManeuverExample.cpp
// 运动学机动系统示例程序（带 Tacview 实时遥测）
// 演示如何使用运动学机动模型模拟战斗机机动，并同时向 Tacview 软件实时输出
// ============================================================================

#define _USE_MATH_DEFINES
#include "KinematicManeuverSystem.h"
#include "FighterJet.h"
#include "TacviewTelemetry.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>
#include <sstream>
#include <cmath>
#include <windows.h>

using namespace KinematicManeuver;

// ============================================================================
// Tacview 遥测辅助类
// ============================================================================
class TacviewExporter {
public:
    TacviewExporter() : objectID_(0x3E9), elapsedTime_(0.0) {}

    // 初始化 Tacview ACMI 文件头
    std::string initializeHeader() {
        std::ostringstream oss;
        oss << "FileType=text/acmi/tacview\n";
        oss << "FileVersion=2.2\n";
        oss << "0,ReferenceTime=2011-06-02T05:00:00Z\n";
        oss << "0,DataSource=AircraftClass Simulation\n";
        oss << "0,DataRecorder=KinematicManeuverSystem\n";
        oss << "0,Author=AircraftClass\n";
        oss << "0,Title=Maneuver Simulation\n";
        oss << "0,Comments=Real-time telemetry from AircraftClass\n";
        return oss.str();
    }

    // 设置飞机对象
    std::string createAircraftObject(const std::string& name, const std::string& color) {
        std::ostringstream oss;
        oss << objectIDStr_ << ",Type=Air+FixedWing,Name=" << name << ",Color=" << color;
        return oss.str();
    }

    // 生成对象状态更新行
    // ACMI 标准格式: ID,T=经度|纬度|高度|滚转|俯仰|偏航
    std::string createObjectUpdate(const GeoPosition& pos, const Velocity3& vel, const AttitudeAngles& att) {
        std::ostringstream oss;

        // 转换角度为度数
        double rollDeg = att.roll * 180.0 / M_PI;
        double pitchDeg = att.pitch * 180.0 / M_PI;
        double yawDeg = att.yaw * 180.0 / M_PI;

        oss << objectIDStr_ << ",T="
            << std::fixed << std::setprecision(6)
            << pos.longitude << "|"
            << pos.latitude << "|"
            << std::fixed << std::setprecision(1)
            << pos.altitude << "|"
            << rollDeg << "|"      // 滚转
            << pitchDeg << "|"     // 俯仰
            << yawDeg;             // 偏航
        return oss.str();
    }

    // 设置对象ID
    void setObjectID(uint64_t id) {
        objectID_ = id;
        char buffer[20];
        sprintf_s(buffer, "%llX", objectID_);
        objectIDStr_ = buffer;
    }

    // 生成时间帧标记（ACMI 格式必需）
    std::string createTimeFrame(double time) {
        std::ostringstream oss;
        oss << "#" << std::fixed << std::setprecision(1) << time;
        return oss.str();
    }

    uint64_t getObjectID() const { return objectID_; }

private:
    uint64_t objectID_;
    std::string objectIDStr_;
    double elapsedTime_;
};

// ============================================================================
// 辅助函数
// ============================================================================
void printHeader(const std::string& title) {
    std::cout << "\n";
    std::cout << "==============================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "==============================================\n";
}

void printState(double time, const GeoPosition& pos, const Velocity3& vel,
                const AttitudeAngles& att, double gForce) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "T=" << std::setw(6) << time << "s | "
              << "Lat=" << std::setw(9) << pos.latitude << " | "
              << "Lon=" << std::setw(9) << pos.longitude << " | "
              << "Alt=" << std::setw(7) << pos.altitude << "m | "
              << "G=" << std::setw(4) << gForce << " | "
              << "Pitch=" << std::setw(5) << (att.pitch * 180.0 / M_PI) << " | "
              << "Roll=" << std::setw(5) << (att.roll * 180.0 / M_PI) << " | "
              << "Yaw=" << std::setw(5) << (att.yaw * 180.0 / M_PI) << "\n";
}

// CSV 输出记录
struct SimulationRecord {
    double time;
    double latitude;
    double longitude;
    double altitude;
    double velocityNorth;
    double velocityUp;
    double velocityEast;
    double pitch;
    double roll;
    double yaw;
    double gForce;
};

// ============================================================================
// 示例1：水平转弯（带 Tacview 输出）
// ============================================================================
void exampleLevelTurn() {
    printHeader("示例1：水平转弯机动 (Level Turn) + Tacview遥测");

    // 创建 Tacview 服务器
    TacviewTelemetry telemetry;
    if (!telemetry.start(42674)) {
        std::cerr << "Failed to start Tacview telemetry server!\n";
        return;
    }

    // 创建 F-16 战斗机
    FighterJet fighter(FighterType::F16_FALCON,
                      {116.0, 39.0, 10000},  // 北京附近，高度10000米
                      {250, 0, 0});          // 250m/s 向北飞行

    // 创建机动参数
    auto params = Factory::getDefaultParams(Type::LEVEL_TURN);
    params.initialPosition = fighter.position;
    params.initialVelocity = fighter.velocity;
    params.targetGForce = 3.0;     // 3G 转弯
    params.duration = 20.0;        // 20秒
    params.turnDirection = 1.0;    // 右转

    // 创建机动模型
    auto maneuver = Factory::create(Type::LEVEL_TURN);
    maneuver->initialize(params);

    // 创建 Tacview 导出器
    TacviewExporter exporter;
    exporter.setObjectID(0x3E9);

    // 启用日志记录以便调试
    telemetry.enableLogging("E:\\MyCode\\AircraftClass-main\\x64\\Debug\\telemetry_log.acmi");

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n等待 Tacview 连接...\n";

    // 等待 Tacview 连接
    while (!telemetry.hasClients()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Tacview] 已连接! 发送初始化数据...\n\n";

    // Tacview 连接后，发送初始化数据
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等待握手完成

    // 发送 ACMI 文件头
    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送初始时间帧（ACMI 格式必需）
    std::string timeFrame0 = exporter.createTimeFrame(0.0);
    telemetry.broadcastLine(timeFrame0);
    std::cout << "[发送] " << timeFrame0 << "\n";

    // 发送对象定义（包含初始位置）
    std::ostringstream initLine;
    initLine << exporter.createAircraftObject("F-16", "Red")
             << ",T=" << std::fixed << std::setprecision(6)
             << fighter.position.longitude << "|"
             << fighter.position.latitude << "|"
             << std::fixed << std::setprecision(1)
             << fighter.position.altitude;
    std::string initStr = initLine.str();
    telemetry.broadcastLine(initStr);
    std::cout << "[发送] " << initStr << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n机型: F-16 Fighting Falcon\n";
    std::cout << "初始高度: " << fighter.position.altitude << " m\n";
    std::cout << "初始速度: " << params.initialVelocity.north << " m/s\n";
    std::cout << "机动参数: " << params.targetGForce << "G, " << params.duration << "秒\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    // 仿真步长
    const double dt = 0.1;
    bool firstPrint = true;

    for (double t = dt; t <= params.duration + 2.0; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());

        // 发送 Tacview 遥测数据：先发送时间帧，再发送对象更新
        std::string timeFrame = exporter.createTimeFrame(t);
        std::string objUpdate = exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude);
        telemetry.broadcastLine(timeFrame);
        telemetry.broadcastLine(objUpdate);

        // 打印第一次发送的数据用于调试
        if (firstPrint) {
            std::cout << "\n[调试] 第一次发送的数据:\n";
            std::cout << "  " << timeFrame << "\n";
            std::cout << "  " << objUpdate << "\n\n";
            firstPrint = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 降低发送速率，便于观察
    }

    std::cout << "\n机动完成! 您可以在 Tacview 中回放查看轨迹。\n";
    std::cout << "按 Enter 键返回主菜单...\n";
    std::cin.get();
    telemetry.stop();
}

// ============================================================================
// 示例2：筋斗机动（带 Tacview 输出）
// ============================================================================
void exampleLoop() {
    printHeader("示例2：筋斗机动 (Loop) + Tacview遥测");

    TacviewTelemetry telemetry;
    telemetry.start(42674);

    FighterJet fighter(FighterType::F22_RAPTOR,
                      {116.0, 39.0, 8000},
                      {300, 0, 0});

    auto params = Factory::getDefaultParams(Type::LOOP);
    params.initialPosition = fighter.position;
    params.initialVelocity = fighter.velocity;
    params.targetGForce = 4.0;
    params.duration = 25.0;

    auto maneuver = Factory::create(Type::LOOP);
    maneuver->initialize(params);

    TacviewExporter exporter;
    exporter.setObjectID(0x3EA);  // 不同对象ID

    // 启用日志记录
    telemetry.enableLogging("E:\\MyCode\\AircraftClass-main\\x64\\Debug\\telemetry_log.acmi");

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n等待 Tacview 连接...\n";

    // 等待 Tacview 连接
    while (!telemetry.hasClients()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Tacview] 已连接! 发送初始化数据...\n\n";

    // Tacview 连接后，发送初始化数据
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 发送 ACMI 文件头
    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送初始时间帧
    std::string timeFrame0 = exporter.createTimeFrame(0.0);
    telemetry.broadcastLine(timeFrame0);
    std::cout << "[发送] " << timeFrame0 << "\n";

    // 发送对象定义（包含初始位置）
    std::ostringstream initLine;
    initLine << exporter.createAircraftObject("F-22", "Blue")
             << ",T=" << std::fixed << std::setprecision(6)
             << fighter.position.longitude << "|"
             << fighter.position.latitude << "|"
             << std::fixed << std::setprecision(1)
             << fighter.position.altitude;
    std::string initStr = initLine.str();
    telemetry.broadcastLine(initStr);
    std::cout << "[发送] " << initStr << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n机型: F-22 Raptor\n";
    std::cout << "机动参数: " << params.targetGForce << "G 筋斗, " << params.duration << "秒\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    const double dt = 0.1;
    bool firstPrint = true;

    for (double t = dt; t <= params.duration; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());

        // 发送时间帧和对象更新
        std::string timeFrame = exporter.createTimeFrame(t);
        std::string objUpdate = exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude);
        telemetry.broadcastLine(timeFrame);
        telemetry.broadcastLine(objUpdate);

        if (firstPrint) {
            std::cout << "\n[调试] 第一次发送的数据:\n";
            std::cout << "  " << timeFrame << "\n";
            std::cout << "  " << objUpdate << "\n\n";
            firstPrint = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\n机动完成! 您可以在 Tacview 中回放查看轨迹。\n";
    std::cout << "按 Enter 键返回主菜单...\n";
    std::cin.get();
    telemetry.stop();
}

// ============================================================================
// 示例3：半滚倒转（带 CSV 输出 + Tacview）
// ============================================================================
void exampleSplitS() {
    printHeader("示例3：半滚倒转机动 (Split-S) + Tacview遥测");

    TacviewTelemetry telemetry;
    telemetry.start(42674);

    FighterJet fighter(FighterType::F35_LIGHTNING,
                      {116.0, 39.0, 12000},
                      {280, 0, 0});

    auto params = Factory::getDefaultParams(Type::SPLIT_S);
    params.initialPosition = fighter.position;
    params.initialVelocity = fighter.velocity;
    params.duration = 30.;
    params.rollDirection = 1.0;

    auto maneuver = Factory::create(Type::SPLIT_S);
    maneuver->initialize(params);

    TacviewExporter exporter;
    exporter.setObjectID(0x3EB);

    // 启用日志记录
    telemetry.enableLogging("E:\\MyCode\\AircraftClass-main\\x64\\Debug\\telemetry_log.acmi");

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n等待 Tacview 连接...\n";

    // 等待 Tacview 连接
    while (!telemetry.hasClients()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Tacview] 已连接! 发送初始化数据...\n\n";

    // Tacview 连接后，发送初始化数据
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 发送 ACMI 文件头
    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送初始时间帧
    std::string timeFrame0 = exporter.createTimeFrame(0.0);
    telemetry.broadcastLine(timeFrame0);
    std::cout << "[发送] " << timeFrame0 << "\n";

    // 发送对象定义（包含初始位置）
    std::ostringstream initLine;
    initLine << exporter.createAircraftObject("F-35", "Green")
             << ",T=" << std::fixed << std::setprecision(6)
             << fighter.position.longitude << "|"
             << fighter.position.latitude << "|"
             << std::fixed << std::setprecision(1)
             << fighter.position.altitude;
    std::string initStr = initLine.str();
    telemetry.broadcastLine(initStr);
    std::cout << "[发送] " << initStr << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n机型: F-35 Lightning II\n";
    std::cout << "初始高度: " << fighter.position.altitude << " m\n";
    std::cout << "机动参数: " << params.duration << "秒 半滚倒转\n\n";

    std::vector<SimulationRecord> records;
    const double dt = 0.1;
    bool firstPrint = true;

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    for (double t = dt; t <= params.duration; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);

        // 记录数据
        records.push_back({
            t,
            fighter.position.latitude,
            fighter.position.longitude,
            fighter.position.altitude,
            fighter.velocity.north,
            fighter.velocity.up,
            fighter.velocity.east,
            fighter.attitude.pitch,
            fighter.attitude.roll,
            fighter.attitude.yaw,
            maneuver->getCurrentGForce()
        });

        // 发送 Tacview 数据：先发送时间帧，再发送对象更新
        std::string timeFrame = exporter.createTimeFrame(t);
        std::string objUpdate = exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude);
        telemetry.broadcastLine(timeFrame);
        telemetry.broadcastLine(objUpdate);

        if (firstPrint) {
            std::cout << "\n[调试] 第一次发送的数据:\n";
            std::cout << "  " << timeFrame << "\n";
            std::cout << "  " << objUpdate << "\n\n";
            firstPrint = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // 每秒打印一次
        if (static_cast<int>(t * 10) % 10 == 0) {
            printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());
        }
    }

    std::cout << "\n机动完成!\n";

    // 保存到 CSV 文件
    std::ofstream csv("SplitS_Simulation.csv");
    csv << "Time,Latitude,Longitude,Altitude,V_north,V_up,V_east,Pitch,Roll,Yaw,GForce\n";
    for (const auto& r : records) {
        csv << r.time << ","
            << r.latitude << ","
            << r.longitude << ","
            << r.altitude << ","
            << r.velocityNorth << ","
            << r.velocityUp << ","
            << r.velocityEast << ","
            << r.pitch << ","
            << r.roll << ","
            << r.yaw << ","
            << r.gForce << "\n";
    }
    csv.close();

    std::cout << "\n数据已保存到: SplitS_Simulation.csv\n";

    // 统计信息
    double altLoss = records[0].altitude - records.back().altitude;
    std::cout << "高度损失: " << altLoss << " m\n";
    std::cout << "最终高度: " << records.back().altitude << " m\n";
    std::cout << "\n您可以在 Tacview 中回放查看轨迹。\n";
    std::cout << "按 Enter 键返回主菜单...\n";
    std::cin.get();

    telemetry.stop();
}

// ============================================================================
// 示例4：横滚机动（带 Tacview 输出）
// ============================================================================
void exampleRoll() {
    printHeader("示例4：横滚机动 (Roll) + Tacview遥测");

    TacviewTelemetry telemetry;
    telemetry.start(42674);

    FighterJet fighter(FighterType::SU27_FLANKER,
                      {116.0, 39.0, 9000},
                      {270, 0, 0});

    auto params = Factory::getDefaultParams(Type::ROLL);
    params.initialPosition = fighter.position;
    params.initialVelocity = fighter.velocity;
    params.duration = 4.0;
    params.rollDirection = 1.0;

    auto maneuver = Factory::create(Type::ROLL);
    maneuver->initialize(params);

    TacviewExporter exporter;
    exporter.setObjectID(0x3EC);

    // 启用日志记录
    telemetry.enableLogging("E:\\MyCode\\AircraftClass-main\\x64\\Debug\\telemetry_log.acmi");

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n等待 Tacview 连接...\n";

    // 等待 Tacview 连接
    while (!telemetry.hasClients()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "[Tacview] 已连接! 发送初始化数据...\n\n";

    // Tacview 连接后，发送初始化数据
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 发送 ACMI 文件头
    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送初始时间帧
    std::string timeFrame0 = exporter.createTimeFrame(0.0);
    telemetry.broadcastLine(timeFrame0);
    std::cout << "[发送] " << timeFrame0 << "\n";

    // 发送对象定义（包含初始位置）
    std::ostringstream initLine;
    initLine << exporter.createAircraftObject("Su-27", "Orange")
             << ",T=" << std::fixed << std::setprecision(6)
             << fighter.position.longitude << "|"
             << fighter.position.latitude << "|"
             << std::fixed << std::setprecision(1)
             << fighter.position.altitude;
    std::string initStr = initLine.str();
    telemetry.broadcastLine(initStr);
    std::cout << "[发送] " << initStr << "\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n机型: Su-27 Flanker\n";
    std::cout << "机动参数: " << params.duration << "秒 360度横滚\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    const double dt = 0.1;
    bool firstPrint = true;

    for (double t = dt; t <= params.duration; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());

        // 发送时间帧和对象更新
        std::string timeFrame = exporter.createTimeFrame(t);
        std::string objUpdate = exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude);
        telemetry.broadcastLine(timeFrame);
        telemetry.broadcastLine(objUpdate);

        if (firstPrint) {
            std::cout << "\n[调试] 第一次发送的数据:\n";
            std::cout << "  " << timeFrame << "\n";
            std::cout << "  " << objUpdate << "\n\n";
            firstPrint = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "\n机动完成! 您可以在 Tacview 中回放查看轨迹。\n";
    std::cout << "按 Enter 键返回主菜单...\n";
    std::cin.get();
    telemetry.stop();
}

// ============================================================================
// 主程序
// ============================================================================
int main() {
    // 设置控制台为 UTF-8 编码
    SetConsoleOutputCP(CP_UTF8);

    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║     运动学机动系统示例程序 (带 Tacview 实时遥测)                ║\n";
    std::cout << "║     Kinematic Maneuver System + Tacview Real-time Telemetry      ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";

    std::cout << "\n使用说明:\n";
    std::cout << "1. 程序会自动启动 Tacview TCP 服务器 (端口 42674)\n";
    std::cout << "2. 启动 Tacview 软件\n";
    std::cout << "3. 在 Tacview 中选择: 文件 → 连接到实时遥测源...\n";
    std::cout << "4. 选择本机地址 (127.0.0.1:42674)\n";
    std::cout << "5. 连接成功后即可实时查看飞行轨迹\n\n";

    int choice = 0;
    while (choice != 5) {
        std::cout << "\n请选择示例:\n";
        std::cout << "  1. 水平转弯 (Level Turn)\n";
        std::cout << "  2. 筋斗机动 (Loop)\n";
        std::cout << "  3. 半滚倒转 (Split-S) - 输出 CSV\n";
        std::cout << "  4. 横滚机动 (Roll)\n";
        std::cout << "  5. 退出\n";
        std::cout << "选择: ";
        std::cin >> choice;

        switch (choice) {
            case 1: exampleLevelTurn(); break;
            case 2: exampleLoop(); break;
            case 3: exampleSplitS(); break;
            case 4: exampleRoll(); break;
            case 5: std::cout << "退出程序。\n"; break;
            default: std::cout << "无效选择，请重试。\n";
        }
    }

    return 0;
}
