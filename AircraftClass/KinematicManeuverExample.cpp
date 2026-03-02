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
    std::string createObjectUpdate(const GeoPosition& pos, const Velocity3& vel, const AttitudeAngles& att) {
        std::ostringstream oss;

        // ACMI 格式: ID,T=经度|纬度|高度|俯仰|横滚|偏航|北速|天速|东速
        oss << objectIDStr_ << ",T="
            << std::fixed << std::setprecision(8)
            << pos.longitude << "|"
            << pos.latitude << "|"
            << std::fixed << std::setprecision(1)
            << pos.altitude << "|"
            << (att.pitch * 180.0 / M_PI) << "|"
            << (att.roll * 180.0 / M_PI) << "|"
            << (att.yaw * 180.0 / M_PI) << "|"
            << vel.north << "|"
            << vel.up << "|"
            << vel.east;
        return oss.str();
    }

    // 设置对象ID
    void setObjectID(uint64_t id) {
        objectID_ = id;
        char buffer[20];
        sprintf_s(buffer, "%llX", objectID_);
        objectIDStr_ = buffer;
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

    // 发送 ACMI 文件头
    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 发送对象定义
    telemetry.broadcastLine(exporter.createAircraftObject("F-16", "Red"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n连接成功后，按 Enter 键开始仿真...\n";
    std::cin.ignore();
    std::cin.get();

    std::cout << "\n机型: F-16 Fighting Falcon\n";
    std::cout << "初始高度: " << fighter.position.altitude << " m\n";
    std::cout << "初始速度: " << params.initialVelocity.north << " m/s\n";
    std::cout << "机动参数: " << params.targetGForce << "G, " << params.duration << "秒\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    // 仿真步长
    const double dt = 0.1;

    for (double t = 0.0; t <= params.duration + 2.0; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());

        // 发送 Tacview 遥测数据
        telemetry.broadcastLine(exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude));
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
    params.duration = 20.0;

    auto maneuver = Factory::create(Type::LOOP);
    maneuver->initialize(params);

    TacviewExporter exporter;
    exporter.setObjectID(0x3EA);  // 不同对象ID

    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    telemetry.broadcastLine(exporter.createAircraftObject("F-22", "Blue"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n连接成功后，按 Enter 键开始仿真...\n";
    std::cin.ignore();
    std::cin.get();

    std::cout << "\n机型: F-22 Raptor\n";
    std::cout << "机动参数: " << params.targetGForce << "G 筋斗, " << params.duration << "秒\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    const double dt = 0.1;

    for (double t = 0.0; t <= params.duration; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());
        telemetry.broadcastLine(exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude));
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
    params.duration = 15.0;
    params.rollDirection = 1.0;

    auto maneuver = Factory::create(Type::SPLIT_S);
    maneuver->initialize(params);

    TacviewExporter exporter;
    exporter.setObjectID(0x3EB);

    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    telemetry.broadcastLine(exporter.createAircraftObject("F-35", "Green"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n连接成功后，按 Enter 键开始仿真...\n";
    std::cin.ignore();
    std::cin.get();

    std::cout << "\n机型: F-35 Lightning II\n";
    std::cout << "初始高度: " << fighter.position.altitude << " m\n";
    std::cout << "机动参数: " << params.duration << "秒 半滚倒转\n\n";

    std::vector<SimulationRecord> records;
    const double dt = 0.1;

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    for (double t = 0.0; t <= params.duration; t += dt) {
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

        // 发送 Tacview 数据
        telemetry.broadcastLine(exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude));
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

    telemetry.broadcastLine(exporter.initializeHeader());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    telemetry.broadcastLine(exporter.createAircraftObject("Su-27", "Orange"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "\n[Tacview] 请按以下步骤连接 Tacview:\n";
    std::cout << "  1. 打开 Tacview 软件\n";
    std::cout << "  2. 选择: 文件 -> 连接到实时遥测源...\n";
    std::cout << "  3. 选择 127.0.0.1:42674\n";
    std::cout << "\n连接成功后，按 Enter 键开始仿真...\n";
    std::cin.ignore();
    std::cin.get();

    std::cout << "\n机型: Su-27 Flanker\n";
    std::cout << "机动参数: " << params.duration << "秒 360度横滚\n\n";

    std::cout << "时间 | 经度 | 纬度 | 高度 | 过载 | 俯仰 | 滚转 | 偏航\n";
    std::cout << "-----------------------------------------------------------\n";

    const double dt = 0.1;

    for (double t = 0.0; t <= params.duration; t += dt) {
        maneuver->update(t, dt, fighter.position, fighter.velocity, fighter.attitude);
        printState(t, fighter.position, fighter.velocity, fighter.attitude, maneuver->getCurrentGForce());
        telemetry.broadcastLine(exporter.createObjectUpdate(fighter.position, fighter.velocity, fighter.attitude));
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
