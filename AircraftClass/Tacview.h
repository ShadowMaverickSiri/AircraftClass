#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace tacview {

using ObjectId = std::uint64_t;

// 所有位置和姿态单位固定，避免数据源之间产生隐式单位差异。
struct ObjectState {
    ObjectId id = 0;
    double time = 0.0;          // 仿真开始后的秒数
    double longitude = 0.0;     // 度
    double latitude = 0.0;      // 度
    double altitude = 0.0;      // 米
    double roll = 0.0;          // 度
    double pitch = 0.0;         // 度
    double yaw = 0.0;           // 度
};

struct ObjectInfo {
    ObjectId id = 0;
    std::string name = "Object";
    std::string type = "Air+FixedWing";
    std::string color = "Blue";
};

struct Options {
    unsigned short port = 42674;
    std::string serverName = "Tacview Telemetry";
    std::string dataSource = "C++ Simulation";
    std::string recordingFile;  // 留空表示不记录 ACMI 文件
    std::size_t queueCapacity = 4096;
};

// 面向普通使用者的唯一入口。网络、ACMI 编码和线程均由内部管理。
class Tacview {
public:
    explicit Tacview(Options options = {});
    ~Tacview();

    Tacview(const Tacview&) = delete;
    Tacview& operator=(const Tacview&) = delete;

    bool start();
    void stop();

    bool addObject(const ObjectInfo& object);
    bool update(const ObjectState& state);
    bool removeObject(ObjectId id, double time);

    bool isRunning() const;
    std::size_t clientCount() const;
    std::string lastError() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tacview
