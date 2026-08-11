#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace tacview {

// ACMI 对象标识使用无符号整数；编码时会转换为大写十六进制文本。
using ObjectId = std::uint64_t;

/**
 * @brief 某个对象在指定仿真时刻的完整遥测状态。
 *
 * 所有数据源都应先转换成该结构。固定单位可以避免不同来源在接入时
 * 隐式混用弧度/角度、千米/米。结构只描述数据，不依赖任何飞行模型。
 */
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

/**
 * @brief 对象首次出现时发送给 Tacview 的静态信息。
 *
 * 同一 ID 在注册期间只能对应一个对象。若需改变元数据，可先删除对象，
 * 再使用相同 ID 重新注册。
 */
struct ObjectInfo {
    ObjectId id = 0;                       // 唯一 ID；0 由 ACMI 保留
    std::string name = "Object";           // Tacview 界面显示名称
    std::string type = "Air+FixedWing";    // ACMI 类型，例如 Air+FixedWing
    std::string color = "Blue";            // Tacview 颜色，例如 Blue/Red
};

/** @brief 不参与模型匹配、可以在运行中更新的对象显示属性。 */
struct ObjectProperties {
    std::string shortName;                   // Tacview 默认对象短标签
    std::string label;                       // 额外自由文本，例如实时经纬度
};

/** @brief TCP 服务、ACMI 文件头和异步队列的启动配置。 */
struct Options {
    unsigned short port = 42674;                    // Tacview 实时遥测默认端口
    std::string serverName = "Tacview Telemetry";  // 握手中显示的服务名
    std::string dataSource = "C++ Simulation";     // ACMI 文件头的数据源
    std::string recordingFile;                      // 留空表示不记录文件
    std::size_t queueCapacity = 4096;               // 异步发送队列上限
};

/**
 * @brief 面向普通使用者的 Tacview 遥测入口。
 *
 * 典型调用顺序：start() -> addObject() -> update() -> stop()。
 * 网络、ACMI 编码、对象快照和线程均由内部管理。状态提交与查询接口使用
 * 同步保护；start() 和 stop() 属于生命周期操作，应由同一管理线程调用。
 */
class Tacview {
public:
    /** @brief 保存配置；不会在构造阶段占用端口或创建线程。 */
    explicit Tacview(Options options = {});
    /** @brief 自动停止服务；正常用法仍建议显式调用 stop()。 */
    ~Tacview();

    Tacview(const Tacview&) = delete;
    Tacview& operator=(const Tacview&) = delete;

    /**
     * @brief 启动监听、异步发送线程和可选文件记录。
     * @return 成功或已经启动时返回 true；失败原因由 lastError() 返回。
     */
    bool start();
    /** @brief 排空待发送数据，并关闭客户端、端口、线程和记录文件。 */
    void stop();

    /**
     * @brief 注册对象元数据，必须在第一次 update() 前调用。
     * @return ID 合法且未重复时返回 true。
     */
    bool addObject(const ObjectInfo& object);
    /**
     * @brief 提交一帧完整状态到异步队列。
     *
     * 即使暂时没有 Tacview 客户端也可以调用；最新状态会保存在对象注册表，
     * 供稍后连接的客户端获取快照。
     */
    bool update(const ObjectState& state);
    /** @brief 更新对象短名称和自由文本标签，不改变用于模型匹配的 Name。 */
    bool updateProperties(ObjectId id, double time, const ObjectProperties& properties);
    /** @brief 在给定仿真时间从场景移除对象并释放其快照状态。 */
    bool removeObject(ObjectId id, double time);

    /** @brief 查询服务是否已经成功启动且尚未停止。 */
    bool isRunning() const;
    /** @brief 返回当前已经完成握手的客户端数量。 */
    std::size_t clientCount() const;
    /** @brief 返回最近一次错误；成功调用不会主动清空旧错误。 */
    std::string lastError() const;

private:
    // PImpl 隔离 Winsock 与 STL 线程实现，使公共头文件保持轻量稳定。
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace tacview
