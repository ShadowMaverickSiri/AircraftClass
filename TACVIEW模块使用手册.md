# Tacview 独立遥测模块使用手册

## 1. 模块用途

该模块把任意数据源的对象状态实时发送给 Tacview，并可同时保存为 `.acmi` 文件。
它不依赖飞行器类或机动模型。数据可以来自仿真程序、UDP、串口、CSV 回放或其他系统。

使用者只需接触 `Tacview.h` 中的一个类：`tacview::Tacview`。

## 2. 最小使用流程

```cpp
#include "Tacview.h"

tacview::Tacview output;

if (!output.start()) {
    std::cerr << output.lastError() << '\n';
    return;
}

output.addObject({1, "F-16C", "Air+FixedWing", "Blue"});

tacview::ObjectState state;
state.id = 1;
state.time = simulationTime;
state.longitude = longitudeDeg;
state.latitude = latitudeDeg;
state.altitude = altitudeM;
state.roll = rollDeg;
state.pitch = pitchDeg;
state.yaw = yawDeg;
output.update(state);

output.stop();
```

完整示例位于 `AircraftClass/TacviewMinimalExample.cpp`。

## 3. 固定单位

| 字段 | 单位 | 有效范围 |
|---|---|---|
| `time` | 秒 | 大于等于 0 |
| `longitude` | 度 | -180 到 180 |
| `latitude` | 度 | -90 到 90 |
| `altitude` | 米 | 有限数 |
| `roll/pitch/yaw` | 度 | 有限数 |

对象 ID 必须大于 0，因为 ACMI 保留 ID 0 表示全局属性。

## 4. 常用配置

```cpp
tacview::Options options;
options.port = 42674;
options.serverName = "My Simulator";
options.dataSource = "My Data Source";
options.recordingFile = "flight.acmi"; // 留空则不保存文件

tacview::Tacview output(options);
```

模块启动后在指定端口等待 Tacview。Tacview 晚于仿真连接也没有问题：新客户端会自动收到 ACMI 文件头和所有对象的最新状态。

## 5. 多对象和对象删除

每个对象使用唯一 ID：

```cpp
output.addObject({1, "Leader",   "Air+FixedWing", "Blue"});
output.addObject({2, "Wingman",  "Air+FixedWing", "Blue"});
output.addObject({3, "Opponent", "Air+FixedWing", "Red"});

output.update(leaderState);
output.update(wingmanState);
output.update(opponentState);

output.removeObject(3, simulationTime);
```

必须先 `addObject()`，再为该 ID 调用 `update()`。

## 6. 数据源适配

模块不要求数据源继承任何类。只需在取得一帧数据后填充 `ObjectState`：

```cpp
void sendToTacview(const ExternalData& source, tacview::Tacview& output) {
    tacview::ObjectState state;
    state.id = source.objectId;
    state.time = source.timestamp;
    state.longitude = source.lon;
    state.latitude = source.lat;
    state.altitude = source.alt;
    state.roll = source.roll;
    state.pitch = source.pitch;
    state.yaw = source.heading;
    output.update(state);
}
```

这层转换是数据源唯一需要编写的适配代码。

## 7. 错误处理

`start()`、`addObject()`、`update()` 和 `removeObject()` 返回 `bool`。失败后使用：

```cpp
std::cerr << output.lastError() << '\n';
```

模块会拒绝重复 ID、未注册对象、NaN、无穷值及非法经纬度，不会把无效数据发送给 Tacview。

发送队列有固定容量，默认可缓存 4096 条消息。如果生产速度长期超过网络发送速度，模块会丢弃最旧消息并始终保留对象最新状态；可通过 `queueCapacity` 调整容量。

## 8. 内部框架

```text
任意数据源
    │ ObjectState
    ▼
tacview::Tacview
    ├── 输入校验与对象注册表
    ├── AcmiEncoder（ACMI 2.2 文本）
    ├── 有界异步发送队列
    ├── Winsock TCP 服务与客户端快照
    └── 可选 ACMI 文件记录
```

`AcmiEncoder` 不接触网络；网络线程不依赖飞行器或机动模型。公开头文件也不包含项目中的飞行器类型。

## 9. 发布集成

将以下文件加入使用者的 Visual Studio 工程：

- `Tacview.h`
- `Tacview.cpp`
- `AcmiEncoder.h`
- `AcmiEncoder.cpp`

工程需要链接 Windows 系统库 `Ws2_32.lib`。源码已包含 MSVC 的自动链接指令，通常无需手工配置。

要求 C++14 或更高版本；推荐 C++17。模块没有 SimTools、Eigen 或机动模型依赖。

也可以只构建独立静态库：

```powershell
cmake -S AircraftClass -B build/tacview
cmake --build build/tacview --config Release
```

`CMakeLists.txt` 默认只编译 Tacview 模块，不会编译当前项目的机动模型。

运行内置的 ACMI 编码测试：

```powershell
ctest --test-dir build/tacview -C Release --output-on-failure
```
