# Tacview 通信接口说明文档

## 目录
1. [概述](#概述)
2. [架构设计](#架构设计)
3. [通信协议](#通信协议)
4. [核心类接口](#核心类接口)
5. [ACMI数据格式](#acmi数据格式)
6. [使用指南](#使用指南)
7. [技术要点](#技术要点)

---

## 概述

Tacview通信模块实现了与Tacview飞行数据分析软件的实时遥测数据传输功能。模块采用TCP客户端-服务器架构，支持多客户端连接，数据格式符合ACMI 2.2标准。

### 核心特性
- **实时传输**：TCP直连，低延迟数据传输
- **多客户端**：支持多个Tacview实例同时连接
- **标准协议**：完全符合Tacview ACMI 2.2规范
- **线程安全**：内置互斥锁保护共享资源
- **日志记录**：可选的数据记录功能

---

## 架构设计

### 组件结构

```
┌─────────────────────────────────────────────────────────────┐
│                     应用层代码                               │
│  (KinematicManeuverExample, FlightSimulation, etc.)        │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ 调用接口
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                   TacviewTelemetry                          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  • start()          - 启动TCP服务器                  │    │
│  │  • stop()           - 停止服务器                     │    │
│  │  • broadcastLine()  - 广播数据行                     │    │
│  │  • hasClients()     - 查询连接状态                   │    │
│  └─────────────────────────────────────────────────────┘    │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ 内部使用
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                        ACMI                                  │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  • createHeader()    - 生成文件头                    │    │
│  │  • setFileType()    - 设置文件类型                  │    │
│  │  • setObjectID()    - 设置对象ID                    │    │
│  │  • setPosition()    - 设置位置                      │    │
│  │  • setOrientation() - 设置姿态角                    │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                       │
                       │ 网络传输
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                   Tacview客户端                             │
│              (实时显示飞行轨迹和姿态)                         │
└─────────────────────────────────────────────────────────────┘
```

### 文件组成

| 文件 | 功能 | 行数 |
|------|------|------|
| `TacviewTelemetry.h` | TCP服务器类声明 | ~200 |
| `TacviewTelemetry.cpp` | TCP服务器类实现 | ~270 |
| `ACMI.h` | ACMI格式生成器声明 | ~230 |
| `ACMI.cpp` | ACMI格式生成器实现 | ~330 |

---

## 通信协议

### 握手协议

客户端连接后，服务器立即发送握手协议：

```
XtraLib.Stream.0\n
Tacview.RealTimeTelemetry.0\n
AircraftClass Server\n
\0
```

**关键点**：
- 必须以`\0`（空字符）结尾
- 三行文本格式固定
- 服务器名称可自定义

### ACMI数据流格式

#### 1. 文件头（必须首先发送）

```
FileType=text/acmi/tacview
FileVersion=2.2
0,ReferenceTime=2011-06-02T05:00:00Z
0,DataSource=AircraftClass Simulation
0,DataRecorder=KinematicManeuverSystem
0,Author=AircraftClass
0,Title=Maneuver Simulation
0,Comments=Real-time telemetry from AircraftClass
0,ReferenceLongitude=116.0000000000
0,ReferenceLatitude=39.0000000000
```

#### 2. 时间帧标记

```
#0.1
```

格式：`#` + 相对时间（秒）

#### 3. 对象定义

```
3EC,Type=Air+FixedWing,Name=Su-27,Color=Orange,T=116.000000|39.000000|9000.0
```

字段说明：
- `3EC`：对象ID（十六进制）
- `Type=Air+FixedWing`：对象类型
- `Name=Su-27`：对象名称
- `Color=Orange`：显示颜色
- `T=经度|纬度|高度`：初始位置

#### 4. 状态更新

```
3EC,T=116.000123|39.000456|9050.5|10.2|-5.3|180.0
```

字段顺序：`经度|纬度|高度|滚转|俯仰|偏航`

---

## 核心类接口

### TacviewTelemetry类

#### 公共接口

| 方法 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `start()` | `port` (默认42674) | `bool` | 启动TCP服务器 |
| `stop()` | 无 | `void` | 停止服务器 |
| `broadcastLine()` | `line` (字符串) | `void` | 广播数据行 |
| `isRunning()` | 无 | `bool` | 查询运行状态 |
| `hasClients()` | 无 | `bool` | 查询客户端连接 |
| `enableLogging()` | `logFile` (路径) | `void` | 启用日志 |

#### 私有接口

| 方法 | 说明 |
|------|------|
| `acceptLoop()` | 接受客户端连接的线程函数 |
| `sendAll()` | 可靠发送，确保数据完整传输 |
| `sendHandshake()` | 发送Tacview握手协议 |
| `removeClientLocked()` | 移除断开的客户端 |

### ACMI类

#### 文件元数据设置

| 方法 | 说明 |
|------|------|
| `setFileType()` | 设置文件类型（标准值："text/acmi/tacview"） |
| `setFileVersion()` | 设置文件版本（标准值："2.2"） |
| `setReferenceTime()` | 设置参考时间（ISO 8601格式） |
| `setDataSource()` | 设置数据源标识 |
| `setDataRecorder()` | 设置数据记录器标识 |
| `setAuthor()` | 设置作者信息 |
| `setTitle()` | 设置标题 |
| `setComments()` | 设置注释 |
| `createHeader()` | 生成文件头内容 |

#### 对象属性设置

| 方法 | 说明 |
|------|------|
| `setObjectID()` | 设置对象ID（十六进制） |
| `setObjectName()` | 设置对象名称 |
| `setPosition()` | 设置位置（经纬度） |
| `setOrientation()` | 设置姿态角（俯仰、滚转、偏航） |
| `setAltitude()` | 设置海拔高度 |
| `setColor()` | 设置显示颜色 |
| `setValue()` | 设置自定义属性 |

---

## ACMI数据格式

### 数值精度要求

| 字段 | 精度 | 单位 | 示例 |
|------|------|------|------|
| 经度 | 6位小数 | 度 | 116.000123 |
| 纬度 | 6位小数 | 度 | 39.000456 |
| 高度 | 1位小数 | 米 | 9000.0 |
| 滚转角 | 1位小数 | 度 | 10.5 |
| 俯仰角 | 1位小数 | 度 | -5.3 |
| 偏航角 | 1位小数 | 度 | 180.0 |

### 对象类型

常见类型标识：
- `Air+FixedWing`：固定翼飞机
- `Air+Rotorcraft`：直升机
- `Ground+Vehicle`：地面车辆
- `Weapon+Missile`：导弹

### 显示颜色

常用颜色：
- `Red`：红色（敌方）
- `Blue`：蓝色（友方）
- `Green`：绿色
- `Orange`：橙色（中立）
- `Yellow`：黄色

---

## 使用指南

### 基本使用流程

```cpp
#include "TacviewTelemetry.h"
#include "ACMI.h"

// 1. 创建并启动服务器
TacviewTelemetry telemetry;
telemetry.start(42674);

// 2. 等待客户端连接
while (!telemetry.hasClients()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 3. 生成并发送文件头
acmi generator;
generator.setFileType("text/acmi/tacview");
generator.setFileVersion("2.2");
generator.setDataSource("My Simulation");
generator.createHeader();
telemetry.broadcastLine(generator.header);

// 4. 发送初始时间帧和对象定义
telemetry.broadcastLine("#0.0");
telemetry.broadcastLine("3EC,Type=Air+FixedWing,Name=F-16,Color=Red,T=116.0|39.0|9000.0");

// 5. 循环发送状态更新
for (double t = 0.1; t <= 10.0; t += 0.1) {
    // 更新飞行状态...

    // 发送时间帧
    char timeFrame[20];
    snprintf(timeFrame, sizeof(timeFrame), "#%.1f", t);
    telemetry.broadcastLine(timeFrame);

    // 发送状态更新
    char update[200];
    snprintf(update, sizeof(update), "3EC,T=%.6f|%.6f|%.1f|%.1f|%.1f|%.1f",
             lon, lat, alt, roll, pitch, yaw);
    telemetry.broadcastLine(update);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

// 6. 停止服务器
telemetry.stop();
```

### 启用日志记录

```cpp
// 启动服务器后启用日志
telemetry.enableLogging("telemetry_log.acmi");

// 所有通过broadcastLine发送的数据都会记录到文件
```

---

## 技术要点

### 线程安全

1. **客户端列表保护**
   - 使用`clientsMutex_`保护`clients_`向量
   - `broadcastLine()`内部已加锁，外部调用无需同步

2. **日志文件保护**
   - 使用`logMutex_`保护日志写入
   - 防止并发写入导致数据损坏

### 可靠传输

1. **sendAll()函数**
   - 循环调用send()直到所有数据发送完毕
   - 处理TCP分包问题

2. **客户端断开处理**
   - 发送失败自动移除断开的客户端
   - 不影响其他客户端的数据传输

### 性能优化

1. **缓冲区大小**
   - 文件头缓冲区：1000字节
   - 帧数据缓冲区：1000字节
   - 足够应对大多数场景

2. **更新频率**
   - 建议：10-20 Hz（50-100毫秒间隔）
   - 过高频率可能导致网络拥塞

### 错误处理

| 错误场景 | 处理方式 |
|----------|----------|
| bind()失败 | 打印错误信息，返回false |
| accept()失败 | 打印错误信息，继续监听 |
| send()失败 | 移除断开的客户端 |
| 无客户端连接 | broadcastLine()直接返回 |

---

## 常见问题

### Q: 客户端连接后Tacview不显示数据？

A: 检查以下几点：
1. 是否先发送文件头？
2. 对象ID是否一致？
3. 数值格式是否符合精度要求？

### Q: 如何调试通信问题？

A: 启用日志记录功能：
```cpp
telemetry.enableLogging("debug.acmi");
```
生成的.acmi文件可直接用Tacview打开查看。

### Q: 支持多个飞行对象吗？

A: 支持。使用不同的对象ID即可：
```cpp
telemetry.broadcastLine("3EC,Type=Air+FixedWing,Name=F-16,Color=Red,T=...");
telemetry.broadcastLine("3ED,Type=Air+FixedWing,Name=Su-27,Color=Blue,T=...");
```

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2024 | 初始版本，支持ACMI 2.2标准 |

---

## 参考资料

- [Tacview官方网站](https://www.tacview.net)
- [ACMI文件格式规范](https://www.tacview.net/documentation/acmi/en/)
- [Tacview实时遥测协议](https://www.tacview.net/documentation/real-time-telemetry/en/)

---

**文档生成日期**: 2024年
**适用版本**: AircraftClass v1.0
