# AircraftClass - 飞行器机动运动学仿真系统

## 项目简介

AircraftClass 是一个用 C++ 实现的飞行器机动运动学仿真系统，主要用于建模和模拟战斗机典型机动动作。系统支持 Tacview 实时遥测，可以直观地观察飞行轨迹和姿态变化。

## 主要功能

### 1. 机动模型 (KinematicManeuverSystem)

实现了十种经典战斗机机动：

#### 基础机动

| 机动类型 | 英文名称 | 说明 |
|---------|---------|------|
| 水平转弯 | Level Turn | 恒定过载的水平圆周运动 |
| 筋斗 | Loop | 垂直平面内的完整圆周运动 |
| 横滚 | Roll | 绕纵轴的滚转机动 |
| 半滚倒转 | Split-S | 先滚转180°倒飞，再完成半筋斗向下 |

#### 高级机动

| 机动类型 | 英文名称 | 说明 | 复杂度 |
|---------|---------|------|--------|
| 定速定高盘旋 | Constant Turn | 水平圆周运动，可指定圈数 | 低 |
| 跑道型盘旋 | Racetrack Pattern | 矩形跑道路径（直道段+弯道段） | 中 |
| 8字型盘旋 | Eight Pattern | 两个相切圆形成的8字形 | 中 |
| 置尾降高逃逸 | Immelman Escape | 最大速率掉转机头+下降至指定高度+改平 | 中 |
| L型机动 | L Pattern | 直飞→90度转→直飞 | 低 |
| S型机动 | S Pattern | 两次反向转弯形成S形航线 | 低 |

### 2. 四元数姿态解算 (Quaternion Attitude Solver)

- 完整的四元数数学库实现
- 避免欧拉角万向节锁 (Gimbal Lock) 问题
- SLERP 球面线性插值实现平滑旋转过渡
- 支持欧拉角与四元数双向转换

### 3. Tacview 实时遥测

- 支持 Tacview 实时遥测协议 (TCP 端口 42674)
- 提供独立的 `tacview::Tacview` 模块，不依赖机动模型、SimTools 或 Eigen
- 上游只需提交统一的 `ObjectState`，可接入仿真、网络或文件回放等数据源
- 自动完成 ACMI 2.2 编码、对象管理、多客户端发送和晚连接状态补发
- 支持异步发送和可选 `.acmi` 文件记录

最小调用流程：

```cpp
tacview::Tacview output;
output.start();
output.addObject({1, "F-16C", "Air+FixedWing", "Blue"});
output.update(state);
output.stop();
```

详细接口、单位约定和多对象示例见 [TACVIEW模块使用手册.md](TACVIEW模块使用手册.md)。

### 4. 飞行器模型库 (AircraftModelLibrary)

提供基础的飞行器物理模型：
- 地理坐标系 (经度/纬度/高度)
- 速度向量 (北/天/东)
- 姿态角 (俯仰/滚转/偏航)
- 地球常数和单位转换

## 文件结构

```
AircraftClass-main/
├── AircraftClass/
│   ├── AircraftModelLibrary.h          # 基础数据结构和常量定义
│   ├── KinematicManeuverSystem.h       # 机动模型头文件（含四元数类）
│   ├── KinematicManeuverSystem.cpp     # 机动模型实现
│   ├── KinematicManeuverExample.cpp    # 使用示例程序
│   ├── Tacview.h/cpp                   # 推荐的独立遥测模块
│   ├── AcmiEncoder.h/cpp               # ACMI 2.2 编码器
│   ├── TacviewMinimalExample.cpp       # 独立模块最小示例
│   ├── CMakeLists.txt                  # 独立静态库构建入口
│   ├── TacviewTelemetry.h/cpp          # 旧示例兼容接口
│   └── ACMI.h/cpp                      # 旧版实验性编码器（不推荐新代码使用）
├── bin/
│   └── SimTools_v2.h                   # 静态库头文件
└── README.md
```

## 编译和运行

### 编译要求

- Visual Studio 2019/2022
- C++17 标准
- Windows 平台

### 使用方法

1. 编译项目
2. 运行生成的 `AircraftClass.exe`
3. 打开 Tacview 软件
4. 启用 Tacview 实时遥测 (连接到 localhost:42674)
5. 观察飞行轨迹

## 运动学模型说明

### 筋斗机动 (Loop)

筋斗半径公式：
```
R = v² / (g × (n - 1))
```

其中：
- `v` = 飞行速度 (m/s)
- `g` = 重力加速度 (9.81 m/s²)
- `n` = 过载值

完成时间：
```
T = 2πR / v = 2πv / (g × (n - 1))
```

### 半滚倒转 (Split-S)

半滚倒转由两个阶段组成：

1. **滚转阶段** (约2秒)：飞机滚转180°进入倒飞状态
2. **半筋斗阶段**：在倒飞状态下推杆，完成向下的半圆运动

半筋斗阶段的姿态使用四元数SLERP插值计算，确保：
- 滚转角从180°平滑过渡到0°
- 俯仰角自然变化（底部最大）
- 偏航角从初始航向过渡到反向航向

### 新增机动类型说明 (v3.0.0)

#### 定速定高盘旋 (Constant Turn)

水平圆周运动，支持指定完成圈数。

**参数配置：**
```cpp
params.targetGForce = 3.0;           // 转弯过载
params.turnDirection = -1.0;          // 转弯方向 (1.0=右转, -1.0=左转)
params.numCircles = 2;                // 完成圈数
params.autoCalculateDuration = true;  // 自动计算持续时间
```

**计算公式：**
```
转弯半径: R = v² / (g × (n - 1))
转弯速率: ω = g × (n - 1) / v
一圈时间: T = 2π / ω
总时间: T_total = T × numCircles
```

#### 跑道型盘旋 (Racetrack Pattern)

矩形跑道路径，由两段直道和两个180°转弯组成。

**参数配置：**
```cpp
params.straightLength = 8000.0;  // 直道段长度（米）
params.turnRadius = 1500.0;      // 转弯半径（米）
params.numLaps = 1;              // 圈数
params.autoCalculateDuration = true;
```

**阶段划分：**
1. LEG1_STRAIGHT - 第一段直道
2. LEG2_TURN - 第一个180°转弯
3. LEG3_STRAIGHT - 第二段直道（反向）
4. LEG4_TURN - 第二个180°转弯

#### 8字型盘旋 (Eight Pattern)

两个相切圆形成的8字形轨迹。

**参数配置：**
```cpp
params.targetGForce = 4.0;          // 转弯过载
params.turnDirection = -1.0;         // 转弯方向
params.autoCalculateDuration = true;
```

**几何原理：**
- 第一个圆：从切点开始，完成360°圆周运动
- 第二个圆：从第一个圆结束点开始，完成相反方向的360°圆周运动
- 两个圆圆心位于起始点两侧，距离为2×转弯半径

#### 置尾降高逃逸 (Immelman Escape)

战术机动：快速掉转机头并下降至安全高度。

**参数配置：**
```cpp
params.targetAltitude = 8000.0;   // 目标高度（米）
params.descentRate = 80.0;        // 下降速率（米/秒）
params.maxTurnRate = 15.0;        // 最大转弯速率（度/秒）
params.duration = 40.0;           // 总持续时间
```

**阶段划分：**
1. TURN_PHASE - 以最大速率掉转机头180°
2. DESCENT_PHASE - 以指定速率下降至目标高度
3. LEVEL_PHASE - 改平保持高度飞行

#### L型机动 (L Pattern)

直飞→90°转弯→直飞，常用于航线转换。

**参数配置：**
```cpp
params.leg1Distance = 8000.0;  // 第一段距离（米）
params.turnAngle = 90.0;       // 转弯角度（度）
params.targetGForce = 3.0;     // 转弯过载
```

#### S型机动 (S Pattern)

两次反向转弯形成S形航线，常用于规避机动。

**参数配置：**
```cpp
params.sTurnRadius = 1500.0;  // 转弯半径（米）
params.numTurns = 2;          // 转弯次数
params.targetGForce = 4.0;    // 转弯过载
```

### 航向角归一化

所有新增机动类型都实现了航向角归一化，确保角度值始终保持在 `[-π, π]` 范围内：

```cpp
attitude.yaw = currentHeading;
while (attitude.yaw > M_PI) attitude.yaw -= 2.0 * M_PI;
while (attitude.yaw < -M_PI) attitude.yaw += 2.0 * M_PI;
```

这对于多圈机动和持续转弯尤为重要，可防止角度累积导致的 Tacview 显示异常。

## 四元数工具类

项目实现了完整的四元数工具类，用于姿态计算：

```cpp
class Quaternion {
public:
    double w, x, y, z;

    // 从欧拉角构造 (ZYX顺序: yaw->pitch->roll)
    static Quaternion fromEuler(double roll, double pitch, double yaw);

    // 从旋转轴和角度构造
    static Quaternion fromAxisAngle(double axisX, double axisY, double axisZ, double angle);

    // 转换为欧拉角
    AttitudeAngles toEuler() const;

    // 球面线性插值 (SLERP)
    static Quaternion slerp(const Quaternion& q0, const Quaternion& q1, double t);

    // 四元数运算
    Quaternion operator*(const Quaternion& q) const;
    Quaternion operator*(double scalar) const;
    Quaternion operator+(const Quaternion& q) const;
    Quaternion normalized() const;
    Quaternion conjugate() const;
    Quaternion inverse() const;
    double norm() const;
    double dot(const Quaternion& q) const;
};
```

## 数据结构

### GeoPosition - 地理位置结构

```cpp
struct GeoPosition {
    double longitude;  // 经度 (度)
    double latitude;   // 纬度 (度)
    double altitude;   // 高度 (米)
};
```

### Velocity3 - 三维速度结构 (北天东坐标系)

```cpp
struct Velocity3 {
    double north;  // 北向速度 (m/s)
    double up;     // 天向速度 (m/s)
    double east;   // 东向速度 (m/s)
};
```

### AttitudeAngles - 姿态角结构

```cpp
struct AttitudeAngles {
    double pitch;  // 俯仰角 (弧度)
    double roll;   // 滚转角 (弧度)
    double yaw;    // 偏航角 (弧度)

    double getPitchDegrees() const;
    double getRollDegrees() const;
    double getYawDegrees() const;
};
```

## 常量定义

```cpp
namespace Constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;
    constexpr double EARTH_RADIUS = 6378137.0;  // 地球半径 (米)
    constexpr double G = 9.80665;                // 标准重力加速度 (m/s²)
}
```

## 示例代码

```cpp
#include "KinematicManeuverSystem.h"

using namespace KinematicManeuver;

// 创建机动模型
auto splitS = Factory::create(Type::SPLIT_S);

// 配置参数
Parameters params = Factory::getDefaultParams(Type::SPLIT_S);
params.targetGForce = 4.0;                      // 4G过载
params.duration = 25.0;                          // 25秒完成机动
params.initialPosition = {120.0, 30.0, 12000.0}; // 经度、纬度、高度
params.initialVelocity = {300.0, 0.0, 0.0};      // 北向、天向、东向速度

// 初始化并运行
splitS->initialize(params);

// 在仿真循环中更新
GeoPosition position;
Velocity3 velocity;
AttitudeAngles attitude;

double currentTime = 0.0;
double dt = 0.1;  // 时间步长

while (currentTime < params.duration) {
    splitS->update(currentTime, dt, position, velocity, attitude);
    // 处理position, velocity, attitude数据
    currentTime += dt;
}
```

## 技术要点

### Split-S 机动姿态解算

Split-S 机动使用四元数 SLERP 插值计算姿态，避免欧拉角的万向节锁问题：

1. **起点四元数**：倒飞状态 (roll=180°, pitch=0°, yaw=初始航向)
2. **终点四元数**：正飞状态 (roll=0°, pitch=0°, yaw=初始航向+180°)
3. **插值过程**：使用 SLERP 在两点间进行球面线性插值

这种方法确保：
- 姿态变化平滑连续
- 速度方向与姿态匹配
- 无异常滚转动画

## 许可证

MIT License

## 作者

liusi

## 更新日志

### v3.0.0 (2026-03-29)
- 新增6种高级机动类型：
  - 定速定高盘旋 (Constant Turn) - 支持指定圈数的水平圆周运动
  - 跑道型盘旋 (Racetrack Pattern) - 矩形跑道路径
  - 8字型盘旋 (Eight Pattern) - 两个相切圆形成的8字形
  - 置尾降高逃逸 (Immelman Escape) - 战术逃逸机动
  - L型机动 (L Pattern) - 航线转换机动
  - S型机动 (S Pattern) - 规避机动
- 实现航向角归一化，修复 Tacview 多圈机动显示异常问题
- 扩展 Parameters 结构体，新增多种机动专用参数
- 更新示例程序，新增6个示例函数和菜单选项
- 生成轨迹文件输出 (.dat 和 .csv 格式)

### v2.0.0 (2025-03-06)
- 重构 Split-S 机动模型，使用四元数 SLERP 插值
- 修复欧拉角万向节锁问题
- 修复姿态与速度方向不匹配的问题
- 优化筋斗机动，使用恒定速度模型（假设推力补偿能量损失）
- 添加完整的四元数工具类实现

### v1.0.0 (2025-03-03)
- 初始版本
- 基础机动模型实现
- Tacview 遥测支持
