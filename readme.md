# AircraftClass - 飞行器机动运动学仿真系统

## 项目简介

AircraftClass 是一个用 C++ 实现的飞行器机动运动学仿真系统，主要用于建模和模拟战斗机典型机动动作。系统支持 Tacview 实时遥测，可以直观地观察飞行轨迹和姿态变化。

## 主要功能

### 1. 机动模型 (KinematicManeuverSystem)

实现了四种经典战斗机机动：

| 机动类型 | 英文名称 | 说明 |
|---------|---------|------|
| 水平转弯 | Level Turn | 恒定过载的水平圆周运动 |
| 筋斗 | Loop | 垂直平面内的完整圆周运动 |
| 横滚 | Roll | 绕纵轴的滚转机动 |
| 半滚倒转 | Split-S | 先滚转180°倒飞，再完成半筋斗向下 |

### 2. 四元数姿态解算 (Quaternion Attitude Solver)

- 完整的四元数数学库实现
- 避免欧拉角万向节锁 (Gimbal Lock) 问题
- SLERP 球面线性插值实现平滑旋转过渡
- 支持欧拉角与四元数双向转换

### 3. Tacview 实时遥测

- 支持 Tacview 实时遥测协议 (TCP 端口 42674)
- 自动发送 ACMI 格式数据
- 包含时间帧标记、对象初始化和状态更新

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
│   ├── TacviewTelemetry.cpp            # Tacview遥测数据生成
│   └── ACMI.h/cpp                      # ACMI 文件格式支持
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
