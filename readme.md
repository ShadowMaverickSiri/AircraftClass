# AircraftClass - 飞行器机动仿真系统

## 项目简介

AircraftClass 是一个用 C++ 实现的飞行器机动仿真系统，主要用于建模和模拟战斗机典型机动动作。系统支持 Tacview 实时遥测，可以直观地观察飞行轨迹和姿态变化。

## 主要功能

### 1. 机动模型 (KinematicManeuverSystem)

实现了四种经典战斗机机动：

- **水平转弯 (Level Turn)**：飞机在水平面内进行恒定过载转弯
- **筋斗 (Loop)**：飞机在垂直平面内完成 360° 闭合圆形轨迹
- **横滚 (Roll)**：飞机绕纵轴滚转指定角度
- **半滚倒转 (Split-S)**：滚转 180° + 向下半筋斗，用于快速改出和高度损失

### 2. Tacview 实时遥测

- 支持 Tacview 实时遥测协议 (TCP 端口 42674)
- 自动发送 ACMI 格式数据
- 包含时间帧标记、对象初始化和状态更新

### 3. 飞行器模型库 (AircraftModelLibrary)

提供基础的飞行器物理模型：
- 地理坐标系 (经度/纬度/高度)
- 速度向量 (北/天/东)
- 姿态角 (俯仰/滚转/偏航)
- 地球常数和单位转换

## 文件结构

```
AircraftClass/
├── AircraftModelLibrary.h/cpp    # 飞行器模型基础库
├── KinematicManeuverSystem.h/cpp # 机动模型实现
├── KinematicManeuverExample.cpp  # 示例程序
├── TacviewTelemetry.h/cpp        # Tacview 遥测接口
├── ACMI.h/cpp                    # ACMI 文件格式支持
└── EulerAngleCalculator.h        # 欧拉角计算工具
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

## 机动定义

### 筋斗 (Loop)
飞机在垂直平面内完成 360° 圆周运动，从平飞状态开始，向上拉起，完成一圈后回到平飞状态。

### 半滚倒转 (Split-S)
1. 第一阶段：飞机滚转 180° 进入倒飞状态
2. 第二阶段：在倒飞状态下推杆，完成向下的 180° 垂直半圆
3. 最终状态：恢复正飞，航向反转 180°，高度损失 2R (R 为筋斗半径)

## 示例代码

```cpp
#include "KinematicManeuverSystem.h"

using namespace KinematicManeuver;

// 创建机动模型
auto loop = Factory::create(Type::LOOP);

// 配置参数
Parameters params = Factory::getDefaultParams(Type::LOOP);
params.initialPosition = {121.5, 25.0, 1000};  // 经度, 纬度, 高度
params.initialVelocity = {200, 0, 50};        // 北速, 天速, 东速
params.targetGForce = 4.0;                     // 目标过载

// 初始化并运行
loop->initialize(params);

// 在仿真循环中更新
loop->update(currentTime, dt, position, velocity, attitude);
```

## 许可证

MIT License

## 作者

liusi

## 更新日志

- 2025-03-03: 修复筋斗和半滚倒转机动轨迹，修正 Tacview 遥测协议
