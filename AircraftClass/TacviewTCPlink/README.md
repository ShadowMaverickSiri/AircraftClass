# TacviewTCPlink

一个独立、轻量的 Windows C++ Tacview 实时遥测模块。它接收统一的对象状态，负责 ACMI 2.2 编码、TCP 握手、客户端管理、异步发送和可选文件记录。

模块不依赖 AircraftClass、机动模型、SimTools 或 Eigen。

## 目录

```text
TacviewTCPlink/
├── include/tacview/
│   ├── Tacview.h           # 普通使用者需要的公共 API
│   └── AcmiEncoder.h       # 可选的底层 ACMI 编码 API
├── src/
│   ├── Tacview.cpp         # TCP、线程、队列、对象注册表和记录
│   └── AcmiEncoder.cpp     # 纯文本 ACMI 2.2 编码
├── examples/
│   └── TacviewExample.cpp  # 可直接连接 Tacview 的最小示例
├── tests/
│   └── TacviewTest.cpp     # 编码与公共接口自动测试
├── docs/
│   ├── 使用手册.md
│   └── 代码框架说明.md
└── CMakeLists.txt
```

## 快速构建

```powershell
cmake -S . -B build -DTACVIEW_BUILD_EXAMPLE=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix package
```

如果从 AircraftClass 仓库根目录执行：

```powershell
cmake -S AircraftClass/TacviewTCPlink -B build/tacview -DTACVIEW_BUILD_EXAMPLE=ON
cmake --build build/tacview --config Release
```

执行安装命令后，`package/include`、`package/lib` 和 `package/docs` 可以直接作为发布包交付。

静态库名称按构建配置区分：

- Debug：`TacviewTCPlink_d.lib`
- Release：`TacviewTCPlink.lib`

## 最小代码

```cpp
#include "tacview/Tacview.h"

tacview::Tacview output;
output.start();
output.addObject({1, "Aircraft", "Air+FixedWing", "Blue"});

tacview::ObjectState state;
state.id = 1;
state.time = 0.0;
state.longitude = 116.0;
state.latitude = 40.0;
state.altitude = 5000.0;
state.yaw = 90.0;
output.update(state);

output.stop();
```

坐标使用度，高度使用米，姿态使用度，时间使用秒。详细内容见 [使用手册](docs/使用手册.md)。
