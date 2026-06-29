# Tacview 通信模块 - 快速导读

> 给协作者的简要说明：先看这个，再读详细文档

---

## 一分钟了解

这个模块向 Tacview 软件实时发送飞行数据。**两件事**：

1. **TCP服务器** (`TacviewTelemetry`) - 接收 Tacview 连接，广播数据
2. **数据格式化** (`ACMI`) - 生成 Tacview 能识别的数据格式

---

## 核心代码位置

```
AircraftClass/AircraftClass/
├── TacviewTelemetry.h       ← TCP服务器接口（先看这个）
├── TacviewTelemetry.cpp     ← TCP服务器实现
├── ACMI.h                    ← 数据格式化接口
├── ACMI.cpp                  ← 数据格式化实现
└── TACVIEW通信接口说明.md    ← 完整文档（有问题查这个）
```

---

## 看代码的顺序

### 第一步：看通信协议（2分钟）

**文件**: `TacviewTelemetry.cpp` 第 173-203 行

```cpp
void TacviewTelemetry::sendHandshake(SOCKET client)
{
    // 握手协议：必须以 \0 结尾！
    std::string handshake =
        "XtraLib.Stream.0\n"
        "Tacview.RealTimeTelemetry.0\n"
        "AircraftClass Server\n";

    sendAll(client, handshake.c_str(), ...);

    char terminator = '\0';  // ← 关键：必须有这个
    sendAll(client, &terminator, 1);
}
```

### 第二步：看数据发送（3分钟）

**文件**: `TacviewTelemetry.cpp` 第 240-268 行

```cpp
void TacviewTelemetry::broadcastLine(const std::string& line)
{
    // 1. 加锁保护客户端列表
    std::lock_guard<std::mutex> lk(clientsMutex_);

    // 2. 添加换行符
    std::string msg = line + "\n";

    // 3. 向所有客户端发送
    for (size_t i = 0; i < clients_.size(); ++i) {
        if (!sendAll(s, msg.c_str(), ...)) {
            removeClientLocked(i);  // 发送失败则移除客户端
        }
    }
}
```

### 第三步：看ACMI数据格式（5分钟）

**文件**: `ACMI.cpp` 第 19-37 行（格式模板）

```cpp
// 文件头格式
const char acmi::headerTemplate[] =
    "FileType=%s\n"           // 文件类型
    "FileVersion=%s\n"        // 版本号
    "0,ReferenceTime=...\n"  // 参考时间
    "...";

// 帧数据格式
const char acmi::frameTemplate[] =
    "#%s\n"                   // 时间帧标记，如 #0.1
    "%s,T=%s|%s|%s|...\n";    // 对象状态：经度|纬度|高度|姿态
```

---

## 必须理解的概念

### 1. 握手协议的 `\0` 终止符

```cpp
// 错误 ❌
send("Hello\n");

// 正确 ✅
send("Hello\n");
send("\0");  // Tacview 要求必须有这个空字符
```

**位置**: `TacviewTelemetry.cpp:196`

---

### 2. ACMI 数据的三种类型

```acmi
# 类型1: 文件头（只需发送一次）
FileType=text/acmi/tacview
FileVersion=2.2
0,ReferenceTime=2011-06-02T05:00:00Z
...

# 类型2: 时间帧（每个时间点）
#0.0
#0.1
#0.2

# 类型3: 对象状态（每个时间点，每个对象）
3EC,T=116.0|39.0|9000.0|0.0|0.0|0.0
3ED,T=116.1|39.1|9100.0|5.0|-3.0|180.0
```

**格式说明**:
- `#时间` → 时间帧标记
- `ID,T=经度|纬度|高度|滚转|俯仰|偏航` → 对象状态

---

### 3. broadcastLine() 的线程安全

```cpp
// 外部调用 - 无需加锁
telemetry.broadcastLine("...");  // ✅ 内部已处理

{
    std::lock_guard<std::mutex> lk(clientsMutex_);  // 内部自动加锁
    // ... 操作客户端列表
}
```

**位置**: `TacviewTelemetry.cpp:242`

---

## 使用示例（最快理解方式）

```cpp
// 1. 启动服务器
TacviewTelemetry telemetry;
telemetry.start(42674);

// 2. 等待 Tacview 连接
while (!telemetry.hasClients()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// 3. 发送文件头（一次性）
telemetry.broadcastLine("FileType=text/acmi/tacview");
telemetry.broadcastLine("FileVersion=2.2");
// ... 其他元数据

// 4. 循环发送状态
for (double t = 0; t <= 10; t += 0.1) {
    telemetry.broadcastLine("#" + std::to_string(t));           // 时间帧
    telemetry.broadcastLine("3EC,T=116.0|39.0|9000.0|0|0|0");  // 状态
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
```

---

## 重点代码行号速查

| 功能 | 文件 | 行号 |
|------|------|------|
| **握手协议** | | |
| 发送握手 | TacviewTelemetry.cpp | 173-203 |
| 可靠发送 | TacviewTelemetry.cpp | 211-226 |
| **数据广播** | | |
| 广播函数 | TacviewTelemetry.cpp | 240-268 |
| 移除客户端 | TacviewTelemetry.cpp | 230-236 |
| **ACMI格式** | | |
| 格式模板 | ACMI.cpp | 19-37 |
| 生成文件头 | ACMI.cpp | 189-212 |

---

## 调试技巧

### 启用日志查看发送的数据

```cpp
telemetry.enableLogging("debug.acmi");
```

生成的 `debug.acmi` 文件可以直接用 Tacview 打开，验证数据格式是否正确。

---

## 常见困惑

### Q1: 为什么我的数据 Tacview 不显示？

**检查清单**:
1. ✅ 握手发送了吗？
2. ✅ 文件头发送了吗？
3. ✅ 对象ID一致吗？（如都用 `3EC`）
4. ✅ 数值格式对吗？（经纬度6位小数）

### Q2: `broadcastLine()` 需要加锁吗？

**不需要**。函数内部已经处理了线程安全。

### Q3: ACMI 类和直接发字符串有什么区别？

ACMI 类是辅助工具，帮你格式化数据。你也可以直接拼接字符串发送：
```cpp
// 方式1: 使用 ACMI 类
acmi gen;
gen.createHeader();
telemetry.broadcastLine(gen.header);

// 方式2: 直接发送（更简单）
telemetry.broadcastLine("FileType=text/acmi/tacview\nFileVersion=2.2");
```

---

## 下一步

看完这个后：

1. 有问题 → 查 `TACVIEW通信接口说明.md`
2. 看详细注释 → 打开 `.h` 和 `.cpp` 文件
3. 调试问题 → 启用日志查看生成的 `.acmi` 文件

---

**时间估计**: 通读这份导读约 5-10 分钟，即可理解核心概念。
