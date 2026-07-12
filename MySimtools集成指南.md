# MySimtools 静态库集成指南

本文档说明如何在 AircraftClass 项目中使用 MySimtools 静态库。

## 目录结构

```
D:\mycode\
├── MySimtools\              # MySimtools 静态库项目
│   ├── SimTools_v2.h        # 主头文件
│   ├── SimTools_*.cpp       # 源文件
│   └── build\               # 编译输出目录
│       ├── Debug\           # Debug 版本
│       │   └── SimTools_static.lib
│       └── Release\         # Release 版本
│           └── SimTools_static.lib
│
└── AircraftClass\           # AircraftClass 项目
    └── AircraftClass\
        ├── SimToolsExample.cpp  # 使用示例（新增）
        └── AircraftClass.vcxproj # 已配置好
```

## 已完成的配置

项目配置文件 `AircraftClass.vcxproj` 已经更新，包含以下配置：

### 1. 头文件路径
```xml
<IncludePath>...;D:\mycode\MySimtools</IncludePath>
```

### 2. 库文件路径
```xml
<LibraryPath>...;D:\mycode\MySimtools\build\Debug</LibraryPath>
```

### 3. 链接库
```xml
<AdditionalDependencies>SimTools_static.lib;...</AdditionalDependencies>
```

### 4. 预处理器定义
```xml
<PreprocessorDefinitions>SIMTOOLS_STATIC;USE_EIGEN;...</PreprocessorDefinitions>
```

## 使用方法

### 步骤1：包含头文件

在需要使用 MySimtools 的源文件中包含主头文件：

```cpp
#include "SimTools_v2.h"
```

### 步骤2：使用命名空间

```cpp
using namespace SimTools;
```

或者使用完整的命名空间前缀：

```cpp
double result = SimTools::Math::Max(1.0, 2.0);
```

## 主要模块

### 1. 数学工具 (Math)

```cpp
// 符号函数
double sign = Math::Sign(-5.5);  // 返回 -1

// 最大值/最小值
double max_val = Math::Max(3.0, 7.0);   // 返回 7.0
double min_val = Math::Min(3.0, 7.0);   // 返回 3.0

// 向量范数
Vector3d vec(3.0, 4.0, 0.0);
double norm = Math::Norm2(vec);  // 返回 5.0

// 向量归一化
Vector3d normalized = Math::Normalize(vec);

// 角度规范化
double angle = Math::Regulate180(370.0);  // 返回 10.0
```

### 2. 插值算法 (Interpolation)

```cpp
std::vector<double> x = {0.0, 1.0, 2.0, 3.0};
std::vector<double> y = {0.0, 1.0, 4.0, 9.0};

// 线性插值
double y1 = Interpolation::Linear(1.5, x, y);

// 拉格朗日插值（7点）
double y2 = Interpolation::Lagrange7(1.5, x, y);

// 全局拉格朗日插值
double y3 = Interpolation::LagrangeGlobal(1.5, x, y);
```

### 3. 坐标转换 (Coordinate)

```cpp
// GPS (经度°, 纬度°, 高度m) -> ECEF (X, Y, Z)m
Vector3d gps(116.4074, 39.9042, 100.0);
Vector3d ecef = Coordinate::GpsToEcef(gps);

// ECEF -> GPS
Vector3d gps_back = Coordinate::EcefToGps(ecef);

// ECEF -> NED (相对于参考点)
Vector3d ref_gps(116.0, 40.0, 0.0);
Vector3d ned = Coordinate::EcefToNed(ecef, ref_gps);

// 旋转矩阵
Matrix3d rot = Coordinate::RotationMatrix(angle, axis);  // axis: 1=X, 2=Y, 3=Z
```

### 4. 地理计算 (Geodesy)

```cpp
// 计算两点间距离（高精度）
double dist = Geodesy::VincentyDistance(lon1, lat1, lon2, lat2);

// 大圆距离（快速）
double gc_dist = Geodesy::GreatCircleDistance(lon1, lat1, lon2, lat2);

// 计算方位角和距离
double azimuth, distance;
Geodesy::AzimuthAndDistance(gps1, gps2, azimuth, distance);

// Vincenty 反解
double az1, az2;
Geodesy::VincentyInverse(lon1, lat1, lon2, lat2, distance, az1, az2);
```

### 5. 大气参数 (Atmosphere)

```cpp
// 获取完整大气参数
auto params = Atmosphere::GetParameters(height);
std::cout << "Pressure: " << params.pressure << " Pa" << std::endl;
std::cout << "Density: " << params.density << " kg/m³" << std::endl;
std::cout << "Sound Speed: " << params.sound_speed << " m/s" << std::endl;

// 单独获取各项参数
double g = Atmosphere::Gravity(height);
double rho = Atmosphere::Density(height);
double sound_speed = Atmosphere::SoundSpeed(height);
double pressure = Atmosphere::Pressure(height);

// 马赫数转换
double velocity = Atmosphere::VelocityFromMach(mach, height);
double mach = Atmosphere::MachFromVelocity(velocity, height);

// 动压计算
double q = Atmosphere::DynamicPressure(velocity, height);
```

### 6. 随机数生成 (Random)

```cpp
// 设置种子
Random::Seed(42);

// [0, 1] 均匀分布
double u01 = Random::Uniform01();

// [a, b] 均匀分布
double u_ab = Random::Uniform(10.0, 20.0);

// 标准正态分布 N(0, 1)
double n01 = Random::Normal01();

// 正态分布 N(mu, sigma²)
double n = Random::Normal(100.0, 5.0);
```

### 7. 文件I/O (FileIO)

```cpp
// 统计文件行数
int lines = FileIO::CountLines("data.txt");

// 读取二维数据
auto data = FileIO::ReadTable("data.txt");

// 读取单列数据
auto column = FileIO::ReadColumn("data.txt", 0);

// 写入数据
FileIO::WriteVector("output.txt", data_vector);

// 数字转字符串
std::string str = FileIO::ToString(3.14159, 4);  // "3.1416"
```

### 8. 数值计算 (Numerical)

```cpp
// 定义微分方程函数
using OdeFunction = std::function<VectorXd(double, const VectorXd&)>;

OdeFunction f = [](double t, const VectorXd& y) -> VectorXd {
    VectorXd dy(y.size());
    dy(0) = -y(0);  // dy/dt = -y
    return dy;
};

// 龙格-库塔4阶积分
VectorXd y0(1);
y0(0) = 1.0;
VectorXd y_end = Numerical::RungeKutta4(f, 0.0, y0, 0.1, 10);

// 自适应步长龙格-库塔
VectorXd y_end2 = Numerical::RungeKutta45(f, 0.0, y0, 1.0, 1e-6);
```

### 9. 几何计算 (Geometry)

```cpp
// 判断点是否在三角形内
Geometry::Point2D point(1.0, 1.0);
Geometry::Point2D a(0.0, 0.0), b(2.0, 0.0), c(0.0, 2.0);
bool inside = Geometry::IsPointInTriangle(point, a, b, c);

// 两点间距离
double dist = Geometry::Distance(a, b);

// 点到线段距离
double dist_to_line = Geometry::DistanceToLineSegment(point, line_start, line_end);

// 平面法向量
Vector3d normal = Geometry::PlaneNormal(p1, p2, p3);

// 点到平面距离
double dist_to_plane = Geometry::DistanceToPlane(point, plane_point, normal);
```

### 10. 矩阵工具 (MatrixUtils)

```cpp
// C风格数组矩阵乘法
double A[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
double b[3] = {1, 2, 3};
double c[3];
MatrixUtils::Multiply(A, b, c);

// 矩阵转置
MatrixUtils::Transpose(A, B);

// 向量外积
Matrix3d outer = MatrixUtils::OuterProduct(v1, v2);

// 斜对称矩阵（叉乘矩阵）
Matrix3d skew = MatrixUtils::SkewSymmetric(v);

// 四元数 -> 旋转矩阵
Matrix3d R = MatrixUtils::QuaternionToMatrix(q);
```

## 在 AircraftClass 中的实际应用示例

### 示例1：计算动压

```cpp
#include "SimTools_v2.h"

double CalculateDynamicPressure(double velocity, double altitude) {
    double rho = SimTools::Atmosphere::Density(altitude);
    return 0.5 * rho * velocity * velocity;
}
```

### 示例2：GPS坐标转换

```cpp
#include "SimTools_v2.h"

// 获取飞机相对于机场的位置
Vector3d GetRelativePosition(Vector3d aircraft_gps, Vector3d airport_gps) {
    // 转换为ECEF坐标
    Vector3d ecef = SimTools::Coordinate::GpsToEcef(aircraft_gps);

    // 转换为相对于机场的NED坐标
    return SimTools::Coordinate::EcefToNed(ecef, airport_gps);
}
```

### 示例3：计算马赫数

```cpp
#include "SimTools_v2.h"

double CalculateMach(double velocity, double altitude) {
    double sound_speed = SimTools::Atmosphere::SoundSpeed(altitude);
    return velocity / sound_speed;
}
```

### 示例4：计算两点间距离

```cpp
#include "SimTools_v2.h"

double CalculateDistance(Vector3d gps1, Vector3d gps2) {
    return SimTools::Geodesy::VincentyDistance(
        gps1(0), gps1(1),  // longitude, latitude
        gps2(0), gps2(1)
    );
}
```

## 编译说明

### 首次编译

1. 确保 MySimtools 已经编译：
   ```bash
   cd D:\mycode\MySimtools\build
   cmake ..
   cmake --build . --config Debug
   ```

2. 在 Visual Studio 中打开 `AircraftClass.sln`

3. 选择 **x64/Debug** 配置

4. 生成解决方案

### 注意事项

1. **Eigen3 库**：如果使用 Eigen 功能，需要设置 `EIGEN3_ROOT` 环境变量或在项目属性中添加 Eigen 路径

2. **字符编码**：MySimtools 使用 UTF-8 编码，确保项目设置为 `/utf-8` 编码选项

3. **运行时库**：确保 MySimtools 和 AircraftClass 使用相同的运行时库（MD/MT）

## 常见问题

### Q1: 链接错误 "无法解析的外部符号"

**解决方案**：
- 确认 MySimtools 已编译
- 检查库路径是否正确
- 确认 Debug/Release 配置匹配

### Q2: 找不到头文件

**解决方案**：
- 检查附加包含目录是否包含 `D:\mycode\MySimtools`
- 确认 `SimTools_v2.h` 文件存在

### Q3: Eigen 相关错误

**解决方案**：
- 设置 `EIGEN3_ROOT` 环境变量
- 或在项目属性中添加 Eigen 包含路径
- 如果不使用 Eigen，移除 `USE_EIGEN` 预处理器定义

## 更多信息

- MySimtools API 文档：`D:\mycode\MySimtools\API完整参考.md`
- MySimtools 快速入门：`D:\mycode\MySimtools\快速入门指南.md`
- 示例代码：`D:\mycode\AircraftClass\AircraftClass\AircraftClass\SimToolsExample.cpp`
