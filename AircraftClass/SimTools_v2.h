#pragma once

#include <vector>
#include <string>
#include <array>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <memory>
#include <functional>
#include <utility>

// ============================================================
// Eigen 库
// ============================================================

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace SimTools {
    using Vector3d = Eigen::Vector3d;
    using Matrix3d = Eigen::Matrix3d;
    using VectorXd = Eigen::VectorXd;
    using MatrixXd = Eigen::MatrixXd;

    // ============================================================
    // Quaternion 类
    // ============================================================
    class Quaternion {
        public:
            // 默认构造：单位四元数
            Quaternion() : q_(1.0, 0.0, 0.0, 0.0) {}

            // 从分量构造 (w, x, y, z)
            Quaternion(double w, double x, double y, double z) : q_(w, x, y, z) {}

            // 从 Eigen 四元数构造
            explicit Quaternion(const Eigen::Quaterniond& eigen_q) : q_(eigen_q) {}

            // 获取底层的 Eigen 四元数
            const Eigen::Quaterniond& GetEigen() const { return q_; }
            Eigen::Quaterniond& GetEigen() { return q_; }

            // ============================================================
            // 静态工厂方法
            // ============================================================
            static Quaternion Identity() {
                return Quaternion();
            }

            static Quaternion FromEuler(double roll, double pitch, double yaw) {
                Eigen::Quaterniond q =
                    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
                return Quaternion(q);
            }

            static Quaternion FromAxisAngle(const Vector3d& axis, double angle) {
                return Quaternion(Eigen::Quaterniond(
                    Eigen::AngleAxisd(angle, axis.normalized())
                ));
            }

            static Quaternion FromAxisAngle(const double axis[3], double angle) {
                return FromAxisAngle(Vector3d(axis[0], axis[1], axis[2]), angle);
            }

            static Quaternion FromRotationMatrix(const Matrix3d& R) {
                return Quaternion(Eigen::Quaterniond(R));
            }

            // ============================================================
            // 转换方法
            // ============================================================
            Vector3d ToEuler() const {
                Eigen::Vector3d euler = q_.toRotationMatrix().eulerAngles(2, 1, 0);
                return Vector3d(euler[2], euler[1], euler[0]); // roll, pitch, yaw
            }

            void ToEuler(double& roll, double& pitch, double& yaw) const {
                Vector3d euler = ToEuler();
                roll = euler[0];
                pitch = euler[1];
                yaw = euler[2];
            }

            Matrix3d ToRotationMatrix() const {
                return q_.toRotationMatrix();
            }

            void ToRotationMatrix(double matrix[3][3]) const {
                Matrix3d R = ToRotationMatrix();
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        matrix[i][j] = R(i, j);
                    }
                }
            }

            void ToAxisAngle(Vector3d& axis, double& angle) const {
                Eigen::AngleAxisd aa(q_);
                axis = Vector3d(aa.axis()[0], aa.axis()[1], aa.axis()[2]);
                angle = aa.angle();
            }

            void ToAxisAngle(double axis[3], double& angle) const {
                Vector3d ax;
                ToAxisAngle(ax, angle);
                axis[0] = ax[0];
                axis[1] = ax[1];
                axis[2] = ax[2];
            }

            // ============================================================
            // 运算方法
            // ============================================================
            Quaternion Normalized() const {
                return Quaternion(q_.normalized());
            }

            Quaternion Conjugate() const {
                return Quaternion(q_.conjugate());
            }

            Quaternion Inverse() const {
                return Quaternion(q_.inverse());
            }

            Quaternion operator*(const Quaternion& other) const {
                return Quaternion(q_ * other.q_);
            }

            Quaternion operator*(double scalar) const {
                return Quaternion(Eigen::Quaterniond(
                    q_.w() * scalar, q_.x() * scalar,
                    q_.y() * scalar, q_.z() * scalar
                ));
            }

            Quaternion operator+(const Quaternion& other) const {
                return Quaternion(Eigen::Quaterniond(
                    q_.w() + other.q_.w(), q_.x() + other.q_.x(),
                    q_.y() + other.q_.y(), q_.z() + other.q_.z()
                ));
            }

            Quaternion operator-(const Quaternion& other) const {
                return Quaternion(Eigen::Quaterniond(
                    q_.w() - other.q_.w(), q_.x() - other.q_.x(),
                    q_.y() - other.q_.y(), q_.z() - other.q_.z()
                ));
            }

            // ============================================================
            // 属性访问
            // ============================================================
            double W() const { return q_.w(); }
            double X() const { return q_.x(); }
            double Y() const { return q_.y(); }
            double Z() const { return q_.z(); }

            double& operator[](int i) {
                switch (i) {
                    case 0: return q_.w();
                    case 1: return q_.x();
                    case 2: return q_.y();
                    case 3: return q_.z();
                    default: return q_.w();
                }
            }

            const double& operator[](int i) const {
                switch (i) {
                    case 0: return q_.w();
                    case 1: return q_.x();
                    case 2: return q_.y();
                    case 3: return q_.z();
                    default: return q_.w();
                }
            }

            // ============================================================
            // 计算方法
            // ============================================================
            double Norm() const { return q_.norm(); }
            double SquaredNorm() const { return q_.squaredNorm(); }
            double Dot(const Quaternion& other) const { return q_.dot(other.q_); }
            bool IsUnit(double tol = 1e-6) const { return std::abs(SquaredNorm() - 1.0) < tol; }

            // ============================================================
            // 应用方法
            // ============================================================
            Vector3d Rotate(const Vector3d& v) const {
                Eigen::Vector3d result = q_ * v;
                return Vector3d(result[0], result[1], result[2]);
            }

            void RotateVector(double vx, double vy, double vz,
                             double& rx, double& ry, double& rz) const {
                Vector3d result = Rotate(Vector3d(vx, vy, vz));
                rx = result[0];
                ry = result[1];
                rz = result[2];
            }

            // ============================================================
            // 静态计算方法
            // ============================================================
            static Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, double t) {
                return Quaternion(q0.q_.slerp(t, q1.q_));
            }

            static Quaternion Exp(const Vector3d& v) {
                double norm = v.norm();
                if (norm < 1e-10) return Identity();
                double sin_norm = std::sin(norm);
                double cos_norm = std::cos(norm);
                double scale = sin_norm / norm;
                return Quaternion(cos_norm, v[0] * scale, v[1] * scale, v[2] * scale);
            }

            static Vector3d Log(const Quaternion& q) {
                Eigen::Quaterniond qn = q.q_.normalized();
                double w = qn.w();
                double w_clamped = std::max(-1.0, std::min(1.0, w));
                double theta = std::acos(w_clamped);
                if (std::abs(w) >= 1.0) return Vector3d::Zero();
                double sin_theta = std::sin(theta);
                double scale = theta / sin_theta;
                return Vector3d(qn.x() * scale, qn.y() * scale, qn.z() * scale);
            }

            // 输出
            friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
                os << "(" << q.W() << ", " << q.X() << ", " << q.Y() << ", " << q.Z() << ")";
                return os;
            }

        private:
            Eigen::Quaterniond q_;
        };

    // 类型别名
    using Quaterniond = Quaternion;
    using Vector4d = Quaternion;

    // 为 Eigen::Vector3d 添加友好的输出格式
    inline std::ostream& operator<<(std::ostream& os, const Eigen::Vector3d& v) {
        os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
        return os;
    }
}

// ============================================================
// 平台导出宏定义
// ============================================================

#ifdef SIMTOOLS_STATIC
    #define SIMTOOLS_API
#else
    #ifdef _WIN32
        #ifdef SIMTOOLS_EXPORTS
            #define SIMTOOLS_API __declspec(dllexport)
        #else
            #define SIMTOOLS_API __declspec(dllimport)
        #endif
    #else
        #define SIMTOOLS_API __attribute__((visibility("default")))
    #endif
#endif

// ============================================================
// 常量定义
// ============================================================

namespace SimTools {
    namespace Constants {
        // 数学常量
        constexpr double PI = 3.14159265358979323846;
        constexpr double RAD_TO_DEG = 180.0 / PI;
        constexpr double DEG_TO_RAD = PI / 180.0;

        // CGCS2000 坐标系参数
        constexpr double EARTH_SEMIMAJOR = 6378137.0;          // 长半轴 (m)
        constexpr double EARTH_SEMIMINOR = 6356752.3141;       // 短半轴 (m)
        constexpr double EARTH_FLATTENING = 1.0 / 298.257222101;
        constexpr double ECCENTRICITY_FIRST = 0.081819191042816;
        constexpr double ECCENTRICITY_SECOND = 0.082094438151917;

        // 计算精度
        constexpr double EPSILON = 1e-10;
        constexpr double ITERATION_TOL = 1e-10;

        // 物理常量
        constexpr double GRAVITY_SEA_LEVEL = 9.80665;         // m/s²
        constexpr double GAS_CONSTANT_AIR = 287.05287;        // J/(kg·K)
        constexpr double STANDARD_PRESSURE = 101325.0;        // Pa
        constexpr double STANDARD_TEMPERATURE = 288.15;       // K
        constexpr double STANDARD_DENSITY = 1.225;            // kg/m³
    }
}

// ============================================================
// 数学工具模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Math {
        // ===== 基础数学函数 =====

        // 符号函数：返回 x 的符号 (-1, 0, 1)
        template<typename T>
        static inline T Sign(T x) {
            return (T(0) < x) - (x < T(0));
        }

        // 最大值
        template<typename T>
        static inline T Max(T a, T b) {
            return (a >= b) ? a : b;
        }

        // 最小值
        template<typename T>
        static inline T Min(T a, T b) {
            return (a <= b) ? a : b;
        }

        // 限制值在范围内
        template<typename T>
        static inline T Clamp(T value, T min_val, T max_val) {
            return Max(min_val, Min(value, max_val));
        }

        // 查找绝对值最大值及其索引
        template<typename T>
        static T FindAbsMax(const std::vector<T>& data, int& index) {
            if (data.empty()) {
                index = -1;
                return T(0);
            }

            T maxVal = data[0];
            index = 0;
            for (size_t i = 1; i < data.size(); ++i) {
                if (std::abs(data[i]) > std::abs(maxVal)) {
                    maxVal = data[i];
                    index = static_cast<int>(i);
                }
            }
            return maxVal;
        }

        // 向量二范数
        static inline double Norm2(const Vector3d& vec) {
            return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
        }

        static inline double Norm2(const VectorXd& vec) {
            return vec.norm();
        }

        // 向量归一化
        static inline Vector3d Normalize(const Vector3d& vec) {
            double n = Norm2(vec);
            if (n < Constants::EPSILON) {
                return Vector3d::Zero();
            }
            return vec / n;
        }

        // 角度规范化到 [-180, 180]
        static inline double Regulate180(double angle) {
            double temp = angle - 360.0 * std::floor(angle / 360.0);
            if (std::abs(temp) > 180.0) {
                angle = temp - 360.0 * Sign(temp);
            } else {
                angle = temp;
            }
            return angle;
        }

        // 角度规范化到 [0, 360]
        static inline double Regulate360(double angle) {
            angle = angle - 360.0 * std::floor(angle / 360.0);
            if (angle < 0) {
                angle += 360.0;
            }
            return angle;
        }

        // 弧度规范化到 [-PI, PI]
        static inline double RegulatePi(double angle) {
            double temp = angle - 2 * Constants::PI * std::floor(angle / (2 * Constants::PI));
            if (std::abs(temp) > Constants::PI) {
                angle = temp - 2 * Constants::PI * Sign(temp);
            } else {
                angle = temp;
            }
            return angle;
        }
    };
}

// ============================================================
// 插值算法模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Interpolation {
        // ===== 一维插值 =====

        // 线性插值
        static double Linear(double x,
                            const std::vector<double>& xx,
                            const std::vector<double>& yy);

        // 拉格朗日插值（使用前后7个点）
        static double Lagrange7(double x,
                                const std::vector<double>& xx,
                                const std::vector<double>& yy);

        // 全局拉格朗日插值
        static double LagrangeGlobal(double x,
                                    const std::vector<double>& xx,
                                    const std::vector<double>& yy);

        // 三次样条插值
        static double CubicSpline(double x,
                                 const std::vector<double>& xx,
                                 const std::vector<double>& yy);

        // ===== 二维插值 =====

        // 双线性插值
        static double Bilinear(double u, double v,
                              const std::vector<double>& x,
                              const std::vector<double>& y,
                              const std::vector<std::vector<double>>& z);

        // ===== 查找插值索引 =====

        // 找到 x 在 xx 中的位置（返回大于 x 的第一个索引）
        static int FindIndex(double x, const std::vector<double>& xx);

        // 找到二维插值的索引
        static std::pair<int, int> FindIndex2D(double u, double v,
                                              const std::vector<double>& x,
                                              const std::vector<double>& y);
    };
}

// ============================================================
// 坐标转换模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Coordinate {
        // 类型定义
        using Vector3 = Vector3d;
        using Matrix3 = Matrix3d;

        // ===== 基础坐标系统 =====

        // GPS (经度°, 纬度°, 高度m) -> ECEF (X, Y, Z)m
        static Vector3 GpsToEcef(const Vector3& gps);

        // ECEF (X, Y, Z)m -> GPS (经度°, 纬度°, 高度m)
        static Vector3 EcefToGps(const Vector3& ecef);

        // ECEF -> GPS (使用牛顿迭代，精度更高)
        static Vector3 EcefToGpsNewton(const Vector3& ecef);

        // ===== 局部切平面坐标系 =====

        // 创建 ECEF 到 NED (北东地) 的旋转矩阵
        static Matrix3 EcefToNedMatrix(double longitude, double latitude);

        // 创建 NED 到 ECEF 的旋转矩阵
        static Matrix3 NedToEcefMatrix(double longitude, double latitude);

        // ECEF 位置 -> NED 位置（相对于参考点）
        static Vector3 EcefToNed(const Vector3& ecef_pos,
                                const Vector3& ref_gps);

        // NED 位置 -> ECEF 位置（相对于参考点）
        static Vector3 NedToEcef(const Vector3& ned_pos,
                                const Vector3& ref_gps);

        // ===== 速度转换 =====

        // 从弹道坐标系速度到 NED 速度
        // theta: 弹道倾斜角, phi_v: 弹道偏角, V: 速度
        static void VelocityToNed(double theta, double phi_v, double V,
                                 Vector3& vn);

        // NED 速度 -> ECEF 速度
        static void NedToEcefVelocity(const Vector3& ned_vel,
                                     const Vector3& gps_pos,
                                     Vector3& ecef_vel);

        // ===== 坐标旋转矩阵 =====

        // 创建基本旋转矩阵 (绕 X/Y/Z 轴)
        // axis: 1=X, 2=Y, 3=Z
        static Matrix3 RotationMatrix(double angle, int axis);

        // ===== 角度格式转换 =====

        // 度分秒 -> 十进制
        static inline double DmsToDecimal(double degree, double minute, double second) {
            return degree + minute / 60.0 + second / 3600.0;
        }

        // 十进制 -> 度分秒
        static inline void DecimalToDms(double decimal,
                                       int& degree, int& minute, double& second) {
            degree = static_cast<int>(decimal);
            minute = static_cast<int>((decimal - degree) * 3600.0 / 60.0);
            second = (decimal - degree - minute / 60.0) * 3600.0;
        }

        // 弧度 -> 角度
        static inline double RadToDeg(double rad) {
            return rad * Constants::RAD_TO_DEG;
        }

        // 角度 -> 弧度
        static inline double DegToRad(double deg) {
            return deg * Constants::DEG_TO_RAD;
        }
    };
}

// ============================================================
// 地理计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Geodesy {
        // ===== 距离计算 =====

        // 大圆距离（球面模型，快速但精度较低）
        static double GreatCircleDistance(double lon1, double lat1,
                                         double lon2, double lat2);

        // Haversine 距离（球面模型）
        static double HaversineDistance(double lon1, double lat1,
                                       double lon2, double lat2);

        // Vincenty 距离（椭球面模型，高精度）
        static double VincentyDistance(double lon1, double lat1,
                                      double lon2, double lat2);

        // ===== 方位角计算 =====

        // 计算初始方位角（从点1到点2）
        static double Azimuth(const Vector3d& gps1,
                             const Vector3d& gps2);

        // 计算两点间方位角和距离
        static void AzimuthAndDistance(const Vector3d& gps1,
                                      const Vector3d& gps2,
                                      double& azimuth,
                                      double& distance);

        // ===== Vincenty 正反解 =====

        // Vincenty 反解：已知两点坐标，求距离和方位角
        static void VincentyInverse(double lon1, double lat1,
                                   double lon2, double lat2,
                                   double& distance,
                                   double& azimuth1,
                                   double& azimuth2);

        // Vincenty 正解：已知起点、方位角和距离，求终点
        static void VincentyDirect(double lon1, double lat1,
                                 double azimuth, double distance,
                                 double& lon2, double& lat2,
                                 double& azimuth2);

        // ===== 站心计算 =====

        // 根据观测站坐标、方位角、高度和斜距计算目标GPS坐标
        static Vector3d TargetFromSite(const Vector3d& site_gps,
                                             double azimuth,
                                             double target_height,
                                             double slant_range);

        // 计算两GPS点间的站心方位角
        static double SiteAzimuth(const Vector3d& gpsA,
                                 const Vector3d& gpsB);

        // 计算两GPS点间的站心斜距
        static double SiteDistance(const Vector3d& gpsA,
                                  const Vector3d& gpsB);
    };
}

// ============================================================
// 大气参数模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Atmosphere {
        // 大气参数结构
        struct Parameters {
            double pressure;       // 气压 (Pa)
            double density;        // 空气密度 (kg/m³)
            double gravity;        // 重力加速度 (m/s²)
            double sound_speed;    // 声速 (m/s)
            double temperature;    // 温度 (K)
        };

        // 根据高度获取完整大气参数
        static Parameters GetParameters(double height_meters);

        // ===== 单项计算函数 =====

        // 重力加速度 (m/s²)
        static double Gravity(double height_meters);

        // 声速 (m/s)
        static double SoundSpeed(double height_meters);

        // 空气密度 (kg/m³)
        static double Density(double height_meters);

        // 气压 (Pa)
        static double Pressure(double height_meters);

        // 温度 (K)
        static double Temperature(double height_meters);

        // ===== 速度转换 =====

        // 根据马赫数和高度计算速度
        static double VelocityFromMach(double mach, double height_meters);

        // 根据速度和高度计算马赫数
        static double MachFromVelocity(double velocity, double height_meters);

        // ===== 动压计算 =====

        // 动压 q = 0.5 * rho * V²
        static double DynamicPressure(double velocity, double height_meters);
    };
}

// ============================================================
// 随机数生成模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Random {
        // 设置随机种子（默认使用当前时间）
        static void Seed(unsigned int seed = 0);

        // ===== 均匀分布 =====

        // [0, 1] 均匀分布
        static double Uniform01();

        // [a, b] 均匀分布
        static double Uniform(double a, double b);

        // 整数均匀分布 [a, b]
        static int UniformInt(int a, int b);

        // ===== 正态分布 =====

        // 标准正态分布 N(0, 1)
        static double Normal01();

        // 正态分布 N(mu, sigma²)
        static double Normal(double mu, double sigma);

        // ===== 其他分布 =====

        // 指数分布
        static double Exponential(double lambda);

        // 韦伯分布
        static double Weibull(double shape, double scale);
    };
}

// ============================================================
// 文件 I/O 模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API FileIO {
        // ===== 读取数据 =====

        // 读取文本文件到 Eigen 矩阵（自动处理空格和逗号分隔）
        static MatrixXd ReadMatrix(const std::string& filename);

        // 读取文本文件到二维 vector
        static std::vector<std::vector<double>> ReadTable(const std::string& filename);

        // 读取单列数据
        static std::vector<double> ReadColumn(const std::string& filename,
                                             int column_index = 0);

        // ===== 写入数据 =====

        // 将 Eigen 矩阵写入文件
        static bool WriteMatrix(const std::string& filename,
                               const MatrixXd& matrix,
                               bool scientific = false,
                               int precision = 6);

        // 将 vector 写入文件
        static bool WriteVector(const std::string& filename,
                               const std::vector<double>& data,
                               bool scientific = false,
                               int precision = 6);

        // ===== 文件信息 =====

        // 统计文件行数
        static int CountLines(const std::string& filename);

        // 统计文件列数（第一行）
        static int CountColumns(const std::string& filename);

        // ===== 辅助功能 =====

        // 数字转字符串
        static std::string ToString(int value);
        static std::string ToString(double value, int precision = 6);

        // 判断文件是否存在
        static bool Exists(const std::string& filename);
    };
}

// ============================================================
// 数值计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Numerical {
        // ===== 数值积分 =====

        // 龙格-库塔 4 阶积分
        // f: 微分方程函数 dy/dt = f(t, y)
        // t0: 初始时间, y0: 初始状态, h: 步长, steps: 积分步数
        using OdeFunction = std::function<VectorXd(double, const VectorXd&)>;

        static VectorXd RungeKutta4(const OdeFunction& f,
                                          double t0,
                                          const VectorXd& y0,
                                          double h,
                                          int steps);

        // 自适应步长龙格-库塔
        static VectorXd RungeKutta45(const OdeFunction& f,
                                           double t0,
                                           const VectorXd& y0,
                                           double t_end,
                                           double tolerance = 1e-6);

        // ===== 根查找 =====

        // 二分法求根
        static double Bisection(const std::function<double(double)>& f,
                               double a, double b,
                               double tol = 1e-10);

        // 牛顿法求根
        static double Newton(const std::function<double(double)>& f,
                           const std::function<double(double)>& df,
                           double x0,
                           double tol = 1e-10,
                           int max_iter = 100);

        // ===== 数值微分 =====

        // 中心差分求导数
        static double Derivative(const std::function<double(double)>& f,
                                double x,
                                double h = 1e-6);
    };
}

// ============================================================
// 几何计算模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Geometry {
        // 二维点结构
        struct Point2D {
            double x, y;
            Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
        };

        // ===== 点与多边形关系 =====

        // 判断点是否在三角形内
        static bool IsPointInTriangle(const Point2D& point,
                                     const Point2D& a,
                                     const Point2D& b,
                                     const Point2D& c);

        // 判断点是否在多边形内
        static bool IsPointInPolygon(const Point2D& point,
                                    const std::vector<Point2D>& polygon);

        // ===== 距离计算 =====

        // 点到线段的距离
        static double DistanceToLineSegment(const Point2D& point,
                                          const Point2D& line_start,
                                          const Point2D& line_end);

        // 两点间距离
        static inline double Distance(const Point2D& p1, const Point2D& p2) {
            return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
        }

        // ===== 三维几何 =====

        // 三点确定的平面法向量
        static Vector3d PlaneNormal(const Vector3d& p1,
                                          const Vector3d& p2,
                                          const Vector3d& p3);

        // 点到平面的距离
        static double DistanceToPlane(const Vector3d& point,
                                     const Vector3d& plane_point,
                                     const Vector3d& plane_normal);
    };
}

// ============================================================
// 时间和日期工具
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Time {
        // GPS 时间戳结构
        struct GpsTime {
            int week;          // GPS 周
            double seconds;    // 周内秒
        };

        // Unix 时间戳 (秒)
        static double GetUnixTimestamp();

        // Unix 时间 -> GPS 时间
        static GpsTime UnixToGpsTime(double unix_time);

        // GPS 时间 -> Unix 时间
        static double GpsTimeToUnix(const GpsTime& gps_time);

        // UTC -> GPS 时间秒
        static double UtcToGpsSeconds(int year, int month, int day,
                                     int hour, int minute, double second);

        // 格林威治恒星时
        static double GreenwhichSiderealTime(double mjd);
    };
}

// ============================================================
// 单位转换模块
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Units {
        // ===== 距离 =====
        static constexpr double M_TO_FT = 3.2808399;
        static constexpr double FT_TO_M = 1.0 / M_TO_FT;
        static constexpr double KM_TO_M = 1000.0;
        static constexpr double NM_TO_M = 1852.0;  // 海里

        // ===== 速度 =====
        static constexpr double KNOTS_TO_MS = 0.514444444;
        static constexpr double MS_TO_KNOTS = 1.0 / KNOTS_TO_MS;
        static constexpr double KMH_TO_MS = 0.277777778;
        static constexpr double MS_TO_KMH = 1.0 / KMH_TO_MS;

        // ===== 角度 =====
        static constexpr double DEG_TO_RAD = Constants::DEG_TO_RAD;
        static constexpr double RAD_TO_DEG = Constants::RAD_TO_DEG;
        static constexpr double MRAD_TO_RAD = 0.001;
        static constexpr double RAD_TO_MRAD = 1000.0;

        // ===== 质量 =====
        static constexpr double KG_TO_LB = 2.20462262;
        static constexpr double LB_TO_KG = 1.0 / KG_TO_LB;

        // ===== 力/推力 =====
        static constexpr double N_TO_LBF = 0.224808943;
        static constexpr double LBF_TO_N = 1.0 / N_TO_LBF;
        static constexpr double KN_TO_N = 1000.0;

        // ===== 压力 =====
        static constexpr double PA_TO_PSI = 0.000145037738;
        static constexpr double PSI_TO_PA = 1.0 / PA_TO_PSI;
        static constexpr double PA_TO_BAR = 1e-5;
        static constexpr double BAR_TO_PA = 1e5;
        static constexpr double ATM_TO_PA = 101325.0;

        // ===== 温度 =====
        static inline double CelsiusToKelvin(double c) { return c + 273.15; }
        static inline double KelvinToCelsius(double k) { return k - 273.15; }
        static inline double FahrenheitToKelvin(double f) {
            return (f - 32.0) * 5.0 / 9.0 + 273.15;
        }
    };
}

// ============================================================
// 矩阵运算工具（Eigen 扩展）
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API MatrixUtils {
        // 矩阵乘法（C风格数组接口）
        static void Multiply(const double A[3][3], const double B[3], double C[3]);
        static void Multiply3x3(const double A[3][3], const double B[3][3], double C[3][3]);

        // 矩阵转置
        static void Transpose(const double A[3][3], double B[3][3]);

        // 向量外积
        static Matrix3d OuterProduct(const Vector3d& a,
                                           const Vector3d& b);

        // 斜对称矩阵（用于叉乘）
        static Matrix3d SkewSymmetric(const Vector3d& v);

        // ===== 四元数与旋转矩阵转换 =====

        // 四元数 -> 旋转矩阵
        static Matrix3d QuaternionToMatrix(const Quaterniond& q);

        // 旋转矩阵 -> 四元数
        static Quaterniond MatrixToQuaternion(const Matrix3d& R);

        // ===== 欧拉角转换 =====

        // 旋转矩阵 -> 欧拉角
        static Vector3d MatrixToEuler(const Matrix3d& R);

        // 欧拉角 -> 旋转矩阵
        static Matrix3d EulerToMatrix(double roll, double pitch, double yaw);

        // 欧拉角 -> 四元数
        static Quaterniond EulerToQuaternion(double roll, double pitch, double yaw);

        // 四元数 -> 欧拉角
        static Vector3d QuaternionToEuler(const Quaterniond& q);

        // ===== 轴角转换 =====

        // 轴角 -> 四元数
        static Quaterniond AxisAngleToQuaternion(const Vector3d& axis, double angle);

        // 四元数 -> 轴角
        static void QuaternionToAxisAngle(const Quaterniond& q, Vector3d& axis, double& angle);

        // ===== 四元数运算 =====

        // 四元数球面线性插值 (SLERP)
        // t: 插值参数 [0, 1], t=0返回q0, t=1返回q1
        static Quaterniond Slerp(const Quaterniond& q0, const Quaterniond& q1, double t);

        // 用四元数旋转向量
        static Vector3d RotateVector(const Quaterniond& q, const Vector3d& v);

        // 四元数指数映射 (用于姿态微积分)
        static Quaterniond QuaternionExp(const Vector3d& v);

        // 四元数对数映射 (用于姿态微积分)
        static Vector3d QuaternionLog(const Quaterniond& q);
    };
}

// ============================================================
// 仿真实用工具
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Simulation {
        // 性能计时器
        class Timer {
        public:
            Timer();
            ~Timer();
            void Start();
            void Stop();
            double ElapsedSeconds() const;
            double ElapsedMilliseconds() const;
        private:
            class Impl;
            std::unique_ptr<Impl> impl_;
        };

        // 日志级别
        enum class LogLevel {
            Debug,
            Info,
            Warning,
            Error,
            Fatal
        };

        // 简单日志功能
        class Logger {
        public:
            static void Log(LogLevel level, const std::string& message);
            static void Debug(const std::string& message);
            static void Info(const std::string& message);
            static void Warning(const std::string& message);
            static void Error(const std::string& message);

            static void SetLogLevel(LogLevel level);
            static void SetOutputFile(const std::string& filename);
        };
    };
}

// ============================================================
// 版本信息
// ============================================================

namespace SimTools {
    struct SIMTOOLS_API Version {
        static constexpr int MAJOR = 2;
        static constexpr int MINOR = 0;
        static constexpr int PATCH = 0;

        static std::string ToString() {
            return std::to_string(MAJOR) + "." +
                   std::to_string(MINOR) + "." +
                   std::to_string(PATCH);
        }
    };
}
