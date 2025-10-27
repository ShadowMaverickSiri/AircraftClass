#pragma once
#ifndef AIRCRAFT_MODEL_LIBRARY_H
#define AIRCRAFT_MODEL_LIBRARY_H

#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <cmath>

#include <string>
#include "AircraftModule.h"

// Forward declaration for UnifiedManeuverModel
class UnifiedManeuverModel;
struct UnifiedManeuverParameters;

// 定义圆周率常量
namespace Constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;        // 度转弧度
    constexpr double RAD_TO_DEG = 180.0 / PI;        // 弧度转度
	constexpr double EARTH_RADIUS = 6378137.0;        // 地球半径 (米)
	constexpr double EARTH_RADIUS_SHORTER = 6356752.0; // 地球极半径 (米)
	constexpr double ECCE1 = 0.08181919084262149; // 地球第一偏心率
	constexpr double ECCE2 = 0.082094438151917; // 地球第二偏心率
	constexpr double E_EARTH = 1/298.257222101;  //地球椭球扁率
}

// 保持向后兼容性（逐步迁移）
#ifndef M_PI
#define M_PI Constants::PI
#endif

#ifndef Re
#define Re Constants::EARTH_RADIUS
#endif

#ifndef Rad
#define Rad Constants::RAD_TO_DEG
#endif

// 结构体：用经纬高表示位置
struct GeoPosition {
	double longitude;  // 经度 (度)
	double latitude;   // 纬度 (度)
	double altitude;   // 高度 (米，海平面以上)
};

// 结构体：用北-天-东坐标系表示速度
struct Velocity3 {
	double north; // 北向速度 (m/s)
	double up;    // 天向速度 (m/s)
	double east;  // 东向速度 (m/s)
};

// 结构体：包含姿态角信息
struct AttitudeAngles {
	double pitch;    // 俯仰角 (弧度) - 头向下偏转角度
	double roll;     // 滚转角 (弧度) - 机身向左偏转角度
	double yaw;      // 偏航角 (弧度) - 头向右偏转角度

	AttitudeAngles() : pitch(0.0), roll(0.0), yaw(0.0) {}

	// 转换为度数
	double getPitchDegrees() const { return pitch * Constants::RAD_TO_DEG; }
	double getRollDegrees() const { return roll * Constants::RAD_TO_DEG; }
	double getYawDegrees() const { return yaw * Constants::RAD_TO_DEG; }

	// 从度数设置
	void setPitchDegrees(double degrees) { pitch = degrees * Constants::DEG_TO_RAD; }
	void setRollDegrees(double degrees) { roll = degrees * Constants::DEG_TO_RAD; }
	void setYawDegrees(double degrees) { yaw = degrees * Constants::DEG_TO_RAD; }
};

// 结构体：包含飞机性能参数
struct AircraftPerformance {
	double maxTurnRate;      // 最大转弯率 (弧度/秒)	
	double maxClimbRate;     // 最大爬升率 (米/秒)
	double maxRollRate;      // 最大滚转率 (弧度/秒)
	double maxPitchRate;     // 最大俯仰率 (弧度/秒)	
	double maxThrust;        // 最大推力 (牛顿)
	double dragCoefficient;  // 阻力系数
	double wingArea;         // 机翼面积 (平方米)
	double mass;             // 飞机质量 (千克)
	
	// 新增过载相关参数
	double maxGForce;        // 最大过载 (G)
	double maxPositiveG;     // 最大正过载 (G)
	double maxNegativeG;     // 最大负过载 (G)
	double maxLateralG;      // 最大侧向过载 (G)
	double stallSpeed;       // 失速速度 (m/s)
	double maxSpeed;         // 最大速度 (m/s)

	AircraftPerformance() : maxTurnRate(0.5), maxClimbRate(50.0),
		maxRollRate(2.0), maxPitchRate(1.0),
		maxThrust(200000.0), dragCoefficient(0.02),
		wingArea(50.0), mass(10000.0),
		maxGForce(9.0), maxPositiveG(9.0), maxNegativeG(-3.0),
		maxLateralG(2.0), stallSpeed(80.0), maxSpeed(800.0) {}
};

//飞行器基类
class Aircraft {
public:
	using ManeuverFunc = std::function<void(Aircraft&, double)>;

	Aircraft(const std::string& type, const std::string& model);
	virtual ~Aircraft();

	// 获取飞机类型
	std::string getType() const { return type; }
	// 获取飞机型号
	std::string getModel() const { return model; }

	GeoPosition position;    // 位置
	Velocity3 velocity;        // 速度
	AttitudeAngles attitude; // 姿态
	AircraftPerformance performance; // 性能

	// 根据当前状态计算加速度，然后根据加速度更新位置
	virtual void updateKinematics(double dt);

	// 计算当前加速度 (m/s^2)
	virtual Velocity3 computeAcceleration() const = 0;

	// 更新姿态
	//virtual void updateAttitude(double dt);

	// 传统机动方法（保持向后兼容）
	void setManeuver(const std::string& name);         // 设置机动
	void performManeuver(double dt);                   // 执行当前机动
	static void registerManeuver(const std::string& name, ManeuverFunc func); // 注册机动

	// 新的统一机动模型方法
	void setManeuverModel(std::shared_ptr<UnifiedManeuverModel> model);  // 设置机动模型
	void initializeManeuver(const UnifiedManeuverParameters& params);    // 初始化机动
	void updateManeuver(double dt);                                      // 更新机动
	void resetManeuver();                                                // 重置机动状态

	// 获取当前机动状态
	const std::shared_ptr<UnifiedManeuverModel> getManeuverModel() const { return currentManeuverModel; }

	// 获取性能参数
	const AircraftPerformance& getPerformance() const { return performance; }

	// 坐标转换相关方法
	void setReferencePosition(const GeoPosition& refPos);  // 设置参考位置
	GeoPosition getReferencePosition() const { return referencePosition; }

	// 获取地球坐标系位置（简化版本，不依赖Eigen）
	Velocity3 getECEFPosition() const;

	// 获取相对于参考点的当地地理系位置
	Velocity3 getLocalNUEPosition() const;

	// 获取相对于参考点的距离和方位角
	double getDistanceFromReference() const;
	double getBearingFromReference() const;

	// 功能模块管理接口
	void addModule(std::shared_ptr<AircraftModule> module) {
		modules.push_back(module);
	}
	template<typename T>
	std::shared_ptr<T> getModule() const {
		for (const auto& m : modules) {
			auto casted = std::dynamic_pointer_cast<T>(m);
			if (casted) return casted;
		}
		return nullptr;
	}
	void updateModules(double dt) {
		for (auto& m : modules) {
			m->update(*this, dt);
		}
	}

protected:
	std::string type;   // 飞机类型（如fighter, passenger, uav等）
	std::string model;  // 飞机型号（如F-15, Su-27等）
	ManeuverFunc currentManeuver;
	static std::map<std::string, ManeuverFunc> maneuvers; // 机动函数映射

	// 新的统一机动模型成员
	std::shared_ptr<UnifiedManeuverModel> currentManeuverModel;

	// 坐标转换相关成员
	GeoPosition referencePosition;  // 参考位置（用于计算相对位置）
	std::vector<std::shared_ptr<AircraftModule>> modules;
};

// 根据速度和时间步长更新位置
GeoPosition updateGeoPosition(const GeoPosition& pos, const Velocity3& velocity, double dt);

#endif // AIRCRAFT_MODEL_LIBRARY_H
