#include "AircraftModelLibrary.h"
#include"EulerAngleCalculator.h"
#include "UnifiedManeuverSystem.h"
#include <cmath>
#include <stdexcept>
#include <sstream>

// 地球半径 (单位：米)
const double EARTH_RADIUS = 6371000.0;

// 静态成员初始化
std::map<std::string, Aircraft::ManeuverFunc> Aircraft::maneuvers;

Aircraft::Aircraft(const std::string& type, const std::string& model)
	: type(type), model(model),
	position{ 0.0, 0.0, 0.0 },
	velocity{ 0.0, 0.0, 0.0 },
	attitude(),
	performance(),
	currentManeuver(nullptr),
	currentManeuverModel(nullptr) {}

Aircraft::~Aircraft() {}

void Aircraft::updateKinematics(double dt) {
	Velocity3 a = computeAcceleration();

	// 速度更新
	velocity.north += a.north * dt;
	velocity.up += a.up * dt;
	velocity.east += a.east * dt;

	// 根据当前速度更新位置
	position = updateGeoPosition(position, velocity, dt);
}

// 注册操纵动作
void Aircraft::registerManeuver(const std::string& name, ManeuverFunc func) {
	maneuvers[name] = func;
}

void Aircraft::setManeuver(const std::string& name) {
	auto it = maneuvers.find(name);
	if (it == maneuvers.end()) {
		std::ostringstream oss;
		oss << "Unknown maneuver: " << name << "\nAvailable maneuvers: ";
		for (const auto& pair : maneuvers)
			oss << pair.first << " ";
		throw std::invalid_argument(oss.str());
	}
	currentManeuver = it->second;
}

void Aircraft::performManeuver(double dt) {
	if (currentManeuver)
		currentManeuver(*this, dt);
}

// 新的统一操纵模型设置
void Aircraft::setManeuverModel(std::shared_ptr<UnifiedManeuverModel> model) {
	currentManeuverModel = model;
}

void Aircraft::initializeManeuver(const UnifiedManeuverParameters& params) {
	if (currentManeuverModel) {
		currentManeuverModel->initialize(params);
	}
}

void Aircraft::updateManeuver(double dt) {
	if (currentManeuverModel) {
		currentManeuverModel->update(*this, dt);
	}
}

void Aircraft::resetManeuver() {
	if (currentManeuverModel) {
		currentManeuverModel->reset();
	}
}

//void Aircraft::updateAttitude(double dt) {
//	// 根据速度更新姿态
//	attitude = EulerAngleCalculator::calculateFromVelocity(velocity);
//}

// 设置参考位置
void Aircraft::setReferencePosition(const GeoPosition & refPos) {
	referencePosition = refPos;
}

Velocity3 Aircraft::getECEFPosition() const {
	// 计算ECEF坐标（使用Eigen）
	const double EARTH_RADIUS = 6371000.0;
	const double PI = 3.14159265358979323846;

	double lat = position.latitude * PI / 180.0;  // 转换为弧度
	double lon = position.longitude * PI / 180.0;
	double h = position.altitude;

	// 使用WGS84椭球体计算
	const double a = 6378137.0;  // 长半轴
	const double e2 = 0.006694379990141316;  // 第一偏心率平方

	double N = a / sqrt(1 - e2 * sin(lat) * sin(lat));  // 卯酉圈曲率半径

	Velocity3 ecef;
	ecef.north = (N + h) * cos(lat) * cos(lon);  // X轴
	ecef.up = (N + h) * cos(lat) * sin(lon);     // Y轴
	ecef.east = (N * (1 - e2) + h) * sin(lat);   // Z轴

	return ecef;
}

Velocity3 Aircraft::getLocalNUEPosition() const {
	// 计算本地NUE坐标系下的位置
	Velocity3 refECEF = getECEFPosition();  // 当前飞机的ECEF位置

	// 参考ECEF位置
	const double EARTH_RADIUS = 6371000.0;
	const double PI = 3.14159265358979323846;

	double refLat = referencePosition.latitude * PI / 180.0;
	double refLon = referencePosition.longitude * PI / 180.0;
	double refH = referencePosition.altitude;

	const double a = 6378137.0;
	const double e2 = 0.006694379990141316;

	double refN = a / sqrt(1 - e2 * sin(refLat) * sin(refLat));

	Velocity3 refECEFPos;
	refECEFPos.north = (refN + refH) * cos(refLat) * cos(refLon);
	refECEFPos.up = (refN + refH) * cos(refLat) * sin(refLon);
	refECEFPos.east = (refN * (1 - e2) + refH) * sin(refLat);

	// 相对位置
	Velocity3 relativeECEF;
	relativeECEF.north = refECEF.north - refECEFPos.north;
	relativeECEF.up = refECEF.up - refECEFPos.up;
	relativeECEF.east = refECEF.east - refECEFPos.east;

	// 转换为NUE坐标系
	Velocity3 localNUE;

	// 计算旋转矩阵
	double sinLat = sin(refLat);
	double cosLat = cos(refLat);
	double sinLon = sin(refLon);
	double cosLon = cos(refLon);

	// 应用旋转
	localNUE.north = -sinLat * cosLon * relativeECEF.north - sinLat * sinLon * relativeECEF.up + cosLat * relativeECEF.east;
	localNUE.up = cosLat * cosLon * relativeECEF.north + cosLat * sinLon * relativeECEF.up + sinLat * relativeECEF.east;
	localNUE.east = -sinLon * relativeECEF.north + cosLon * relativeECEF.up;

	return localNUE;
}

double Aircraft::getDistanceFromReference() const {
	// 计算到参考点的距离
	double dLat = position.latitude - referencePosition.latitude;
	double dLon = position.longitude - referencePosition.longitude;
	double dAlt = position.altitude - referencePosition.altitude;

	// 转换为弧度
	const double PI = 3.14159265358979323846;
	dLat *= PI / 180.0;
	dLon *= PI / 180.0;

	// 使用Haversine公式计算水平距离
	double lat1 = referencePosition.latitude * PI / 180.0;
	double lat2 = position.latitude * PI / 180.0;

	double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));

	const double EARTH_RADIUS = 6371000.0;
	double horizontalDistance = EARTH_RADIUS * c;

	// 总距离（水平距离和高度）
	return sqrt(horizontalDistance * horizontalDistance + dAlt * dAlt);
}

double Aircraft::getBearingFromReference() const {
	// 计算从参考点到当前位置的方位角
	double lat1 = referencePosition.latitude * 3.14159265358979323846 / 180.0;
	double lon1 = referencePosition.longitude * 3.14159265358979323846 / 180.0;
	double lat2 = position.latitude * 3.14159265358979323846 / 180.0;
	double lon2 = position.longitude * 3.14159265358979323846 / 180.0;

	double dLon = lon2 - lon1;

	double y = sin(dLon) * cos(lat2);
	double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

	double bearing = atan2(y, x);
	return bearing * 180.0 / 3.14159265358979323846;  // 转换为度
}

// 更新地理位置
GeoPosition updateGeoPosition(const GeoPosition & pos, const Velocity3 & vel, double dt) {
	GeoPosition newPos = pos;

	double dNorth = vel.north * dt;
	double dEast = vel.east * dt;

	// 纬度变化 = 距离 / 地球半径
	newPos.latitude += (dNorth / EARTH_RADIUS) * (180.0 / M_PI);

	// 经度变化 = 距离 / (地球半径 * cos(纬度))
	double radiusAtLat = EARTH_RADIUS * std::cos(pos.latitude * M_PI / 180.0);
	if (std::abs(radiusAtLat) > 1e-6)
		newPos.longitude += (dEast / radiusAtLat) * (180.0 / M_PI);

	// 高度变化
	newPos.altitude += vel.up * dt;

	return newPos;
}
