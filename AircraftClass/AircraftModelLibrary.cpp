#include "AircraftModelLibrary.h"
#include <cmath>
#include <stdexcept>
#include <sstream>

// 静态成员初始化
std::map<std::string, Aircraft::ManeuverFunc> Aircraft::maneuvers;

Aircraft::Aircraft(const std::string& type, const std::string& model)
    : type(type), model(model),
      position{0.0, 0.0, 0.0},
      velocity{0.0, 0.0, 0.0},
      attitude(),
      performance(),
      currentManeuver(nullptr) {}

Aircraft::~Aircraft() {}

void Aircraft::updateKinematics(double dt) {
    Velocity3 a = computeAcceleration();
    velocity.north += a.north * dt;
    velocity.up += a.up * dt;
    velocity.east += a.east * dt;
    position = updateGeoPosition(position, velocity, dt);
}

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

void Aircraft::setReferencePosition(const GeoPosition& refPos) {
    referencePosition = refPos;
}

Velocity3 Aircraft::getECEFPosition() const {
    const double a = 6378137.0;  // WGS84 长半轴
    const double e2 = 0.006694379990141316;  // 第一偏心率平方

    double lat = position.latitude * Constants::DEG_TO_RAD;
    double lon = position.longitude * Constants::DEG_TO_RAD;
    double h = position.altitude;

    double N = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));

    Velocity3 ecef;
    ecef.north = (N + h) * std::cos(lat) * std::cos(lon);
    ecef.up = (N + h) * std::cos(lat) * std::sin(lon);
    ecef.east = (N * (1 - e2) + h) * std::sin(lat);

    return ecef;
}

Velocity3 Aircraft::getLocalNUEPosition() const {
    Velocity3 refECEF = getECEFPosition();

    const double a = 6378137.0;
    const double e2 = 0.006694379990141316;

    double refLat = referencePosition.latitude * Constants::DEG_TO_RAD;
    double refLon = referencePosition.longitude * Constants::DEG_TO_RAD;
    double refH = referencePosition.altitude;

    double refN = a / std::sqrt(1 - e2 * std::sin(refLat) * std::sin(refLat));

    Velocity3 refECEFPos;
    refECEFPos.north = (refN + refH) * std::cos(refLat) * std::cos(refLon);
    refECEFPos.up = (refN + refH) * std::cos(refLat) * std::sin(refLon);
    refECEFPos.east = (refN * (1 - e2) + refH) * std::sin(refLat);

    Velocity3 relativeECEF;
    relativeECEF.north = refECEF.north - refECEFPos.north;
    relativeECEF.up = refECEF.up - refECEFPos.up;
    relativeECEF.east = refECEF.east - refECEFPos.east;

    Velocity3 localNUE;
    double sinLat = std::sin(refLat);
    double cosLat = std::cos(refLat);
    double sinLon = std::sin(refLon);
    double cosLon = std::cos(refLon);

    localNUE.north = -sinLat * cosLon * relativeECEF.north - sinLat * sinLon * relativeECEF.up + cosLat * relativeECEF.east;
    localNUE.up = cosLat * cosLon * relativeECEF.north + cosLat * sinLon * relativeECEF.up + sinLat * relativeECEF.east;
    localNUE.east = -sinLon * relativeECEF.north + cosLon * relativeECEF.up;

    return localNUE;
}

double Aircraft::getDistanceFromReference() const {
    double dLat = (position.latitude - referencePosition.latitude) * Constants::DEG_TO_RAD;
    double dLon = (position.longitude - referencePosition.longitude) * Constants::DEG_TO_RAD;
    double dAlt = position.altitude - referencePosition.altitude;

    double lat1 = referencePosition.latitude * Constants::DEG_TO_RAD;
    double lat2 = position.latitude * Constants::DEG_TO_RAD;

    double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double horizontalDistance = Constants::EARTH_RADIUS * c;
    return std::sqrt(horizontalDistance * horizontalDistance + dAlt * dAlt);
}

double Aircraft::getBearingFromReference() const {
    double lat1 = referencePosition.latitude * Constants::DEG_TO_RAD;
    double lon1 = referencePosition.longitude * Constants::DEG_TO_RAD;
    double lat2 = position.latitude * Constants::DEG_TO_RAD;
    double lon2 = position.longitude * Constants::DEG_TO_RAD;

    double dLon = lon2 - lon1;

    double y = std::sin(dLon) * std::cos(lat2);
    double x = std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);

    return std::atan2(y, x) * Constants::RAD_TO_DEG;
}

GeoPosition updateGeoPosition(const GeoPosition& pos, const Velocity3& vel, double dt) {
    GeoPosition newPos = pos;

    double dNorth = vel.north * dt;
    double dEast = vel.east * dt;

    newPos.latitude += (dNorth / Constants::EARTH_RADIUS) * Constants::RAD_TO_DEG;

    double radiusAtLat = Constants::EARTH_RADIUS * std::cos(pos.latitude * Constants::DEG_TO_RAD);
    if (std::abs(radiusAtLat) > 1e-6)
        newPos.longitude += (dEast / radiusAtLat) * Constants::RAD_TO_DEG;

    newPos.altitude += vel.up * dt;

    return newPos;
}
