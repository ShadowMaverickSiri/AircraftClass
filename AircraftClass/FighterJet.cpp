#include <string>
#include "FighterJet.h"
#include <cmath>

// 构造函数
FighterJet::FighterJet(const std::string& modelName, const std::string& type)
    : Aircraft(modelName, type)
{
    fighterType = determineFighterType(modelName);
    initializeAerodynamics(fighterType);
}

FighterJet::FighterJet(FighterType type, const GeoPosition& pos, const Velocity3& vel, const AttitudeAngles& att)
    : Aircraft("Fighter", "Jet"), fighterType(type)
{
    position = pos;
    velocity = vel;
    attitude = att;
    initializeAerodynamics(type);
}

// 根据型号名称确定战斗机类型
FighterType FighterJet::determineFighterType(const std::string& modelName) const {
    if (modelName.find("F-15") != std::string::npos) return ::FighterType::F15_EAGLE;
    if (modelName.find("F-16") != std::string::npos) return ::FighterType::F16_FALCON;
    if (modelName.find("Su-27") != std::string::npos || modelName.find("SU-27") != std::string::npos) return ::FighterType::SU27_FLANKER;
    if (modelName.find("MiG-29") != std::string::npos || modelName.find("MIG-29") != std::string::npos) return ::FighterType::MIG29_FULCRUM;
    if (modelName.find("F-22") != std::string::npos) return ::FighterType::F22_RAPTOR;
    if (modelName.find("F-35") != std::string::npos) return ::FighterType::F35_LIGHTNING;
    
    // 默认返回F-16
    return ::FighterType::F16_FALCON;
}

// 初始化战斗机气动参数
void FighterJet::initializeAerodynamics(FighterType type) {
    switch (type) {
        case FighterType::F15_EAGLE:
            aerodynamics.CL0 = 0.15;           // F-15有较大的零攻角升力
            aerodynamics.CLalpha = 5.2;        // 升力曲线斜率
            aerodynamics.CD0 = 0.020;          // 零升阻力系数
            aerodynamics.K = 0.085;            // 诱导阻力因子
            aerodynamics.maxAOA = 15.0 * M_PI / 180.0;  // 15度
            aerodynamics.minAOA = -5.0 * M_PI / 180.0;  // -5度
            aerodynamics.stallAOA = 18.0 * M_PI / 180.0; // 18度
            break;
            
        case FighterType::F16_FALCON:
            aerodynamics.CL0 = 0.12;           // F-16标准值
            aerodynamics.CLalpha = 5.5;        // 升力曲线斜率
            aerodynamics.CD0 = 0.018;          // 零升阻力系数
            aerodynamics.K = 0.080;            // 诱导阻力因子
            aerodynamics.maxAOA = 14.0 * M_PI / 180.0;  // 14度
            aerodynamics.minAOA = -4.0 * M_PI / 180.0;  // -4度
            aerodynamics.stallAOA = 16.0 * M_PI / 180.0; // 16度
            break;
            
        case FighterType::SU27_FLANKER:
            aerodynamics.CL0 = 0.18;           // Su-27有较大的零攻角升力
            aerodynamics.CLalpha = 5.8;        // 升力曲线斜率
            aerodynamics.CD0 = 0.022;          // 零升阻力系数
            aerodynamics.K = 0.090;            // 诱导阻力因子
            aerodynamics.maxAOA = 16.0 * M_PI / 180.0;  // 16度
            aerodynamics.minAOA = -6.0 * M_PI / 180.0;  // -6度
            aerodynamics.stallAOA = 20.0 * M_PI / 180.0; // 20度
            break;
            
        case FighterType::MIG29_FULCRUM:
            aerodynamics.CL0 = 0.14;           // MiG-29
            aerodynamics.CLalpha = 5.6;        // 升力曲线斜率
            aerodynamics.CD0 = 0.021;          // 零升阻力系数
            aerodynamics.K = 0.088;            // 诱导阻力因子
            aerodynamics.maxAOA = 15.0 * M_PI / 180.0;  // 15度
            aerodynamics.minAOA = -5.0 * M_PI / 180.0;  // -5度
            aerodynamics.stallAOA = 18.0 * M_PI / 180.0; // 18度
            break;
            
        case FighterType::F22_RAPTOR:
            aerodynamics.CL0 = 0.10;           // F-22隐身设计
            aerodynamics.CLalpha = 6.0;        // 高升力曲线斜率
            aerodynamics.CD0 = 0.015;          // 低阻力系数
            aerodynamics.K = 0.075;            // 低诱导阻力
            aerodynamics.maxAOA = 17.0 * M_PI / 180.0;  // 17度
            aerodynamics.minAOA = -7.0 * M_PI / 180.0;  // -7度
            aerodynamics.stallAOA = 22.0 * M_PI / 180.0; // 22度
            break;
            
        case FighterType::F35_LIGHTNING:
            aerodynamics.CL0 = 0.11;           // F-35
            aerodynamics.CLalpha = 5.7;        // 升力曲线斜率
            aerodynamics.CD0 = 0.017;          // 零升阻力系数
            aerodynamics.K = 0.078;            // 诱导阻力因子
            aerodynamics.maxAOA = 15.0 * M_PI / 180.0;  // 15度
            aerodynamics.minAOA = -5.0 * M_PI / 180.0;  // -5度
            aerodynamics.stallAOA = 19.0 * M_PI / 180.0; // 19度
            break;
    }
}

// 计算升力系数
double FighterJet::calculateLiftCoefficient(double angleOfAttack) const {
    return aerodynamics.CL0 + aerodynamics.CLalpha * angleOfAttack;
}

// 计算阻力系数
double FighterJet::calculateDragCoefficient(double angleOfAttack) const {
    double liftCoeff = calculateLiftCoefficient(angleOfAttack);
    return aerodynamics.CD0 + aerodynamics.K * liftCoeff * liftCoeff;
}

// 计算升力
double FighterJet::calculateLift(double angleOfAttack, double airspeed) const {
    double liftCoeff = calculateLiftCoefficient(angleOfAttack);
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    return liftCoeff * dynamicPressure * performance.wingArea;
}

// 计算阻力
double FighterJet::calculateDrag(double angleOfAttack, double airspeed) const {
    double dragCoeff = calculateDragCoefficient(angleOfAttack);
    double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
    return dragCoeff * dynamicPressure * performance.wingArea;
}

// 检查是否失速
bool FighterJet::isStalling(double angleOfAttack) const {
    return std::abs(angleOfAttack) > aerodynamics.stallAOA;
}

// 计算加速度
Velocity3 FighterJet::computeAcceleration() const {
    double vx = velocity.north;
    double vy = velocity.up;
    double vz = velocity.east;
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

    Velocity3 acc{ 0.0, 0.0, 0.0 };
    if (speed > 1e-3) {
        double thrustAcc = performance.maxThrust / performance.mass;
        double dragAcc = performance.dragCoefficient * speed * speed / performance.mass;
        double netAcc = thrustAcc - dragAcc;

        // 加速度方向与当前速度方向一致
        acc.north = netAcc * (vx / speed);
        acc.up = netAcc * (vy / speed);
        acc.east = netAcc * (vz / speed);
    }
    return acc;
}
