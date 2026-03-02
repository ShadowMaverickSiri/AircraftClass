#ifndef FIGHTER_JET_H
#define FIGHTER_JET_H

#include "AircraftModelLibrary.h"
#include <string>
#include <cmath>

// ============================================================================
// 战斗机类型枚举
// ============================================================================
enum class FighterType {
    F15_EAGLE,
    F16_FALCON,
    SU27_FLANKER,
    MIG29_FULCRUM,
    F22_RAPTOR,
    F35_LIGHTNING
};

// ============================================================================
// 战斗机气动参数
// ============================================================================
struct FighterAerodynamics {
    double CL0 = 0.0;         // 零攻角升力系数
    double CLalpha = 0.0;     // 升力曲线斜率
    double CD0 = 0.0;         // 零升阻力系数
    double K = 0.0;           // 诱导阻力因子
    double maxAOA = 0.0;      // 最大攻角 (弧度)
    double minAOA = 0.0;      // 最小攻角 (弧度)
    double stallAOA = 0.0;    // 失速攻角 (弧度)
};

// ============================================================================
// 战斗机类 - 内联实现
// ============================================================================
class FighterJet : public Aircraft {
public:
    // 构造函数
    FighterJet(const std::string& modelName, const std::string& type)
        : Aircraft(modelName, type) {
        fighterType = determineFighterType(modelName);
        initializeAerodynamics();
    }

    FighterJet(FighterType type, const GeoPosition& pos = {0, 0, 10000},
               const Velocity3& vel = {200, 0, 0},
               const AttitudeAngles& att = AttitudeAngles())
        : Aircraft("Fighter", "Jet"), fighterType(type) {
        position = pos;
        velocity = vel;
        attitude = att;
        initializeAerodynamics();
    }

    // 重写基类方法
    Velocity3 computeAcceleration() const override {
        double speed = std::sqrt(velocity.north * velocity.north +
                                velocity.up * velocity.up +
                                velocity.east * velocity.east);

        Velocity3 acc{0.0, 0.0, 0.0};
        if (speed > 1e-3) {
            double thrustAcc = performance.maxThrust / performance.mass;
            double dragAcc = performance.dragCoefficient * speed * speed / performance.mass;
            double netAcc = thrustAcc - dragAcc;

            acc.north = netAcc * (velocity.north / speed);
            acc.up = netAcc * (velocity.up / speed);
            acc.east = netAcc * (velocity.east / speed);
        }
        return acc;
    }

    // 获取战斗机类型
    FighterType getFighterType() const { return fighterType; }
    const FighterAerodynamics& getAerodynamics() const { return aerodynamics; }

    // 气动参数查询
    double getCL0() const { return aerodynamics.CL0; }
    double getCLalpha() const { return aerodynamics.CLalpha; }
    double getCD0() const { return aerodynamics.CD0; }
    double getK() const { return aerodynamics.K; }
    double getMaxAOA() const { return aerodynamics.maxAOA; }
    double getMinAOA() const { return aerodynamics.minAOA; }
    double getStallAOA() const { return aerodynamics.stallAOA; }

    // 气动计算（内联）
    double calculateLiftCoefficient(double angleOfAttack) const {
        return aerodynamics.CL0 + aerodynamics.CLalpha * angleOfAttack;
    }

    double calculateDragCoefficient(double angleOfAttack) const {
        double liftCoeff = calculateLiftCoefficient(angleOfAttack);
        return aerodynamics.CD0 + aerodynamics.K * liftCoeff * liftCoeff;
    }

    double calculateLift(double angleOfAttack, double airspeed) const {
        double liftCoeff = calculateLiftCoefficient(angleOfAttack);
        double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
        return liftCoeff * dynamicPressure * performance.wingArea;
    }

    double calculateDrag(double angleOfAttack, double airspeed) const {
        double dragCoeff = calculateDragCoefficient(angleOfAttack);
        double dynamicPressure = 0.5 * 1.225 * airspeed * airspeed;
        return dragCoeff * dynamicPressure * performance.wingArea;
    }

    bool isStalling(double angleOfAttack) const {
        return std::abs(angleOfAttack) > aerodynamics.stallAOA;
    }

private:
    FighterType fighterType;
    FighterAerodynamics aerodynamics;

    // 根据型号名称确定战斗机类型
    FighterType determineFighterType(const std::string& modelName) const {
        if (modelName.find("F-15") != std::string::npos) return FighterType::F15_EAGLE;
        if (modelName.find("F-16") != std::string::npos) return FighterType::F16_FALCON;
        if (modelName.find("Su-27") != std::string::npos ||
            modelName.find("SU-27") != std::string::npos) return FighterType::SU27_FLANKER;
        if (modelName.find("MiG-29") != std::string::npos ||
            modelName.find("MIG-29") != std::string::npos) return FighterType::MIG29_FULCRUM;
        if (modelName.find("F-22") != std::string::npos) return FighterType::F22_RAPTOR;
        if (modelName.find("F-35") != std::string::npos) return FighterType::F35_LIGHTNING;
        return FighterType::F16_FALCON;  // 默认
    }

    // 初始化气动参数
    void initializeAerodynamics() {
        switch (fighterType) {
            case FighterType::F15_EAGLE:
                aerodynamics.CL0 = 0.15;
                aerodynamics.CLalpha = 5.2;
                aerodynamics.CD0 = 0.020;
                aerodynamics.K = 0.085;
                aerodynamics.maxAOA = 15.0 * M_PI / 180.0;
                aerodynamics.minAOA = -5.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 18.0 * M_PI / 180.0;
                break;

            case FighterType::F16_FALCON:
                aerodynamics.CL0 = 0.12;
                aerodynamics.CLalpha = 5.5;
                aerodynamics.CD0 = 0.018;
                aerodynamics.K = 0.080;
                aerodynamics.maxAOA = 14.0 * M_PI / 180.0;
                aerodynamics.minAOA = -4.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 16.0 * M_PI / 180.0;
                break;

            case FighterType::SU27_FLANKER:
                aerodynamics.CL0 = 0.18;
                aerodynamics.CLalpha = 5.8;
                aerodynamics.CD0 = 0.022;
                aerodynamics.K = 0.090;
                aerodynamics.maxAOA = 16.0 * M_PI / 180.0;
                aerodynamics.minAOA = -6.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 20.0 * M_PI / 180.0;
                break;

            case FighterType::MIG29_FULCRUM:
                aerodynamics.CL0 = 0.14;
                aerodynamics.CLalpha = 5.6;
                aerodynamics.CD0 = 0.021;
                aerodynamics.K = 0.088;
                aerodynamics.maxAOA = 15.0 * M_PI / 180.0;
                aerodynamics.minAOA = -5.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 18.0 * M_PI / 180.0;
                break;

            case FighterType::F22_RAPTOR:
                aerodynamics.CL0 = 0.10;
                aerodynamics.CLalpha = 6.0;
                aerodynamics.CD0 = 0.015;
                aerodynamics.K = 0.075;
                aerodynamics.maxAOA = 17.0 * M_PI / 180.0;
                aerodynamics.minAOA = -7.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 22.0 * M_PI / 180.0;
                break;

            case FighterType::F35_LIGHTNING:
                aerodynamics.CL0 = 0.11;
                aerodynamics.CLalpha = 5.7;
                aerodynamics.CD0 = 0.017;
                aerodynamics.K = 0.078;
                aerodynamics.maxAOA = 15.0 * M_PI / 180.0;
                aerodynamics.minAOA = -5.0 * M_PI / 180.0;
                aerodynamics.stallAOA = 19.0 * M_PI / 180.0;
                break;
        }
    }
};

#endif // FIGHTER_JET_H
