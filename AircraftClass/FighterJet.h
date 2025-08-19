#ifndef FIGHTER_JET_H
#define FIGHTER_JET_H

#include "AircraftModelLibrary.h"
#include <string>

// 战斗机类型枚举
enum class FighterType {
    F15_EAGLE,      // F-15 鹰式
    F16_FALCON,     // F-16 猎鹰
    SU27_FLANKER,   // Su-27 侧卫
    MIG29_FULCRUM,  // MiG-29 支点
    F22_RAPTOR,     // F-22 猛禽
    F35_LIGHTNING   // F-35 闪电
};

// 战斗机气动参数结构
struct FighterAerodynamics {
    double CL0;                 // 零攻角升力系数
    double CLalpha;             // 升力曲线斜率 (弧度制)
    double CD0;                 // 零升阻力系数
    double K;                   // 诱导阻力因子
    double maxAOA;              // 最大攻角 (弧度)
    double minAOA;              // 最小攻角 (弧度)
    double stallAOA;            // 失速攻角 (弧度)
    
    FighterAerodynamics() : CL0(0.0), CLalpha(0.0), CD0(0.0), K(0.0), 
                           maxAOA(0.0), minAOA(0.0), stallAOA(0.0) {}
};

// 战斗机类：继承自 Aircraft
class FighterJet : public Aircraft {
public:
    // 构造函数
    FighterJet(const std::string& modelName, const std::string& type);
    FighterJet(FighterType type, const GeoPosition& pos = {0, 0, 10000}, 
               const Velocity3& vel = {200, 0, 0}, const AttitudeAngles& att = AttitudeAngles());
    
    // 重写基类方法
    Velocity3 computeAcceleration() const override;
    
    // 战斗机特有方法
    FighterType getFighterType() const { return fighterType; }
    const FighterAerodynamics& getAerodynamics() const { return aerodynamics; }
    
    // 气动参数查询方法（供机动模型使用）
    double getCL0() const { return aerodynamics.CL0; }
    double getCLalpha() const { return aerodynamics.CLalpha; }
    double getCD0() const { return aerodynamics.CD0; }
    double getK() const { return aerodynamics.K; }
    double getMaxAOA() const { return aerodynamics.maxAOA; }
    double getMinAOA() const { return aerodynamics.minAOA; }
    double getStallAOA() const { return aerodynamics.stallAOA; }
    
    // 计算升力系数
    double calculateLiftCoefficient(double angleOfAttack) const;
    
    // 计算阻力系数
    double calculateDragCoefficient(double angleOfAttack) const;
    
    // 计算升力
    double calculateLift(double angleOfAttack, double airspeed) const;
    
    // 计算阻力
    double calculateDrag(double angleOfAttack, double airspeed) const;
    
    // 检查是否失速
    bool isStalling(double angleOfAttack) const;

private:
    FighterType fighterType;
    FighterAerodynamics aerodynamics;
    
    // 初始化战斗机气动参数
    void initializeAerodynamics(FighterType type);
    
    // 根据型号名称确定战斗机类型
    FighterType determineFighterType(const std::string& modelName) const;
};

#endif // FIGHTER_JET_H
