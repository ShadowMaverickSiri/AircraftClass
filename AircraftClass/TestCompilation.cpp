#include "AircraftModelLibrary.h"
#include "EulerAngleCalculator.h"
#include "ManeuverModel.h"
#include "AircraftModule.h"
#include <iostream>
#include <memory>

// 简单的测试类，继承自Aircraft
class TestAircraft : public Aircraft {
public:
    TestAircraft(const std::string& type, const std::string& model) 
        : Aircraft(type, model) {}
    
    // 实现纯虚函数
    Vector3 computeAcceleration() const override {
        Vector3 acc;
        acc.north = 0.0;
        acc.up = 0.0;
        acc.east = 0.0;
        return acc;
    }
};

// 简单的测试操纵模型
class TestManeuverModel : public ManeuverModel {
public:
    void initialize(const ManeuverParameters& params) override {
        // 简单实现
    }
    
    void update(Aircraft& aircraft, double dt) override {
        // 简单实现
    }
    
    std::string getName() const override {
        return "Test Maneuver";
    }
    
    void reset() override {
        // 简单实现
    }
};

int main() {
    try {
        std::cout << "开始编译测试..." << std::endl;
        
        // 测试基本结构体
        GeoPosition pos{120.0, 30.0, 1000.0};
        Vector3 vel{100.0, 10.0, 50.0};
        AttitudeAngles attitude;
        AircraftPerformance perf;
        
        std::cout << "基本结构体创建成功" << std::endl;
        
        // 测试Aircraft类
        TestAircraft aircraft("fighter", "F-15");
        std::cout << "Aircraft对象创建成功" << std::endl;
        
        // 测试操纵模型
        auto maneuverModel = std::make_shared<TestManeuverModel>();
        aircraft.setManeuverModel(maneuverModel);
        std::cout << "操纵模型设置成功" << std::endl;
        
        // 测试操纵参数
        ManeuverParameters params;
        aircraft.initializeManeuver(params);
        std::cout << "操纵参数初始化成功" << std::endl;
        
        // 测试模块系统
        auto jammerModule = std::make_shared<JammerModule>();
        aircraft.addModule(jammerModule);
        std::cout << "模块系统测试成功" << std::endl;
        
        // 测试坐标转换
        aircraft.setReferencePosition(pos);
        Vector3 ecef = aircraft.getECEFPosition();
        Vector3 localNUE = aircraft.getLocalNUEPosition();
        double distance = aircraft.getDistanceFromReference();
        double bearing = aircraft.getBearingFromReference();
        
        std::cout << "坐标转换测试成功" << std::endl;
        
        // 测试更新函数
        aircraft.updateKinematics(0.1);
        aircraft.updateAttitude(0.1);
        aircraft.updateManeuver(0.1);
        aircraft.updateModules(0.1);
        
        std::cout << "更新函数测试成功" << std::endl;
        
        // 测试欧拉角计算
        AttitudeAngles calculatedAttitude = EulerAngleCalculator::calculateFromVelocity(vel);
        std::cout << "欧拉角计算测试成功" << std::endl;
        
        // 测试地理位置更新
        GeoPosition newPos = updateGeoPosition(pos, vel, 0.1);
        std::cout << "地理位置更新测试成功" << std::endl;
        
        std::cout << "所有编译测试通过！" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "编译测试失败: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "编译测试失败: 未知错误" << std::endl;
        return 1;
    }
}
