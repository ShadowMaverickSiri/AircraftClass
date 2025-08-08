#ifndef AIRCRAFT_MODULE_H
#define AIRCRAFT_MODULE_H

#include <string>
#include <iostream>
// Forward declaration
class Aircraft;

// 通用功能模块基类
class AircraftModule {
public:
	virtual ~AircraftModule() = default;
	virtual std::string getModuleName() const = 0;
	virtual void update(Aircraft& aircraft, double dt) {}
};
// 示例：自定义干扰模块
class JammerModule : public AircraftModule {
public:
	std::string getModuleName() const override { return "Jammer"; }
	void activateJamming() { isActive = true; }
	void deactivateJamming() { isActive = false; }
	bool isJamming() const { return isActive; }
	void update(Aircraft& aircraft, double dt) override {
		if (isActive) {
			// 干扰逻辑示例
		}
	}
private:
	bool isActive = false;
};
#endif // AIRCRAFT_MODULE_H 

