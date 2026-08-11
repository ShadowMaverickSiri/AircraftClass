#include "tacview/AcmiEncoder.h"
#include "tacview/Tacview.h"

#include <array>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <chrono>
#include <cmath>
#include <thread>


void test1(); //验证基础功能
bool example();  //具体输出案例

int main() {
	//test1();
	return example() ? 0 : 1;
}

void test1() {
	// 第一部分：验证独立编码器能稳定输出关键 ACMI 字段。
	tacview::Options options;
	options.dataSource = "Test,Source";
	const std::string header = tacview::AcmiEncoder::header(options);
	if (header.find("FileType=text/acmi/tacview\n") == std::string::npos ||
		header.find("DataSource=Test Source\n") == std::string::npos) {
		std::cerr << "Header encoding failed\n";
	}

	tacview::ObjectInfo info{ 0x3E9, "Demo,Aircraft", "Air+FixedWing", "Blue" };
	tacview::ObjectState state;
	state.id = info.id;
	state.time = 1.25;
	state.longitude = 116.1234567;
	state.latitude = 40.7654321;
	state.altitude = 5000.0;
	state.roll = 10.0;
	state.pitch = -2.0;
	state.yaw = 90.0;

	const std::string definition = tacview::AcmiEncoder::objectDefinition(info, &state);
	if (definition.find("3E9,Type=Air+FixedWing,Name=Demo Aircraft,Color=Blue,T=") != 0 ||
		tacview::AcmiEncoder::objectRemoval(info.id) != "-3E9\n" ||
		tacview::AcmiEncoder::frameTime(state.time) != "#1.250\n") {
		std::cerr << "Object encoding failed\n";
	}
	tacview::ObjectProperties properties;
	properties.shortName = "Viper,One";
	properties.label = "Lon:116.12345 Lat:40.76543";
	const std::string encodedProperties =
		tacview::AcmiEncoder::objectProperties(info.id, properties);
	if (encodedProperties.find(",ShortName=Viper One") == std::string::npos ||
		encodedProperties.find(",Label=Lon:116.12345 Lat:40.76543\n") == std::string::npos) {
		std::cerr << "Object properties encoding failed\n";
	}
	// 第二部分：通过公开 API 走完启动、注册、更新、删除和记录流程。
	const char* recordingPath = "tacview_test_output.acmi";
	tacview::Options serverOptions;
	serverOptions.port = 42675; // 避免与 Tacview 默认端口上的程序冲突
	serverOptions.recordingFile = recordingPath;

	tacview::Tacview output(serverOptions);
	if (!output.start() ||
		!output.addObject({ 1, "Test Aircraft", "Air+FixedWing", "Blue" })) {
		std::cerr << "Server startup failed: " << output.lastError() << '\n';

	}

	state.id = 1;
	state.time = 0.0;
	if (!output.update(state)) {
		std::cerr << "State update failed: " << output.lastError() << '\n';
	}
	state.time = 0.1;
	state.longitude += 0.0001;
	if (!output.update(state) || !output.removeObject(1, 0.2)) {
		std::cerr << "Object lifecycle failed: " << output.lastError() << '\n';
	}
	output.stop(); // stop() 会先写完异步队列，再关闭记录文件

	std::ifstream file(recordingPath, std::ios::binary);
	std::ostringstream content;
	content << file.rdbuf();
	const std::string acmi = content.str();
	file.close();
	std::remove(recordingPath);

	if (acmi.find("FileType=text/acmi/tacview") == std::string::npos ||
		acmi.find("1,Type=Air+FixedWing,Name=Test Aircraft") == std::string::npos ||
		acmi.find("#0.100") == std::string::npos ||
		acmi.find("1,T=") == std::string::npos ||
		acmi.find("#0.200") == std::string::npos ||
		acmi.find("-1") == std::string::npos) {
		std::cerr << "Public API recording test failed\n";
	}

	std::cout << "TacviewTCPlink test passed\n";


}

bool example() {
	tacview::Options options;
	options.serverName = "F-16 vs Su-27 BVR Example";
	options.dataSource = "AIM-120 Intercept Example";
	options.recordingFile = "f16_aim120_vs_su27.acmi";

	tacview::Tacview output(options);
	if (!output.start()) {
		std::cerr << "Unable to start Tacview: " << output.lastError() << '\n';
		return false;
	}

	const tacview::ObjectId f16Id = 1;
	const tacview::ObjectId su27Id = 2;
	const tacview::ObjectId missileId = 3;
	const tacview::ObjectId explosionCoreId = 4;
	const tacview::ObjectId explosionParticleFirstId = 100;
	constexpr std::size_t explosionParticleCount = 12;

	// Name 保留 Tacview 内置机型名称以匹配模型；界面只额外显示一个 Label。
	if (!output.addObject({ f16Id, "F-16", "Air+FixedWing", "Red" }) ||
		!output.addObject({ su27Id, "Su-27", "Air+FixedWing", "Blue" })) {
		std::cerr << "Unable to add aircraft: " << output.lastError() << '\n';
		output.stop();
		return false;
	}

	// 简化运动参数：两机相距 50 km、等高相向飞行，AIM-120 在 12 s 发射。
	const double pi = 3.14159265358979323846;
	const double latitude = 40.0;
	const double altitude = 5000.0;
	const double initialSeparation = 50000.0;
	const double aircraftSpeed = 250.0;
	const double missileSpeed = 1200.0;
	const double launchTime = 12.0;
	const double timeStep = 0.1;
	const double explosionLifetime = 10.0;
	const double maximumScenarioTime = 60.0;

	// 在固定纬度附近，把东西向米数换算成经度差。
	const double metersPerLongitudeDegree =
		111320.0 * std::cos(latitude * pi / 180.0);
	const double f16StartLongitude = 116.0;
	const double su27StartLongitude =
		f16StartLongitude + initialSeparation / metersPerLongitudeDegree;
	auto makeLabel = [](const char* name, double longitude,
		double latitudeValue, double altitudeValue) {
		std::ostringstream text;
		text << std::fixed << std::setprecision(5)
			<< name << " Lon:" << longitude << " Lat:" << latitudeValue
			<< std::setprecision(0) << " Alt:" << altitudeValue << "m";
		return text.str();
	};

	bool missileInFlight = false;
	bool su27Alive = true;
	bool explosionCoreVisible = false;
	std::array<bool, explosionParticleCount> explosionParticlesVisible{};
	double missileLaunchLongitude = 0.0;
	double hitTime = 0.0;
	double hitLongitude = 0.0;
	double lastTime = 0.0;

	const int frameCount = static_cast<int>(maximumScenarioTime / timeStep) + 1;
	for (int frame = 0; frame < frameCount; ++frame) {
		const double currentTime = frame * timeStep;
		lastTime = currentTime;

		tacview::ObjectState f16;
		f16.id = f16Id;
		f16.time = currentTime;
		f16.longitude = f16StartLongitude +
			aircraftSpeed * currentTime / metersPerLongitudeDegree;
		f16.latitude = latitude;
		f16.altitude = altitude;
		f16.roll = 0.0;
		f16.pitch = 0.0;
		f16.yaw = 90.0;

		if (!output.update(f16) ||
			!output.updateProperties(
				f16Id, currentTime,
				{ "", makeLabel("F-16v", f16.longitude, f16.latitude, f16.altitude) })) {
			std::cerr << "Unable to update F-16: " << output.lastError() << '\n';
			output.stop();
			return false;
		}

		tacview::ObjectState su27;
		if (su27Alive) {
			su27.id = su27Id;
			su27.time = currentTime;
			su27.longitude = su27StartLongitude -
				aircraftSpeed * currentTime / metersPerLongitudeDegree;
			su27.latitude = latitude;
			su27.altitude = altitude;
			su27.roll = 0.0;
			su27.pitch = 0.0;
			su27.yaw = 270.0;

			if (!output.update(su27) ||
				!output.updateProperties(
					su27Id, currentTime,
					{ "", makeLabel("Su-27s", su27.longitude, su27.latitude, su27.altitude) })) {
				std::cerr << "Unable to update Su-27: " << output.lastError() << '\n';
				output.stop();
				return false;
			}
		}

		// 在 12 s 时创建红方 AIM-120，并从 F-16 当时的位置向东直线飞行。
		if (!missileInFlight && su27Alive && currentTime >= launchTime) {
			if (!output.addObject({ missileId, "AIM-120", "Weapon+Missile", "Red" })) {
				std::cerr << "Unable to launch AIM-120: " << output.lastError() << '\n';
				output.stop();
				return false;
			}
			missileLaunchLongitude = f16.longitude;
			missileInFlight = true;
		}

		if (missileInFlight && su27Alive) {
			tacview::ObjectState missile;
			missile.id = missileId;
			missile.time = currentTime;
			missile.longitude = missileLaunchLongitude +
				missileSpeed * (currentTime - launchTime) / metersPerLongitudeDegree;
			missile.latitude = latitude;
			missile.altitude = altitude;
			missile.roll = 0.0;
			missile.pitch = 0.0;
			missile.yaw = 90.0;

			// 当本帧导弹到达或越过目标位置时，将位置钳制到目标并判定命中。
			const bool hit = missile.longitude >= su27.longitude;
			if (hit) missile.longitude = su27.longitude;

			if (!output.update(missile)) {
				std::cerr << "Unable to update AIM-120: " << output.lastError() << '\n';
				output.stop();
				return false;
			}

			if (hit) {
				hitTime = currentTime;
				hitLongitude = su27.longitude;

				if (!output.removeObject(missileId, hitTime) ||
					!output.removeObject(su27Id, hitTime) ||
					!output.addObject({ explosionCoreId, "Explosion Core", "Misc+Explosion", "Yellow" })) {
					std::cerr << "Unable to process hit: " << output.lastError() << '\n';
					output.stop();
					return false;
				}

				tacview::ObjectState core;
				core.id = explosionCoreId;
				core.time = hitTime;
				core.longitude = hitLongitude;
				core.latitude = latitude;
				core.altitude = altitude;
				if (!output.update(core)) {
					std::cerr << "Unable to show explosion core: " << output.lastError() << '\n';
					output.stop();
					return false;
				}
				explosionCoreVisible = true;

				const char* particleColors[] = {
					"Yellow", "Orange", "Red", "Orange",
					"Yellow", "Red", "Orange", "Yellow",
					"Red", "Orange", "Yellow", "Red"
				};
				for (std::size_t i = 0; i < explosionParticleCount; ++i) {
					const tacview::ObjectId particleId = explosionParticleFirstId + i;
					if (!output.addObject({ particleId, "Explosion Particle",
						"Misc+Explosion", particleColors[i] })) {
						std::cerr << "Unable to create explosion particle: "
							<< output.lastError() << '\n';
						output.stop();
						return false;
					}

					tacview::ObjectState particle;
					particle.id = particleId;
					particle.time = hitTime;
					particle.longitude = hitLongitude;
					particle.latitude = latitude;
					particle.altitude = altitude;
					if (!output.update(particle)) {
						std::cerr << "Unable to initialize explosion particle: "
							<< output.lastError() << '\n';
						output.stop();
						return false;
					}
					explosionParticlesVisible[i] = true;
				}

				missileInFlight = false;
				su27Alive = false;
			}
		}

		if (!su27Alive) {
			const double explosionAge = currentTime - hitTime;

			if (explosionCoreVisible && explosionAge >= 1.0) {
				if (!output.removeObject(explosionCoreId, currentTime)) {
					std::cerr << "Unable to remove explosion core: "
						<< output.lastError() << '\n';
					output.stop();
					return false;
				}
				explosionCoreVisible = false;
			}

			bool anyParticleVisible = false;
			for (std::size_t i = 0; i < explosionParticleCount; ++i) {
				if (!explosionParticlesVisible[i]) continue;

				const double particleLifetime = 2.0 + (i % 5) * 2.0;
				const tacview::ObjectId particleId = explosionParticleFirstId + i;
				if (explosionAge >= particleLifetime) {
					if (!output.removeObject(particleId, currentTime)) {
						std::cerr << "Unable to remove explosion particle: "
							<< output.lastError() << '\n';
						output.stop();
						return false;
					}
					explosionParticlesVisible[i] = false;
					continue;
				}

				const double angle = 2.0 * pi * i / explosionParticleCount;
				const double radialSpeed = 45.0 + (i % 4) * 18.0;
				const double verticalSpeed = 30.0 + (i % 3) * 20.0;
				const double radialDistance = radialSpeed * explosionAge;

				tacview::ObjectState particle;
				particle.id = particleId;
				particle.time = currentTime;
				particle.longitude = hitLongitude +
					std::cos(angle) * radialDistance / metersPerLongitudeDegree;
				particle.latitude = latitude +
					std::sin(angle) * radialDistance / 111320.0;
				particle.altitude = altitude + verticalSpeed * explosionAge -
					4.905 * explosionAge * explosionAge;
				particle.yaw = angle * 180.0 / pi;

				if (!output.update(particle)) {
					std::cerr << "Unable to update explosion particle: "
						<< output.lastError() << '\n';
					output.stop();
					return false;
				}
				anyParticleVisible = true;
			}

			if (!explosionCoreVisible && !anyParticleVisible &&
				explosionAge >= explosionLifetime) {
				break;
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	// 正常命中时只剩 F-16；下面的分支也负责最大时长到达后的安全清理。
	bool cleanupSucceeded = true;
	if (missileInFlight && !output.removeObject(missileId, lastTime)) cleanupSucceeded = false;
	if (su27Alive && !output.removeObject(su27Id, lastTime)) cleanupSucceeded = false;
	if (explosionCoreVisible && !output.removeObject(explosionCoreId, lastTime)) {
		cleanupSucceeded = false;
	}
	for (std::size_t i = 0; i < explosionParticleCount; ++i) {
		if (explosionParticlesVisible[i] &&
			!output.removeObject(explosionParticleFirstId + i, lastTime)) {
			cleanupSucceeded = false;
		}
	}
	if (!output.removeObject(f16Id, lastTime)) cleanupSucceeded = false;
	if (!cleanupSucceeded) {
		std::cerr << "Unable to clean up scenario: " << output.lastError() << '\n';
		output.stop();
		return false;
	}

	output.stop();
	return true;
}
