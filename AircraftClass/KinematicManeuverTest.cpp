#include <iostream>
#include <iomanip>
#include <memory>
#include "KinematicManeuverSystem.h"
#include "TacviewTelemetry.h"  // 添加Tacview实时遥测支持
#include <fstream> // Added for CSV file output

void LEVEL_TURN_TEST();  //水平转弯测试函数
void LOOP_TEST();		 //筋斗测试函数
void SPLIT_S_TEST();     //半滚倒转测试函数
void TACVIEW_TEST();     //测试tacview通信功能

// 全局Tacview遥测对象
std::unique_ptr<TacviewTelemetry> g_telemetry;
KinematicManeuverParameters params;

constexpr double dt = 0.01;  // 仿真时间步长
double totalTime = 0.0;
double endTime = 30.0;  // 仿真20秒
						



int main() {
    std::cout << "=== 运动学机动系统测试 ===" << std::endl;
    
    // 初始化Tacview实时遥测服务器
    //g_telemetry = std::make_unique<TacviewTelemetry>(42674, "F-22-01");
    //if (g_telemetry->start()) {
    //    std::cout << "Tacview遥测服务器已启动！" << std::endl;
    //    std::cout << "请在Tacview中连接到 127.0.0.1:42674" << std::endl;
    //    std::cout << "连接后即可看到实时轨迹" << std::endl;
    //} else {
    //    std::cout << "错误：无法启动Tacview遥测服务器" << std::endl;
    //    std::cout << "请检查端口42674是否被占用" << std::endl;
    //    std::cout << "程序将继续运行，但仅保存CSV文件" << std::endl;
    //}
    
	//LEVEL_TURN_TEST();
	//LOOP_TEST();
	SPLIT_S_TEST();  //破S机动
	//TACVIEW_TEST();

    // 检查连接状态
    if (g_telemetry) {
        g_telemetry->checkConnectionStatus();
        g_telemetry->stop();
    }
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    return 0;
}

void LEVEL_TURN_TEST()
{
	// 设置初始条件
	params.initialPosition = { 120.0, 30.0, 10000.0 };  // 初始位置：经度0，纬度0，高度10000米
	params.initialVelocity = { 350., 0.0, 0.0 };    // 初始速度：250m/s向北
	params.startTime = 1.0;                         // 5秒后开始机动
	params.duration = 30.0;                         // 机动持续15秒


	// 测试水平转弯
	std::cout << "\n--- 水平转弯测试 ---" << std::endl;
	auto levelTurn = KinematicManeuverFactory::create(ManeuverType::LEVEL_TURN);
	params.type = ManeuverType::LEVEL_TURN;
	params.targetGForce = 3.0;
	params.turnDirection = 1.0;  // 右转
	levelTurn->initialize(params);

	// 创建CSV文件
	std::ofstream csvFile("level_turn_results.csv");
	if (csvFile.is_open()) {
		// 写入CSV头部 - Tacview兼容格式   
		csvFile << "Time(s),Longitude(deg),Latitude(deg),Altitude(m),"
				<< "NorthVelocity(m/s),UpVelocity(m/s),EastVelocity(m/s),"
				<< "GForce,Pitch(deg),Roll(deg),Yaw(deg),"
				<< "AircraftType" << std::endl;
	}

	std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
	std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------" << std::endl;

	GeoPosition position = params.initialPosition;
	Velocity3 velocity = params.initialVelocity;
	AttitudeAngles attitude;


	while (totalTime <= endTime) {
		// 更新机动
		levelTurn->update(totalTime, dt, position, velocity, attitude);

		// 发送实时遥测数据到Tacview
		if (g_telemetry && g_telemetry->isRunning()) {
			g_telemetry->sendSimulationData(totalTime, position, velocity, attitude, 
											levelTurn->getCurrentGForce(), "F-22");
		}

		// 保存到CSV文件（每个时间步都保存）
		if (csvFile.is_open()) {
			csvFile << std::fixed << std::setprecision(6);
			csvFile << totalTime << ","
					<< position.longitude << ","
					<< position.latitude << ","
					<< position.altitude << ","
					<< velocity.north << ","
					<< velocity.up << ","
					<< velocity.east << ","
					<< levelTurn->getCurrentGForce() << ","
					<< attitude.getPitchDegrees() << ","
					<< attitude.getRollDegrees() << ","
					<< attitude.getYawDegrees() << ","
					<< "F-22" << std::endl;
		}

		// 打印状态（每1秒打印一次）
		if (fmod(totalTime, 1.0) < dt) {
			std::cout << std::fixed << std::setprecision(2);
			std::cout << std::setw(8) << totalTime
				<< std::setw(9) << position.longitude
				<< std::setw(9) << position.latitude
				<< std::setw(10) << position.altitude
				<< std::setw(11) << velocity.north
				<< std::setw(11) << velocity.up
				<< std::setw(11) << velocity.east
				<< std::setw(9) << attitude.getPitchDegrees()
				<< std::setw(9) << attitude.getRollDegrees()
				<< std::setw(9) << attitude.getYawDegrees() << std::endl;
		}

		totalTime += dt;
	}

	// 关闭CSV文件
	if (csvFile.is_open()) {
		csvFile.close();
		std::cout << "水平转弯测试结果已保存到 level_turn_results.csv" << std::endl;
	}
}

void LOOP_TEST()
{
	params.initialPosition = { 125., 30., 10000.0 };  // 初始位置：经度0，纬度0，高度10000米
	params.initialVelocity = { 350., 0.0, 0.0 };    // 初始速度：250m/s向北
	params.startTime = 5.0;                         // 5秒后开始机动
	params.duration = 15.0;                         // 机动持续15秒
	// 测试筋斗
	std::cout << "\n--- 筋斗测试 ---" << std::endl;
	auto loop = KinematicManeuverFactory::create(ManeuverType::LOOP);
	params.type = ManeuverType::LOOP;
	params.targetGForce = 4.0;
	params.duration = 10.;
	loop->initialize(params);

	// 创建CSV文件
	std::ofstream csvFile("loop_results.csv");
	if (csvFile.is_open()) {
		// 写入CSV头部 - Tacview兼容格式
		csvFile << "Time(s),Longitude(deg),Latitude(deg),Altitude(m),"
				<< "NorthVelocity(m/s),UpVelocity(m/s),EastVelocity(m/s),"
				<< "GForce,Pitch(deg),Roll(deg),Yaw(deg),"
				<< "AircraftType" << std::endl;
	}

	GeoPosition position = params.initialPosition;
	Velocity3 velocity = params.initialVelocity;
	AttitudeAngles attitude;

	// 重置状态
	position = params.initialPosition;
	velocity = params.initialVelocity;

	std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
	std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------" << std::endl;

	while (totalTime <= endTime) {
		loop->update(totalTime, dt, position, velocity, attitude);

		// 发送实时遥测数据到Tacview
		if (g_telemetry && g_telemetry->isRunning()) {
			g_telemetry->sendSimulationData(totalTime, position, velocity, attitude, 
											loop->getCurrentGForce(), "F-22");
		}

		// 保存到CSV文件（每个时间步都保存）
		if (csvFile.is_open()) {
			csvFile << std::fixed << std::setprecision(6);
			csvFile << totalTime << ","
					<< position.longitude << ","
					<< position.latitude << ","
					<< position.altitude << ","
					<< velocity.north << ","
					<< velocity.up << ","
					<< velocity.east << ","
					<< loop->getCurrentGForce() << ","
					<< attitude.getPitchDegrees() << ","
					<< attitude.getRollDegrees() << ","
					<< attitude.getYawDegrees() << ","
					<< "F-22" << std::endl;
		}

		if (fmod(totalTime, 1.0) < dt) {
			std::cout << std::fixed << std::setprecision(2);
			std::cout << std::setw(8) << totalTime
				<< std::setw(9) << position.longitude
				<< std::setw(9) << position.latitude
				<< std::setw(10) << position.altitude
				<< std::setw(11) << velocity.north
				<< std::setw(11) << velocity.up
				<< std::setw(11) << velocity.east
				<< std::setw(8) << loop->getCurrentGForce()
				<< std::setw(9) << attitude.getPitchDegrees()
				<< std::setw(9) << attitude.getRollDegrees()
				<< std::setw(9) << attitude.getYawDegrees() << std::endl;
		}

		totalTime += dt;
	}

	// 关闭CSV文件
	if (csvFile.is_open()) {
		csvFile.close();
		std::cout << "筋斗测试结果已保存到 loop_results.csv" << std::endl;
	}
}

void SPLIT_S_TEST()
{
	// 设置初始条件
	params.initialPosition = { 120.0, 30.0, 10000.0 };  // 初始位置：经度120°，纬度30°，高度10000米
	params.initialVelocity = { 350.0, 0.0, 0.0 };    // 初始速度：350m/s向北
	params.startTime = 1.0;                         // 1秒后开始机动
	params.duration = 12.0;                         // 机动持续12秒
	
	// 测试半滚倒转
	std::cout << "\n--- 半滚倒转测试 ---" << std::endl;
	auto splitS = KinematicManeuverFactory::create(ManeuverType::SPLIT_S);
	params.type = ManeuverType::SPLIT_S;
	params.targetGForce = 2.0;
	params.rollDirection = 1.0;  // 右滚
	splitS->initialize(params);

	// 创建CSV文件
	std::ofstream csvFile("split_s_results.csv");
	if (csvFile.is_open()) {
		// 写入CSV头部 - Tacview兼容格式
		csvFile << "Time(s),Longitude(deg),Latitude(deg),Altitude(m),"
				<< "NorthVelocity(m/s),UpVelocity(m/s),EastVelocity(m/s),"
				<< "GForce,Pitch(deg),Roll(deg),Yaw(deg),"
				<< "AircraftType" << std::endl;
	}

	GeoPosition position = params.initialPosition;
	Velocity3 velocity = params.initialVelocity;
	AttitudeAngles attitude;

	// 重置状态
	position = params.initialPosition;
	velocity = params.initialVelocity;


	std::cout << "时间(s) | 经度(°) | 纬度(°) | 高度(m) | 北速(m/s) | 天速(m/s) | 东速(m/s) | 过载(G) | 俯仰(°) | 滚转(°) | 偏航(°)" << std::endl;
	std::cout << "--------|---------|---------|----------|-----------|-----------|-----------|--------|---------|---------|---------" << std::endl;

	while (totalTime <= endTime) {
		splitS->update(totalTime, dt, position, velocity, attitude);

		// 发送实时遥测数据到Tacview
		if (g_telemetry && g_telemetry->isRunning()) {
			g_telemetry->sendSimulationData(totalTime, position, velocity, attitude, 
											splitS->getCurrentGForce(), "F-22");
		}

		// 保存到CSV文件（每个时间步都保存）
		if (csvFile.is_open()) {
			csvFile << std::fixed << std::setprecision(6);
			csvFile << totalTime << ","
					<< position.longitude << ","
					<< position.latitude << ","
					<< position.altitude << ","
					<< velocity.north << ","
					<< velocity.up << ","
					<< velocity.east << ","
					<< splitS->getCurrentGForce() << ","
					<< attitude.getPitchDegrees() << ","
					<< attitude.getRollDegrees() << ","
					<< attitude.getYawDegrees() << ","
					<< "F-22" << std::endl;
		}
		//控制台数据观测
		if (fmod(totalTime, 1.0) < dt) {
			std::cout << std::fixed << std::setprecision(2);
			std::cout << std::setw(8) << totalTime
				<< std::setw(9) << position.longitude
				<< std::setw(9) << position.latitude
				<< std::setw(10) << position.altitude
				<< std::setw(11) << velocity.north
				<< std::setw(11) << velocity.up
				<< std::setw(11) << velocity.east
				<< std::setw(8) << splitS->getCurrentGForce()
				<< std::setw(9) << attitude.getPitchDegrees()
				<< std::setw(9) << attitude.getRollDegrees()
				<< std::setw(9) << attitude.getYawDegrees() << std::endl;
		}

		totalTime += dt;
	}

	// 关闭CSV文件
	if (csvFile.is_open()) {
		csvFile.close();
		std::cout << "半滚倒转测试结果已保存到 split_s_results.csv" << std::endl;
	}
}

void TACVIEW_TEST()
{
	std::cout << "=== Tacview实时遥测测试程序 ===" << std::endl;

	// 使用全局遥测对象
	if (!g_telemetry) {
		std::cerr << "错误：全局遥测对象未初始化" << std::endl;
		return;
	}
	
	if (!g_telemetry->isServerRunning()) {
		std::cerr << "错误：遥测服务器未启动" << std::endl;
		return;
	}

	std::cout << "开始发送测试数据到Tacview..." << std::endl;
	std::cout << "请在Tacview中连接到 127.0.0.1:42674 以查看实时轨迹" << std::endl;
	
	// 等待Tacview连接
	std::cout << "等待Tacview连接..." << std::endl;
	int waitCount = 0;
	while (!g_telemetry->isRunning() && waitCount < 30) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		waitCount++;
		std::cout << "等待中... (" << waitCount << "/30)" << std::endl;
	}
	
	if (!g_telemetry->isRunning()) {
		std::cerr << "超时：Tacview未连接，请检查连接状态" << std::endl;
		return;
	}
	
	std::cout << "Tacview已连接，开始发送数据！" << std::endl;

	// 模拟飞行数据
	double time = 0.0;
	double dt = 0.1;  // 100ms更新间隔

	// 模拟一个简单的圆形飞行轨迹
	double radius = 0.01;  // 约1km半径
	double centerLon = 120.0;
	double centerLat = 30.0;
	double altitude = 10000.0;
	
	for (int i = 0; i < 100; ++i) {
		// 计算圆形轨迹
		double angle = time * 0.5;  // 角速度

		double longitude = centerLon + radius * cos(angle);
		double latitude = centerLat + radius * sin(angle);
		
		// 调试输出：显示计算的位置
		if (i % 10 == 0) {
			std::cout << "Debug: time=" << time << ", angle=" << angle 
					  << ", lon=" << longitude << ", lat=" << latitude 
					  << ", cos=" << cos(angle) << ", sin=" << sin(angle) << std::endl;
		}

		// 计算速度（切线方向）
		double speed = 200.0;  // 200 m/s
		double northVel = -speed * sin(angle);
		double eastVel = speed * cos(angle);
		double upVel = 0.0;

		// 计算姿态
		double pitch = 0.0;
		double roll = 0.0;
		double yaw = angle * 180.0 / 3.1415926;  // 转换为度

		// 发送数据
		AttitudeAngles attitude;
		attitude.setPitchDegrees(pitch);
		attitude.setRollDegrees(roll);
		attitude.setYawDegrees(yaw);
		
		g_telemetry->sendSimulationData(time,
			{ longitude, latitude, altitude },
			{ northVel, upVel, eastVel },
			attitude,
			1.0,  // G力
			"F-22");

		// 每10帧显示一次状态
		if (i % 10 == 0) {
			std::cout << "时间: " << std::fixed << std::setprecision(1) << time
				<< "s, 位置: (" << longitude << ", " << latitude << "), "
				<< "高度: " << altitude << "m, 角度: " << angle << ", 已发送: " << g_telemetry->getSentFrames() << " 帧" << std::endl;
		}

		time += dt;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	std::cout << "测试完成！" << std::endl;
	std::cout << "总共发送了 " << g_telemetry->getSentFrames() << " 帧数据" << std::endl;

	
}
