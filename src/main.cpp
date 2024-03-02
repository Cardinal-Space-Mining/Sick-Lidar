#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "lidar_api.h"
#include "./core/mem_utils.hpp"


static std::atomic<bool> _program_running = true;
static void _action(int sig) {
	if(_program_running) std::cout << "Caught signal. Stopping program..." << std::endl;
	_program_running = false;
}

int main(int argc, char** argv) {

	using ldrp::status_t;

	status_t s{0};
	ldrp::LidarConfig _config{};
	_config.points_logging_mode = (ldrp::POINT_LOGGING_INCLUDE_FILTERED | ldrp::POINT_LOGGING_NT);
	// _config.lidar_offset_xyz[2] = 1.f;
	// _config.lidar_offset_quat[0] = 1.f;
	// _config.lidar_offset_quat[3] = 0.f;
	// _config.datalog_fname = "test_configs_import.wpilog";

	s = ldrp::apiInit(_config);
	s = ldrp::lidarInit();
	// std::cout << "lidar inited from main" << std::endl;

	signal(SIGINT, _action);

	using namespace std::chrono_literals;
	float					// x    y    z    w
		pose[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	for(;_program_running.load();) {

		s = ldrp::updateWorldPose(pose, pose + 3);
		pose[0] += 0.1;

		std::this_thread::sleep_for(100ms);
	}

	std::cout << "[Main Thread]: Shutting down lidar resources...?" << std::endl;
	s = ldrp::lidarShutdown();
	s = ldrp::apiDestroy();
	std::cout << "[Main Thread]: Exitting..." << std::endl;

}
