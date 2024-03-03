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
inline static uint8_t* _grid_alloc(size_t s) {
	return (uint8_t*)malloc(s);
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
	using namespace std::chrono;
	// float					// x    y    z    w
	// 	pose[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	ldrp::ObstacleGrid grid{};
	for(;_program_running.load();) {

		// s = ldrp::updateWorldPose(pose, pose + 3);
		// pose[0] += 0.1;
		high_resolution_clock::time_point a = high_resolution_clock::now();
		s = ldrp::waitNextObstacleGrid(grid, &_grid_alloc, 500.0);
		double dur = duration<double>{high_resolution_clock::now() - a}.count();
		if(s & ldrp::STATUS_TIMED_OUT) {
			std::cout << "[Main Thread]: Obstacle export timed out after " << dur << " seconds." << std::endl;
		} else if(s == ldrp::STATUS_SUCCESS) {
			std::cout << "[Main Thread]: Obstacle export succeeded after " << dur << " seconds." << std::endl;
			std::cout << "\t>> grid origin: (" << grid.origin_x_m << ", " << grid.origin_y_m << "), grid size: {" << grid.cells_x << " x " << grid.cells_y << "}" << std::endl;
		} else {
			std::cout << "[Main Thread]: Obstacle export failed after " << dur << " seconds." << std::endl;
		}
		free(grid.grid);
		grid.grid = nullptr;

		// std::this_thread::sleep_for(100ms);
	}

	std::cout << "[Main Thread]: Shutting down lidar resources...?" << std::endl;
	s = ldrp::lidarShutdown();
	s = ldrp::apiDestroy();
	std::cout << "[Main Thread]: Exitting..." << std::endl;

}
