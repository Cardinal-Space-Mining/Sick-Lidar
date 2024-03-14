#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <stdio.h>
#include <fstream>

#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/RawTopic.h>

#include "lidar_api.h"
#include "./core/mem_utils.hpp"


static std::atomic<bool> _program_running = true;
static void _action(int sig) {
	if(_program_running) std::cout << "Caught signal. Stopping program..." << std::endl;
	_program_running = false;
}
static uint8_t* _grid_alloc(size_t s) {
	return (uint8_t*)malloc(s + (sizeof(int64_t) * 2)) + (sizeof(int64_t) * 2);		// extra space for size at the beginning
}

int main(int argc, char** argv) {

	using ldrp::status_t;

	status_t s{0};
	ldrp::LidarConfig _config{};
	_config.points_logging_mode = (ldrp::POINT_LOGGING_INCLUDE_FILTERED | ldrp::POINT_LOGGING_NT);
	_config.nt_use_client = false;
	// _config.lidar_offset_xyz[2] = 7.5f;
	_config.min_scan_theta_degrees = -180.f;
	_config.max_scan_theta_degrees = 180.f;
	// _config.nt_client_team = 1111;
	_config.pose_history_period_s = 1.0;
	_config.map_resolution_cm = 3.f;
	_config.max_filter_threads = 1;

	s = ldrp::apiInit(_config);
	s = ldrp::lidarInit();
	// std::cout << "lidar inited from main" << std::endl;

	// nt::NetworkTableInstance::GetDefault().StartServer();
	nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();
	// nt::FloatArrayEntry nt_localization = nt_inst.GetFloatArrayTopic("rio telemetry/robot/pigeon rotation quat").GetEntry({});
	nt::FloatArrayEntry nt_localization = nt_inst.GetFloatArrayTopic("uesim/pose").GetEntry({});
	nt::RawEntry nt_grid = nt_inst.GetRawTopic("tmain/obstacle grid").GetEntry("Grid<U8>", {});

	signal(SIGINT, _action);

	using namespace std::chrono_literals;
	using namespace std::chrono;
	// float					// x    y    z    w
	// 	pose[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	ldrp::ObstacleGrid grid{};
	for(;_program_running.load();) {

		std::vector<nt::TimestampedFloatArray> updates = nt_localization.ReadQueue();
		// std::cout << "[Main Thread]: Localization recieved " << updates.size() << " pose updates" << std::endl;
		for(const nt::TimestampedFloatArray& u : updates) {
			ldrp::updateWorldPose(u.value.data(), u.value.data() + 3, u.time);
		}

		// s = ldrp::updateWorldPose(pose, pose + 3);
		// pose[0] += 0.1;
		if (grid.grid) {
			free(grid.grid);
			grid.grid = nullptr;
		}
		// high_resolution_clock::time_point a = high_resolution_clock::now();
		s = ldrp::waitNextObstacleGrid(grid, &_grid_alloc, 10.0);
		if(s == ldrp::STATUS_SUCCESS) grid.grid -= (sizeof(int64_t) * 2);
		// double dur = duration<double>{high_resolution_clock::now() - a}.count();
		// if(s & ldrp::STATUS_TIMED_OUT) {
		// 	std::cout << "[Main Thread]: Obstacle export timed out after " << dur << " seconds." << std::endl;
		// } else if(s == ldrp::STATUS_SUCCESS) {
		// 	std::cout << "[Main Thread]: Obstacle export succeeded after " << dur << " seconds." << std::endl;
		// 	std::cout << "\t>> grid origin: (" << grid.origin_x_m << ", " << grid.origin_y_m << "), grid size: {" << grid.cells_x << " x " << grid.cells_y << "}" << std::endl;
		// } else {
		// 	std::cout << "[Main Thread]: Obstacle export failed after " << dur << " seconds." << std::endl;
		// }

		if(grid.grid) {
			reinterpret_cast<int64_t*>(grid.grid)[0] = grid.cells_x;
			reinterpret_cast<int64_t*>(grid.grid)[1] = grid.cells_y;
			nt_grid.Set(
				std::span<const uint8_t>{
					grid.grid,
					grid.grid + (grid.cells_x * grid.cells_y + (sizeof(int64_t) * 2))
				}
			);
		}

		std::this_thread::sleep_for(10ms);
	}

	// bmp::generateBitmapImage(grid.grid, grid.cells_x, grid.cells_y, (char*)"./logs/out.bmp");

	// std::ofstream grid_out;
	// grid_out.open("./logs/out.txt", std::ios::binary | std::ios::out);
	// // std::unique_ptr<char[]> bytes = hexify<uint8_t>(grid.grid, grid.cells_x * grid.cells_y);
	// // grid_out << bytes.get();
	// grid_out.write((const char*)grid.grid, grid.cells_x * grid.cells_y);
	// grid_out.flush();
	// grid_out.close();
	
	// free(grid.grid);


	std::cout << "[Main Thread]: Shutting down lidar resources...?" << std::endl;
	s = ldrp::lidarShutdown();
	s = ldrp::apiDestroy();
	std::cout << "[Main Thread]: Exiting..." << std::endl;

}
