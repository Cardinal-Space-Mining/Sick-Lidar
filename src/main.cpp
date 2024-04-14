#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <stdio.h>
#include <fstream>

#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/RawTopic.h>

#include "lidar_api.h"
#include "./core/mem_utils.hpp"


static std::atomic<bool> _program_running = true;
static void _action(int sig) {
	if(_program_running) std::cout << "Caught signal. Stopping program..." << std::endl;
	_program_running = false;
}
static uint8_t* _grid_alloc(size_t s) {
	return ((uint8_t*)malloc(s + 20) + 20);		// extra space for size at the beginning
}

int main(int argc, char** argv) {

	using ldrp::status_t;

	std::cout << "ldrp feature bits: " << ldrp::featureBits() << std::endl;

	status_t s{0};
	ldrp::LidarConfig _config{};
	_config.points_logging_mode = (ldrp::POINT_LOGGING_INCLUDE_FILTERED | ldrp::POINT_LOGGING_NT);
	_config.nt_use_client = false;
	_config.nt_client_team = 1111;
	// _config.lidar_offset_xyz[2] = 7.5f;
	_config.min_scan_theta_degrees = -90.f;
	_config.max_scan_theta_degrees = 90.f;
	_config.map_resolution_cm = 10.f;
	_config.max_filter_threads = 2;
	_config.pmf_max_window_size_cm = 48.f;
	_config.pose_matching_history_range_s = 0.25;
	_config.pose_matching_max_delta_s = 0.01;
	_config.pose_matching_wait_increment_s = 0.008;
	_config.pose_matching_wait_limit_s = 0.035;
	_config.pose_matching_skip_invalid = false;

	s = ldrp::apiInit(_config);
	s = ldrp::lidarInit();
	// std::cout << "lidar inited from main" << std::endl;

	// nt::NetworkTableInstance::GetDefault().StartServer();
	nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();
	// nt::DoubleArrayEntry nt_localization = nt_inst.GetTable("rio telemetry")->GetSubTable("robot")->GetDoubleArrayTopic("pigeon rotation quat").GetEntry({});	// robot code sends doubles not floats!
	// nt::FloatArrayEntry nt_localization = nt_inst.GetFloatArrayTopic("uesim/pose").GetEntry({});
	nt::RawEntry nt_grid = nt_inst.GetRawTopic("tmain/obstacle grid").GetEntry("Grid<U8>", {});

	signal(SIGINT, _action);

	using namespace std::chrono_literals;
	using namespace std::chrono;
	float						// x    y    z    w
		pose[7] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	ldrp::ObstacleGrid grid{};
	// nt_localization.Set( {{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}} );
	for(;_program_running.load();) {

		// std::vector<nt::TimestampedDoubleArray> updates = nt_localization.ReadQueue();
		// std::vector<nt::TimestampedFloatArray> updates = nt_localization.ReadQueue();
		// std::cout << "[Main Thread]: Localization recieved " << updates.size() << " pose updates" << std::endl;
		// std::optional<int64_t> _dt = nt_inst.GetServerTimeOffset();
		// const int64_t dt = (_dt.has_value() ? *_dt : 0L);
		// std::cout << "Time off (micros): " << dt << std::endl;
		// if(updates.size() <= 0) {
		// 	updates.emplace_back(nt_localization.GetAtomic());
		// }
		// for(const auto& u : updates) {
		// 	const auto* _data = u.value.data();
		// 	pose[0] = static_cast<float>(_data[0]);
		// 	pose[1] = static_cast<float>(_data[1]);
		// 	pose[2] = static_cast<float>(_data[2]);
		// 	pose[3] = static_cast<float>(_data[4]);	// pigeon and wpilib internally use wxyz order, therefore we must reorder to xyzw
		// 	pose[4] = static_cast<float>(_data[5]);
		// 	pose[5] = static_cast<float>(_data[6]);
		// 	pose[6] = static_cast<float>(_data[3]);
		// 	ldrp::updateWorldPose(pose, pose + 3, u.time);	// previous: div by 100 to get into same timebase as internally, but THIS IS ACTUALLY IN MICROSECONDS
		// }

		// s = ldrp::updateWorldPose(pose, pose + 3);

#define NT_EXPORT_GRID
#ifdef NT_EXPORT_GRID
		high_resolution_clock::time_point a = high_resolution_clock::now();
		s = ldrp::waitNextObstacleGrid(grid, &_grid_alloc, 10.0);
		if(s == ldrp::STATUS_SUCCESS) grid.grid -= 20;
		// double dur = duration<double>{high_resolution_clock::now() - a}.count();
		if(s & ldrp::STATUS_TIMED_OUT) {
			// std::cout << "[Main Thread]: Obstacle export timed out after " << dur << " seconds." << std::endl;
		} else if(s == ldrp::STATUS_SUCCESS) {
			// std::cout << "[Main Thread]: Obstacle export succeeded after " << dur << " seconds." << std::endl;
			// std::cout << "\t>> grid origin: (" << grid.origin_x_m << ", " << grid.origin_y_m << "), grid size: {" << grid.cells_x << " x " << grid.cells_y << "}" << std::endl;
		} else {
			// std::cout << "[Main Thread]: Obstacle export failed after " << dur << " seconds." << std::endl;
		}
		if(grid.grid) {
			reinterpret_cast<int32_t*>(grid.grid)[0] = grid.cells_x;
			reinterpret_cast<int32_t*>(grid.grid)[1] = grid.cells_y;
			reinterpret_cast<float*>(grid.grid)[2] = grid.origin_x_m;
			reinterpret_cast<float*>(grid.grid)[3] = grid.origin_y_m;
			reinterpret_cast<float*>(grid.grid)[4] = grid.cell_resolution_m;
			nt_grid.Set(
				std::span<const uint8_t>{
					grid.grid,
					grid.grid + (grid.cells_x * grid.cells_y + 20)
				}
			);
			free(grid.grid);
			grid.grid = nullptr;
		}
#else
		std::this_thread::sleep_for(1ms);
#endif

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
