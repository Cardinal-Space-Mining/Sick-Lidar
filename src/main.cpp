#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <stdio.h>
#include <fstream>

#include <networktables/NetworkTableInstance.h>
#include <networktables/FloatArrayTopic.h>

#include "lidar_api.h"
#include "./core/mem_utils.hpp"


// namespace bmp {

// 	constexpr int BYTES_PER_PIXEL = 1; /// red, green, & blue
// 	constexpr int FILE_HEADER_SIZE = 14;
// 	constexpr int INFO_HEADER_SIZE = 40;

// 	unsigned char* createBitmapFileHeader(int height, int stride);
// 	unsigned char* createBitmapInfoHeader(int height, int width);


// 	void generateBitmapImage (unsigned char* image, int height, int width, char* imageFileName)
// 	{
// 		int widthInBytes = width * BYTES_PER_PIXEL;

// 		unsigned char padding[3] = {0, 0, 0};
// 		int paddingSize = (4 - (widthInBytes) % 4) % 4;

// 		int stride = (widthInBytes) + paddingSize;

// 		FILE* imageFile = fopen(imageFileName, "wb");

// 		unsigned char* fileHeader = createBitmapFileHeader(height, stride);
// 		fwrite(fileHeader, 1, FILE_HEADER_SIZE, imageFile);

// 		unsigned char* infoHeader = createBitmapInfoHeader(height, width);
// 		fwrite(infoHeader, 1, INFO_HEADER_SIZE, imageFile);

// 		int i;
// 		for (i = 0; i < height; i++) {
// 			fwrite(image + (i*widthInBytes), BYTES_PER_PIXEL, width, imageFile);
// 			fwrite(padding, 1, paddingSize, imageFile);
// 		}

// 		fclose(imageFile);
// 	}

// 	unsigned char* createBitmapFileHeader (int height, int stride)
// 	{
// 		int fileSize = FILE_HEADER_SIZE + INFO_HEADER_SIZE + (stride * height);

// 		static unsigned char fileHeader[] = {
// 			0,0,     /// signature
// 			0,0,0,0, /// image file size in bytes
// 			0,0,0,0, /// reserved
// 			0,0,0,0, /// start of pixel array
// 		};

// 		fileHeader[ 0] = (unsigned char)('B');
// 		fileHeader[ 1] = (unsigned char)('M');
// 		fileHeader[ 2] = (unsigned char)(fileSize      );
// 		fileHeader[ 3] = (unsigned char)(fileSize >>  8);
// 		fileHeader[ 4] = (unsigned char)(fileSize >> 16);
// 		fileHeader[ 5] = (unsigned char)(fileSize >> 24);
// 		fileHeader[10] = (unsigned char)(FILE_HEADER_SIZE + INFO_HEADER_SIZE);

// 		return fileHeader;
// 	}

// 	unsigned char* createBitmapInfoHeader (int height, int width)
// 	{
// 		static unsigned char infoHeader[] = {
// 			0,0,0,0, /// header size
// 			0,0,0,0, /// image width
// 			0,0,0,0, /// image height
// 			0,0,     /// number of color planes
// 			0,0,     /// bits per pixel
// 			0,0,0,0, /// compression
// 			0,0,0,0, /// image size
// 			0,0,0,0, /// horizontal resolution
// 			0,0,0,0, /// vertical resolution
// 			0,0,0,0, /// colors in color table
// 			0,0,0,0, /// important color count
// 		};

// 		infoHeader[ 0] = (unsigned char)(INFO_HEADER_SIZE);
// 		infoHeader[ 4] = (unsigned char)(width      );
// 		infoHeader[ 5] = (unsigned char)(width >>  8);
// 		infoHeader[ 6] = (unsigned char)(width >> 16);
// 		infoHeader[ 7] = (unsigned char)(width >> 24);
// 		infoHeader[ 8] = (unsigned char)(height      );
// 		infoHeader[ 9] = (unsigned char)(height >>  8);
// 		infoHeader[10] = (unsigned char)(height >> 16);
// 		infoHeader[11] = (unsigned char)(height >> 24);
// 		infoHeader[12] = (unsigned char)(1);
// 		infoHeader[14] = (unsigned char)(BYTES_PER_PIXEL*8);

// 		return infoHeader;
// 	}

// }


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
	_config.nt_use_client = true;
	// _config.lidar_offset_xyz[2] = 1.f;
	// _config.lidar_offset_quat[0] = 1.f;
	// _config.lidar_offset_quat[3] = 0.f;
	// _config.datalog_fname = "test_configs_import.wpilog";

	s = ldrp::apiInit(_config);
	s = ldrp::lidarInit();
	// std::cout << "lidar inited from main" << std::endl;

	// nt::NetworkTableInstance::GetDefault().StartServer();
	nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();
	nt::FloatArrayEntry nt_localization = nt_inst.GetFloatArrayTopic("uesim/pose").GetEntry({});

	signal(SIGINT, _action);

	using namespace std::chrono_literals;
	using namespace std::chrono;
	// float					// x    y    z    w
	// 	pose[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	ldrp::ObstacleGrid grid{};
	for(;_program_running.load();) {

		std::vector<nt::TimestampedFloatArray> updates = nt_localization.ReadQueue();
		std::cout << "[Main Thread]: Localization recieved " << updates.size() << " pose updates" << std::endl;
		for(const nt::TimestampedFloatArray& u : updates) {
			ldrp::updateWorldPose(u.value.data(), u.value.data() + 3, u.time);
		}

		// s = ldrp::updateWorldPose(pose, pose + 3);
		// pose[0] += 0.1;
		// if(grid.grid) free(grid.grid);
		// high_resolution_clock::time_point a = high_resolution_clock::now();
		s = ldrp::waitNextObstacleGrid(grid, &_grid_alloc, 10.0);
		// double dur = duration<double>{high_resolution_clock::now() - a}.count();
		// if(s & ldrp::STATUS_TIMED_OUT) {
		// 	std::cout << "[Main Thread]: Obstacle export timed out after " << dur << " seconds." << std::endl;
		// } else if(s == ldrp::STATUS_SUCCESS) {
		// 	std::cout << "[Main Thread]: Obstacle export succeeded after " << dur << " seconds." << std::endl;
		// 	std::cout << "\t>> grid origin: (" << grid.origin_x_m << ", " << grid.origin_y_m << "), grid size: {" << grid.cells_x << " x " << grid.cells_y << "}" << std::endl;
		// } else {
		// 	std::cout << "[Main Thread]: Obstacle export failed after " << dur << " seconds." << std::endl;
		// }
		// free(grid.grid);
		// grid.grid = nullptr;

		// std::this_thread::sleep_for(100ms);
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
	std::cout << "[Main Thread]: Exitting..." << std::endl;

}
