#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "lidar_api.h"
#include "./core/mem_utils.hpp"


static std::atomic<bool> _program_running = true;
static void _action(int sig) {
	std::cout << "Caught signal. Stopping program..." << std::endl;
	_program_running = false;
	// ldrp::lidarShutdown();
	// ldrp::apiDestroy();
}

int main(int argc, char** argv) {

	// test bit counting and indexing
// 	{
// 		const int64_t m = 0b0010110101001100011111101010100101101010010110100101001010100101ULL;
// 		const int32_t n = 0b10111111010101001011010100101101UL;
// 		const int8_t c = 0b10101011;
// 		const int16_t h = 0b1010101010101100;

// 		std::cout << ::countBits(m) << std::endl;
// 		std::cout << ::countBits(n) << std::endl;
// 		std::cout << ::countBits(c) << std::endl;
// 		std::cout << ::countBits(h) << std::endl;

// #define _VAR n
// 		for(size_t i = 0; i < sizeof(_VAR) * 8; i++) {
// 			if(_VAR & (1ULL << i)) {
// 				std::cout << "bit " << i << " enabled: index " << ::countBitsBeforeN(_VAR, i) << std::endl;
// 			}
// 		}
// 	}

	// std::cout << "Functional entrypoint!?" << std::endl;
	// std::cout << "Internally Linked PCL v" << ldrp::pclVer() << std::endl;

	using ldrp::status_t;

	status_t s{0};
	ldrp::LidarConfig _config{};
	// _config.lidar_offset_xyz[2] = 1.f;
	// _config.lidar_offset_quat[0] = 1.f;
	// _config.lidar_offset_quat[3] = 0.f;
	// _config.datalog_fname = "test_configs_import.wpilog";

	s = ldrp::apiInit(_config);
	s = ldrp::lidarInit();


// #ifdef WIN32
	signal(SIGINT, _action);
	signal(SIGILL, _action);
	signal(SIGFPE, _action);
	signal(SIGSEGV, _action);
	signal(SIGTERM, _action);
	// signal(SIGBREAK, _action);
	signal(SIGABRT, _action);
// #else
// 	struct sigaction _sig_action;
// 	sigemptyset(&(_sig_action.sa_mask));
// 	_sig_action.sa_flags = SA_SIGINFO;
// 	_sig_action.sa_sigaction = _action;
// 	sigaction(/* signal numbers here */, &_sig_action, nullptr);
// #endif

	using namespace std::chrono_literals;
	// std::this_thread::sleep_for(3s);
	float					// x    y    z    w
		pose[] = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f };
	for(;_program_running.load();) {
		pose[0] += 0.1;
		s = ldrp::updateWorldPose(pose);
		// std::cout << "main thread is still running!?" << std::endl;
		std::this_thread::sleep_for(100ms);
	}

	/* Notes on weird exit behavior (windows testing)
	 * - the main thread seems to stall on one of the SickScanApi**() calls within the sickDeinit() internal call
	 * - manually skipping the deinit lets the main thread and program terminate gracefully (except that the sick api is not destroyed gracefully)
	 * - what to do???? ha thats hillarious :|
	*/
	std::cout << "shutting down lidar processing...?" << std::endl;
	s = ldrp::lidarShutdown();
	s = ldrp::apiDestroy();
	std::cout << "main thread exitting!?" << std::endl;
	// exit(0);

}
