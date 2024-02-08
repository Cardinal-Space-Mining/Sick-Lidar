#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include "lidar_api.h"


static std::atomic<bool> _program_running = true;
static void _action(int sig) {
	std::cout << "Caught signal. Stopping program..." << std::endl;
	_program_running = false;
	ldrp::lidarShutdown();
	ldrp::apiDestroy();
}

int main(int argc, char** argv) {

	// std::cout << "Functional entrypoint!?" << std::endl;
	// std::cout << "Internally Linked PCL v" << ldrp::pclVer() << std::endl;

	using ldrp::status_t;

	status_t s{0};
	s = ldrp::apiInit("", "lidar_log.wpilog");
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
	for(;_program_running.load();) {
		// std::cout << "main thread is still running!?" << std::endl;
		std::this_thread::sleep_for(1s);
	}

	/* Notes on weird exit behavior (windows testing)
	 * - the main thread seems to stall on one of the SickScanApi**() calls within the sickDeinit() internal call
	 * - manually skipping the deinit lets the main thread and program terminate gracefully (except that the sick api is not destroyed gracefully)
	 * - what to do???? ha thats hillarious :|
	*/
	// s = ldrp::lidarShutdown();
	// s = ldrp::apiDestroy();
	std::cout << "main thread exitting!?" << std::endl;
	// exit(0);

}
