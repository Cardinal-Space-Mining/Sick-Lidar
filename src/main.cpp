#include <iostream>
#include <chrono>
#include <thread>

#include "lidar_api.h"


int main(int argc, char** argv) {

	std::cout << "Functional entrypoint!?" << std::endl;
	std::cout << "Internally Linked PCL v" << ldrp::pclVer() << std::endl;

	using ldrp::status_t;

	status_t s = ldrp::apiInit();
	s = ldrp::lidarInit("./sick_multiscan.launch");

	using namespace std::chrono_literals;
	for(;;) std::this_thread::sleep_for(5s);

	s = ldrp::apiClose();

	// SickScanApiHandle handle = SickScanApiCreate(0, nullptr);
	// // SickScanApiInitByLaunchfile(handle, "");
	// std::cout << "Created SickScanAPI Handle: " << (intptr_t)handle << std::endl;
	// SickScanApiClose(handle);

}
