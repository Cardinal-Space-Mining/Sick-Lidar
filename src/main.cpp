#include <iostream>
// #include <pcl/pcl_config.h>
// #include <sick_scan_xd_api/sick_scan_api.h>

#include "lidar_api.h"


int main(int argc, char** argv) {

	std::cout << "Functional entrypoint!?" << std::endl;
	// std::cout << "Linked PCL v" << PCL_VERSION_PRETTY << std::endl;

	// {
		const ldrp::ApiRef lapi = ldrp::create();
	// }

	// SickScanApiHandle handle = SickScanApiCreate(0, nullptr);
	// // SickScanApiInitByLaunchfile(handle, "");
	// std::cout << "Created SickScanAPI Handle: " << (intptr_t)handle << std::endl;
	// SickScanApiClose(handle);

}
