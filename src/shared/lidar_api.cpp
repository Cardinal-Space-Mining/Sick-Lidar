#include "lidar_api.h"

#include <iostream>
#include <memory>


namespace ldrp {

	struct LidarImpl {
	public:
		LidarImpl() {
			std::cout << "Lidar Internal API Created." << std::endl;
		}
		~LidarImpl() {
			std::cout << "Lidar Internal API Destroyed." << std::endl;
		}
	};

	const ApiRef create() {
		return std::static_pointer_cast<void>(std::make_shared<LidarImpl>());
	}

};