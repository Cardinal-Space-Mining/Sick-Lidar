#include "lidar_api.h"

#include <iostream>
#include <memory>

#include <pcl/pcl_config.h>
#include <sick_scan_xd_api/sick_scan_api.h>


namespace ldrp {

	void customizedPointCloudMsgCb(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
	{
	std::cout << "C++ PointCloudMsgCb: " << msg->width << " x " << msg->height << " pointcloud message received" << std::endl; // data processing to be done
	}

	/** Interfacing and Filtering Implementation */
	struct LidarImpl {
	public:
		template<typename T>
		static inline typename std::conditional<std::is_same<T, void>::value,
			std::shared_ptr<LidarImpl>,
			std::shared_ptr<void>
		>::type swap_cast(const std::shared_ptr<T>& in) {
			static_assert(std::is_same<T, void>::value || std::is_same<T, LidarImpl>::value, "Only void and LidarImpl pointers can be swapped between!");
			return std::static_pointer_cast<typename std::conditional<std::is_same<T, void>::value, LidarImpl, void>::type>(in);
		}

	public:
		LidarImpl(int ss_argc = 0, char** ss_argv = nullptr) //:
			// _handle(SickScanApiCreate(ss_argc, ss_argv))
		{
			// this->_sick_status = SickScanApiLoadLibrary("libsick_scan_xd_shared_lib.so");
			this->_handle = SickScanApiCreate(ss_argc, ss_argv);

			std::cout << "Lidar Internal API Created." << std::endl;
		}
		~LidarImpl() {
			std::cout << "Lidar Internal API Destroyed." << std::endl;

			this->_sick_status = SickScanApiDeregisterCartesianPointCloudMsg(this->_handle, &customizedPointCloudMsgCb);
			this->_sick_status = SickScanApiClose(this->_handle);
			this->_sick_status = SickScanApiRelease(this->_handle);
			// this->_sick_status = SickScanApiUnloadLibrary();
			this->_handle = nullptr;
		}


		inline int32_t getSickStatus() const {
			return this->_sick_status;
		}

		void lidarInit(const char* file) {
			this->_sick_status = SickScanApiInitByLaunchfile(this->_handle, file);
			this->_sick_status = SickScanApiRegisterCartesianPointCloudMsg(this->_handle, &customizedPointCloudMsgCb);
		}

		static void sickCallback(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg) {
			std::cout << "Callback Recieved!?!?" << std::endl;
		}

	protected:
		SickScanApiHandle _handle{ nullptr };
		int32_t _sick_status{ 0 };


	};




	/** Static API */

	char const* pclVer() {
		return PCL_VERSION_PRETTY;
	}

	const LidarApi create() {
		return LidarImpl::swap_cast(std::make_shared<LidarImpl>());
	}
	void lidarInit(LidarApi l, const char* file) {
		LidarImpl::swap_cast(l)->lidarInit(file);
	}

	int32_t internalStatus(LidarApi l) {
		return LidarImpl::swap_cast(l)->getSickStatus();
	}


};