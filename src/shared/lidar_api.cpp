#include "lidar_api.h"

#include <iostream>
#include <memory>

#include <pcl/pcl_config.h>
#include <sick_scan_xd_api/sick_scan_api.h>


namespace ldrp {

	// TODO macros for logging

	static const status_t cvtStatus(const int32_t sick_ec) {
		switch(sick_ec) {
			case SICK_SCAN_API_SUCCESS:
				return STATUS_SUCCESS;
			case SICK_SCAN_API_NOT_LOADED:
			case SICK_SCAN_API_NOT_INITIALIZED:
			case SICK_SCAN_API_NOT_IMPLEMENTED:
				return STATUS_PREREQ_UNINITIALIZED | STATUS_EXTERNAL_ERROR;
			case SICK_SCAN_API_ERROR:
			case SICK_SCAN_API_TIMEOUT:
			default:
				return STATUS_EXTERNAL_ERROR;
		}
	}

	// void customizedPointCloudMsgCb(SickScanApiHandle apiHandle, const SickScanPointCloudMsg* msg)
	// {
	// 	std::cout << "C++ PointCloudMsgCb: " << msg->width << " x " << msg->height << " pointcloud message received" << std::endl; // data processing to be done
	// }

	/** Interfacing and Filtering Implementation */
	struct LidarImpl {
	public:
		// template<typename T>
		// static inline typename std::conditional<std::is_same<T, void>::value,
		// 	std::shared_ptr<LidarImpl>,
		// 	std::shared_ptr<void>
		// >::type swap_cast(const std::shared_ptr<T>& in) {
		// 	static_assert(std::is_same<T, void>::value || std::is_same<T, LidarImpl>::value, "Only void and LidarImpl pointers can be swapped between!");
		// 	return std::static_pointer_cast<typename std::conditional<std::is_same<T, void>::value, LidarImpl, void>::type>(in);
		// }

	public:
		LidarImpl() {
			if(this->_log_level > 0)
				(*this->_log_output) << "LDRP global instance initialized." << std::endl;
		}
		~LidarImpl() {
			this->sickDeinit();
			if(this->_log_level > 0)
				(*this->_log_output) << "LDRP global instance destroyed." << std::endl;
		}
		// LidarImpl(int ss_argc = 0, char** ss_argv = nullptr) :
		// 	_handle(SickScanApiCreate(ss_argc, ss_argv))
		// {
		// 	std::cout << "Lidar Internal API Created." << std::endl;
		// }
		// ~LidarImpl() {
		// 	std::cout << "Lidar Internal API Destroyed." << std::endl;

		// 	this->_sick_status = SickScanApiDeregisterCartesianPointCloudMsg(this->_handle, nullptr);
		// 	this->_sick_status = SickScanApiClose(this->_handle);
		// 	this->_sick_status = SickScanApiRelease(this->_handle);
		// 	this->_handle = nullptr;
		// }


		// inline int32_t getSickStatus() const {
		// 	return this->_sick_status;
		// }

		// void lidarInit(const char* file) {
		// 	this->_sick_status = SickScanApiInitByLaunchfile(this->_handle, file);
		// 	this->_sick_status = SickScanApiRegisterCartesianPointCloudMsg(this->_handle, nullptr);
		// }

		const status_t sickInit(int ss_argc = 0, char** ss_argv = nullptr) {
			this->sickDeinit();
			this->_handle = SickScanApiCreate(ss_argc, ss_argv);
			if(this->_handle) {
				if(this->_log_level > 0)
					(*this->_log_output) << "Sick Scan API created successfully: " << this->_handle << std::endl;
				return STATUS_SUCCESS;
			}
			if(this->_log_level > 0)
				(*this->_log_output) << "Sick Scan API creation failed: " << this->_handle << std::endl;
			return STATUS_EXTERNAL_ERROR;
		}
		const status_t sickDeinit() {
			if(this->_handle) {
				status_t s = STATUS_SUCCESS;
				s |= cvtStatus(this->_sick_status = SickScanApiDeregisterCartesianPointCloudMsg(this->_handle, &cartesianPointCloudCallbackWrapper));	// keep track if we have an active lidar!?
				if(this->_log_level > 1 && this->_sick_status)
					(*this->_log_output) << "SickScanApiDeregisterCartesianPointCloudMsg error: " << this->_sick_status << std::endl;	// make a macro for this that can be compiled out if needed
				s |= cvtStatus(this->_sick_status = SickScanApiClose(this->_handle));
				if(this->_log_level > 1 && this->_sick_status)
					(*this->_log_output) << "SickScanApiClose error: " << this->_sick_status << std::endl;
				s |= cvtStatus(this->_sick_status = SickScanApiRelease(this->_handle));
				if(this->_log_level > 1 && this->_sick_status)
					(*this->_log_output) << "SickScanApiRelease error: " << this->_sick_status << std::endl;
				this->_handle = nullptr;
				return s;
			}
			return STATUS_PREREQ_UNINITIALIZED;
		}

	protected:
		SickScanApiHandle _handle{ nullptr };
		int32_t _sick_status{ 0 };

	public:
		std::ostream* _log_output{ &std::cout };
		int32_t _log_level{ 1 };

	public:
		inline static std::shared_ptr<LidarImpl> _global{ nullptr };

		static void cartesianPointCloudCallbackWrapper(SickScanApiHandle handle, const SickScanPointCloudMsg* msg) {
			// call something on the global instance!
		}


	};




	/** Static API */

	char const* pclVer() {
		return PCL_VERSION_PRETTY;
	}

	const status_t apiInit(int ss_argc, char** ss_argv) {
		if(!LidarImpl::_global) {
			LidarImpl::_global = std::make_shared<LidarImpl>();
		}
		return LidarImpl::_global->sickInit(ss_argc, ss_argv);
	}
	const status_t apiClose() {
		if(LidarImpl::_global) {
			return LidarImpl::_global->sickDeinit();
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarInit(const char* config_file) {
		if(LidarImpl::_global) {
			// LidarImpl::_global->lidarInit(config_file);	// TODO
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarClose() {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}

	const status_t setOutput(std::ostream& out) {
		if(LidarImpl::_global) {
			LidarImpl::_global->_log_output = &out;
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t setLogLevel(const int32_t lvl) {
		if(lvl < 0 || lvl > 3) return STATUS_BAD_PARAM;
		if(LidarImpl::_global) {
			LidarImpl::_global->_log_level = lvl;
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}

	const status_t enableFiltering(const bool enable) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t setMaxFilterFrequency(const size_t f_hz) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t applyFilterParams(const FilterParams& params) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}

	const status_t updateWorldPose(const float* xyz, const float* qxyz, const float qw) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}

	const status_t getAccumulator(std::shared_ptr<void>& out_inst) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}



};