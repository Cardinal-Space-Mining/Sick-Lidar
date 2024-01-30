#include "lidar_api.h"

#include <iostream>
#include <memory>

#include <pcl/pcl_config.h>
#include <sick_scan_xd_api/sick_scan_api.h>


namespace ldrp {

#ifndef _LDRP_DISABLE_LOG
#define LDRP_LOG_GLOBAL(__inst, __condition, __output)		if(__condition) { (*__inst->_log_output) << __output; }
#define LDRP_LOG_S_GLOBAL(__inst, __condition, __output)	LDRP_LOG_GLOBAL(__inst, (__inst->_log_output != nullptr && (__condition)), __output)
#define LOG_LEVEL_GLOBAL(__inst, __min_lvl)					(__inst->_log_level >= __min_lvl)
#define LDRP_LOG(__condition, __output)		LDRP_LOG_GLOBAL(this, __condition, __output)
#define LDRP_LOG_S(__condition, __output)	LDRP_LOG_S_GLOBAL(this, __condition, __output)
#define LOG_LEVEL(__min_lvl)				LOG_LEVEL_GLOBAL(this, __min_lvl)
#define LOG_NONE				true
#define LOG_STANDARD			LOG_LEVEL(1)
#define LOG_VERBOSE				LOG_LEVEL(2)
#else
#define LDRP_LOG_GLOBAL(...)
#define LDRP_LOG_S_GLOBAL(...)
#define LOG_LEVEL_GLOBAL(...)
#define LDRP_LOG(...)
#define LDRP_LOG_S(...)
#define LOG_LEVEL(...)
#define LOG_NONE
#define LOG_STANDARD
#define LOG_VERBOSE
#endif

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
			LDRP_LOG_S( LOG_NONE, "LDRP global instance initialized." << std::endl )
		}
		~LidarImpl() {
			this->sickDeinit();
			LDRP_LOG_S( LOG_NONE, "LDRP global instance destroyed." << std::endl )
		}

		// template<typename arg_T, typename... args_T>
		// inline void log(bool condition, arg_T a, args_T... args) {
		// 	if(condition) {
		// 		(*this->_log_output) << a;
		// 		log<args_T>(args);
		// 	}
		// }
		// template<typename arg_T, typename... args_T>
		// inline void log(arg_T a, args_T... args) {
		// 	(*this->_log_output) << a;
		// 	if constexpr(sizeof...(args_T) > 0) log<args_T>(args);
		// }


		const status_t sickInit(int ss_argc = 0, char** ss_argv = nullptr) {
			this->sickDeinit();
			this->_handle = SickScanApiCreate(ss_argc, ss_argv);
			if(this->_handle) {
				LDRP_LOG( LOG_STANDARD, "Sick Scan API created successfully: " << this->_handle << std::endl )
				return STATUS_SUCCESS;
			}
			LDRP_LOG( LOG_STANDARD, "Sick Scan API creation failed: " << this->_handle << std::endl )
			return STATUS_EXTERNAL_ERROR | STATUS_FAIL;
		}
		const status_t sickDeinit() {
			if(this->_handle) {
				status_t s = STATUS_SUCCESS;
				s |= this->lidarClose() & ~(STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL);		// mask uninited fail since this the meaning isn't relevant to the outer function
				s |= cvtStatus( this->_sick_status = SickScanApiClose(this->_handle) );
				LDRP_LOG( LOG_STANDARD && this->_sick_status,
					"SickScanApiClose error: " << this->_sick_status << std::endl )
				s |= cvtStatus( this->_sick_status = SickScanApiRelease(this->_handle) );
				LDRP_LOG( LOG_STANDARD && this->_sick_status,
					"SickScanApiRelease error: " << this->_sick_status << std::endl )
				this->_handle = nullptr;
				return s;
			}
			return STATUS_PREREQ_UNINITIALIZED;
		}

		const status_t lidarInit(const char* config_file) {
			if(this->_handle) {
				status_t s = STATUS_SUCCESS;
				s |= cvtStatus( this->_sick_status = SickScanApiInitByLaunchfile(this->_handle, config_file) );
				if(!s) {
					s |= cvtStatus( this->_sick_status = SickScanApiRegisterCartesianPointCloudMsg(this->_handle, &cartesianPointCloudCallbackWrapper) );
					LDRP_LOG( LOG_STANDARD && this->_sick_status, "SickScanApiRegisterCartesianPointCloudMsg error: " << this->_sick_status << std::endl)
				} else {
					LDRP_LOG( LOG_STANDARD, "SickScanApiInitByLaunchfile error: " << this->_sick_status << " with config file: " << config_file << std::endl)
				}
				return s;
			}
			return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
		}
		const status_t lidarClose() {
			if(this->_handle) {
				if( this->_sick_status = SickScanApiDeregisterCartesianPointCloudMsg(this->_handle, &cartesianPointCloudCallbackWrapper) ) {
					LDRP_LOG( LOG_STANDARD, "SickScanApiDeregisterCartesianPointCloudMsg error: " << this->_sick_status << std::endl )
					return cvtStatus(this->_sick_status) | STATUS_FAIL;
				}
				return STATUS_SUCCESS;
			}
			return STATUS_PREREQ_UNINITIALIZED;
		}

	protected:
		SickScanApiHandle _handle{ nullptr };
		int32_t _sick_status{ 0 };

	public:
		std::ostream* _log_output{ &std::cout };
		int32_t _log_level{ 1 };

		struct {
			
		} _processing;

	public:
		inline static std::unique_ptr<LidarImpl> _global{ nullptr };

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
			LidarImpl::_global = std::make_unique<LidarImpl>();
		}
		return LidarImpl::_global->sickInit(ss_argc, ss_argv);
	}
	const status_t apiClose() {
		if(LidarImpl::_global) {
			return LidarImpl::_global->sickDeinit();
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t apiHardReset() {
		if(LidarImpl::_global) {
			const status_t s = LidarImpl::_global->sickDeinit();
			delete LidarImpl::_global.release();
			return s;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarInit(const char* config_file) {
		if(LidarImpl::_global) {
			return LidarImpl::_global->lidarInit(config_file);
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarClose() {
		if(LidarImpl::_global) {
			return LidarImpl::_global->lidarClose();
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

	const status_t enablePipeline(const bool enable) {
		if(LidarImpl::_global) {
			// TODO
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t setMaxFrequency(const size_t f_hz) {
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