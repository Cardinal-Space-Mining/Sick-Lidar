#pragma once

#include <cstdint>
#include <memory>

#ifdef WIN32
	#ifdef _LIB_SOURCE
		#define __API __declspec(dllexport)
	#else
		#define __API __declspec(dllimport)
	#endif
	#define __API_LOCAL
#else
	#if __GNUC__ >= 4
		#define __API __attribute__ ((visibility ("default")))
		#define __API_LOCAL __attribute__ ((visibility ("hidden")))
	#else
		#define __API
		#define __API_LOCAL
#endif
#endif


// "LiDaR Proccessing container namespace"
namespace ldrp {

	using status_t = int32_t;

	enum : status_t {
		STATUS_SUCCESS				= (0 << 0),		// success
		STATUS_FAIL					= (1 << 0),		// generic fail, often |'d with other stati
		STATUS_EXTERNAL_ERROR		= (1 << 1),		// an error occured external to this codebase (ex. in a thirdparty lib)
		STATUS_ALREADY_SATISFIED	= (1 << 2),		// the target state was already satisfied
		STATUS_BAD_PARAM			= (1 << 3),		// a parameter was not valid
		STATUS_IO_ERROR				= (1 << 4),		// self explainatory
		STATUS_BUSY_MUTEX			= (1 << 5),		// could not aquire mutex control
		STATUS_PREREQ_UNINITIALIZED	= (1 << 6),		// prerequisite resources not available or not initialized
	};


	// struct PipelineConfig {
	// 	float
	// 		min_scan_theta,
	// 		max_scan_theta,
	// 		voxel_size_cm,
	// 		map_resolution_cm,
	// 		pmf_window_base_cm,
	// 		pmf_cell_size_cm,
	// 		pmf_init_distance_cm,
	// 		pmf_max_distance_cm,
	// 		pmf_slope;
	// 	int32_t
	// 		pmf_max_window_size;
	// };



/** API */

	/** Get the pcl version string of the currently linked library */
	__API const char* pclVer();
	/** Get whether wpilib and supported functionality was compiled in */
	__API const bool hasWpilib();

	/** Initialize the global instance resources.
	 * @param dlm_dir - the log directory for the internal [wpi::]DataLogManager
	 * @param dlm_fname - the name of the log file exported by the internal [wpi::]DataLogManager
	 * @param dlm_period - the time between file writes for the internal [wpi::DataLogManager]
	 * @param log_lvl - the maximum log level that will be output: 0 = none, 1 = standard, 2 = verbose, 3 = VERBOOOSE! */
	__API const status_t apiInit(
		const char* dlm_dir = "", const char* dlm_fname = "", double dlm_period = 0.05,
		const int32_t log_lvl = 1);

	/** Deletes the global api instance. Does not need to be called unless a hard reset is necessary. */
	__API const status_t apiDestroy();
	/** Spools up lidar processing resources and threads. */
	__API const status_t lidarInit();
	/** Stops and joins lidar processing threads. */
	__API const status_t lidarShutdown();
	/** Calls lidarInit() or lidarShutdown() depending on the given state. */
	__API const status_t lidarSetState(const bool enabled);

	/** Specify the logging level: 0 = none, 1 = standard, 2 = verbose, 3 = VERBOOOSE! */
	__API const status_t setLogLevel(const int32_t lvl);

	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * Note that the position is internally interpreted as being in meters! The q** params represent quaternion components. */
	__API const status_t updateWorldPose(const float* xyz, const float* qxyz, const float qw, const uint64_t ts_microseconds = 0);
	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * The provided buffer is expected to contain an xyz position and quaternion component -
	 * the _** params represent the respective offsets of each component in the buffer */
	inline const status_t updateWorldPose(const float* pose3, const size_t _xyz = 0, const size_t _qxyz = 3, const size_t _qw = 6, const uint64_t ts_microseconds = 0)
		{ return updateWorldPose(pose3 + _xyz, pose3 + _qxyz, pose3[_qw], ts_microseconds); }
	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * This method assumes the quaternion components are ordered X, Y, Z, W */
	inline const status_t updateWorldPose(const float* xyz, const float* qxyzw, const uint64_t ts_microseconds = 0)
		{ return updateWorldPose(xyz, qxyzw, qxyzw[3], ts_microseconds); }

	/** Access the accumulator map */
	__API const status_t getAccumulator(std::shared_ptr<void>& out_inst);	// not finalized


};