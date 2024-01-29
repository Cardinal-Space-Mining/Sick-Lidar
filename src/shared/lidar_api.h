#pragma once

#include <cstdint>
#include <memory>
#include <ostream>

#ifdef WIN32
#ifdef _LIB_SOURCE
#define __API __declspec(dllexport)
#else
#define __API __declspec(dllimport)
#endif
#else
#define __API
#endif


// "LiDaR Proccessing container namespace"
namespace ldrp {

	using LidarApi = std::shared_ptr<void>;
	using status_t = int32_t;

	enum : status_t {
		STATUS_SUCCESS				= (0 << 0),
		STATUS_FAIL					= (1 << 0),
		STATUS_EXTERNAL_ERROR		= (1 << 1),
		STATUS_ALREADY_INITIALIZED	= (1 << 2),
		STATUS_BAD_PARAM			= (1 << 3),
		STATUS_IO_ERROR				= (1 << 4),
		STATUS_BUSY_MUTEX			= (1 << 5),
		STATUS_PREREQ_UNINITIALIZED	= (1 << 6),
	};


	struct Pose3 {
		union {
			struct {
				float
					x, y, z, _w,
					qx, qy, qz, qw;		// change to wxyz ordering if that is more popular externally
			};
			struct {
				float
					xyz[4],
					quat[4];
			};
			float data[8];	// since the struct will be most likely aligned to 8-bytes anyway
		};
	};

	struct FilterParams {
		float
			min_scan_theta,
			max_scan_theta,
			voxel_size_cm,
			map_resolution_cm,
			pmf_window_base_cm,
			pmf_cell_size_cm,
			pmf_init_distance_cm,
			pmf_max_distance_cm,
			pmf_slope;
		int32_t
			pmf_max_window_size;
	};
	// struct PointBuff {

	// };
	// struct MapBuff {

	// };



/** API */

	/** Get the pcl version string of the currently linked library */
	__API char const* pclVer();

	/** Initialize the global and sick api instances -- optional params are passed directly to SickScanApiCreate when called */
	__API const status_t apiInit(int ss_argc = 0, char** ss_argv = nullptr);
	/** Reset the internal sick api instance. Does not delete buffers or filter params */
	__API const status_t apiClose();
	/** Intialize the lidar connection via config file */
	__API const status_t lidarInit(const char* config_file);
	/** Deregister the currently loaded lidar and stop all filter processes */
	__API const status_t lidarClose();

	/** Specify where output messages should be sent */
	__API const status_t setOutput(std::ostream& out);
	/** Specify the logging level: 0 = none, 1 = standard, 2 = verbose, 3 = VERBOOOSE! */
	__API const status_t setLogLevel(const int32_t lvl);

	/** Enable or disable filtering and accumulation of point data */
	__API const status_t enableFiltering(const bool enable);
	/** Apply an upper limit frequency for how often filtering and accumulation occurs */
	__API const status_t setMaxFilterFrequency(const size_t f_hz);
	/** Apply parameters used in the filtering and accumulation pipeline */
	__API const status_t applyFilterParams(const FilterParams& params);

	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * The q** params represent quaternion components */
	__API const status_t updateWorldPose(const float* xyz, const float* qxyz, const float qw);
	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * The provided buffer is expected to contain an xyz position and quaternion component -
	 * the _** params represent the respective offsets of each component in the buffer */
	inline const status_t updateWorldPose(const float* pose3, const size_t _xyz = 0, const size_t _qw = 3, const size_t _qxyz = 4)
		{ return updateWorldPose(pose3 + _xyz, pose3 + _qxyz, pose3[_qw]); }
	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * This method assumes the quaternion components are ordered X, Y, Z, W */
	inline const status_t updateWorldPose(const float* xyz, const float* qxyzw)
		{ return updateWorldPose(xyz, qxyzw, qxyzw[3]); }

	/** Access the accumulator map */
	__API const status_t getAccumulator(std::shared_ptr<void>& out_inst);	// not finalized


};