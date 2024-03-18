/** Copyright (c) Cardinal Space Mining 2024 */

#pragma once

#include <cstdint>
#include <cstddef>

#ifdef WIN32
	#ifdef _LDRP_SOURCE
		#define __API __declspec(dllexport)
	#elif _LDRP_DYNAMIC_INTERFACE		// defined by the client if linking dynamically and +0.00001% speed increase is desired
		#define __API __declspec(dllimport)
	#else
		#define __API
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
		STATUS_NOT_IMPLEMENTED		= (1 << 7),		// the underlying functionality has not been implemented yet or was disabled during compile time
		STATUS_TIMED_OUT			= (1 << 8),		// a timeout occurred
	};

	enum : uint64_t {
		POINT_LOGGING_NONE		= 0,
		POINT_LOGGING_TAR		= (1 << 0),
		POINT_LOGGING_NT		= (1 << 1),
		POINT_LOGGING_ALL		= (POINT_LOGGING_TAR | POINT_LOGGING_NT),	// not recommended

		POINT_LOGGING_INCLUDE_RAW		= (1 << 10),
		POINT_LOGGING_INCLUDE_VOXELIZED	= (1 << 11),	// not implemented
		POINT_LOGGING_INCLUDE_FILTERED	= (1 << 12),
		POINT_LOGGING_INCLUDE_ALL		= (POINT_LOGGING_INCLUDE_RAW | POINT_LOGGING_INCLUDE_FILTERED),
	};


	struct LidarConfig {

		bool nt_use_client{ false };
		unsigned int nt_client_port{ 0 };	// 0 uses default port
		int nt_client_team{ -1 };			// does not apply if < 0
		const char* nt_client_server{ "localhost" };

		const char* datalog_subdirectory{ "" };
		const char* datalog_fname{ "lidar_log.wpilog" };
		double datalog_flush_period_s{ 0.05 };
		int32_t log_level{ 1 };					// a default value -- can be changed during runtime

		const char* lidar_hostname{ "" };
		int lidar_udp_port{ 2115 };
		bool use_msgpack{ false };

		uint64_t enabled_segments_bits{ 0b111111111111 };
		uint32_t buffered_scan_frames{ 1 };
		int32_t max_filter_threads{ -1 };		// 0 = use all available, <0 = however many less than std::thread::hardware_concurrency() - 1
		uint64_t points_logging_mode{ POINT_LOGGING_NT | POINT_LOGGING_INCLUDE_ALL };
		const char* points_tar_fname{ "lidar_points.tar" };
		double pose_history_period_s{ 0.25 };
		bool skip_invalid_transform_ts{ false };

		uint32_t
			obstacle_point_color = 0x0011DD00U,
			standard_point_color = 0x00E82E88U;

		float
			min_scan_theta_degrees	= -90.f,
			max_scan_theta_degrees	= 90.f,
			min_scan_range_cm		= 10.f,
			max_pmf_range_cm		= 200.f,
			max_z_thresh_cm			= 150.f,
			min_z_thresh_cm			= 25.f,
			voxel_size_cm			= 3.f,
			map_resolution_cm		= 5.f,
			pmf_window_base			= 2.f,
			pmf_max_window_size_cm	= 45.f,
			pmf_cell_size_cm		= 5.f,
			pmf_init_distance_cm	= 5.f,
			pmf_max_distance_cm		= 12.f,
			pmf_slope				= 2.f;

		float
			lidar_offset_xyz[3]		= { 0.f, 0.f, 0.f },
			lidar_offset_quat[4]	= { 0.f, 0.f, 0.f, 1.f };	// x, y, z, w


		static const LidarConfig
			STATIC_DEFAULT;

	};

	template<typename Quant_t = uint8_t>
	struct ObstacleGrid_ {
		using Quant_T = Quant_t;	// "quantization type"

		int32_t
			cells_x = 0,
			cells_y = 0;
		float
			origin_x_m = 0.f,
			origin_y_m = 0.f,
			cell_resolution_m = 1.f;
		Quant_T*
			grid = nullptr;

	};
	using ObstacleGrid = ObstacleGrid_<>;



/** API */

	// /** Get the pcl version string of the currently linked library */
	// __API const char* pclVer();
	// /** Get whether wpilib and supported functionality was compiled in */
	// __API bool hasWpilib();

	/** Initialize the global instance resources.
	 * @param config -- the struct containing all the configs for the lidar instance */
	__API status_t apiInit(const LidarConfig& config = LidarConfig::STATIC_DEFAULT);
	/** Deletes the global api instance. Does not need to be called unless a hard reset is necessary. */
	__API status_t apiDestroy();
	/** Spools up lidar processing resources and threads. */
	__API status_t lidarInit();
	/** Stops and joins lidar processing threads. */
	__API status_t lidarShutdown();
	/** Calls lidarInit() or lidarShutdown() depending on the given state. */
	__API status_t lidarSetState(const bool enabled);

	/** Specify the logging level: 0 = none, 1 = standard, 2 = verbose, 3 = VERBOOOSE! */
	__API status_t setLogLevel(const int32_t lvl);

	/** Apply a new world pose for the lidar -- used directly to transform points to world space.
	 * Note that the position is interpreted as being in meters!
	 * @param xyz - the (x, y, z) position as a float array (pointer to any contiguous float buffer)
	 * @param qxyz - the quaternion representing the robot's rotation in world space -- ordered XYZW!
	 * @param ts_microseconds - the timestamp in microseconds (relative to epoch) when the pose was collected */
	__API status_t updateWorldPose(const float* xyz, const float* qxyzw, const uint64_t ts_microseconds = 0);

	/** Export the current obstacle grid.
	 * @param grid - struct to which all grid data will be exported
	 * @param grid_resize - function pointer which provides a buffer of at least the size passed in	*/
	__API status_t getObstacleGrid(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t));
	/** Wait until the internal accumulator gets updated (relative to last call of this function) and export the obstacle grid or unblock after the given timeout period.
	 * @param grid - struct to which all grid data will be exported
	 * @param grid_resize - function pointer which provides a buffer of at least the size passed in
	 * @param timeout_ms - the timeout, in milliseconds, after which the function will unblock if no new data is available */
	__API status_t waitNextObstacleGrid(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), double timeout_ms);


};
