#include "lidar_api.h"

#include "filtering.hpp"
#include "accumulator_grid2.hpp"
#include "mem_utils.hpp"
#include "pcd_streaming.h"

#include <condition_variable>
#include <shared_mutex>
#include <type_traits>
#include <algorithm>
#include <iostream>
#include <numbers>
#include <utility>
#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>
#include <vector>
#include <limits>
#include <deque>
#include <mutex>
#include <span>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#ifndef USING_WPILIB
  #define USING_WPILIB false
#endif

#if USING_WPILIB
#include <frc/geometry/Pose3d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/BooleanTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/RawTopic.h>
#include <networktables/FloatTopic.h>
#include <DataLogManager.h>
#include <wpi/DataLog.h>
#endif
#include <fmt/format.h>		// fmt is header only(?) so we should be able to pull in fmt as long as the wpilib submodule is present

#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/udp_receiver.h"
#include "sick_scansegment_xd/compact_parser.h"
#include "sick_scansegment_xd/msgpack_parser.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"
#include "sick_scan/sick_cloud_transform.h"


/** Make std:: namespaces more usable */
namespace std {
	namespace chrono {
		using hrc = high_resolution_clock;
	}
}
namespace crno = std::chrono;
using namespace std::chrono_literals;





/** Wpilib TimeInterpolatableBuffer helpers */

template<typename T>
static T&& lerpClosest(T&& a, T&& b, double t) {	// t is nominally in [0, 1]
	return (abs(t) < abs(1.0 - t)) ? std::forward<T>(a) : std::forward<T>(b);
}
#if USING_WPILIB
static frc::Pose3d lerpPose3d(const frc::Pose3d& a, const frc::Pose3d& b, double t) {	// t is nominally in [0, 1]
	return a.Exp(a.Log(b) * t);
}
#endif


/** Other utilities "LiDaR Utilities" */
namespace ldru {

/** Does not swap the 'CompactImuData' struct since it is fairly large and we do not use it. */
static void swapSegmentsNoIMU(sick_scansegment_xd::ScanSegmentParserOutput& a, sick_scansegment_xd::ScanSegmentParserOutput& b) {
	std::swap(a.scandata, b.scandata);
	std::swap(a.timestamp, b.timestamp);
#if true
	// address of end of struct minus address of starting data we want to copy
	const size_t bytes = (reinterpret_cast<uint8_t*>(&a) + sizeof(decltype(a))) - reinterpret_cast<uint8_t*>(&a.timestamp_sec);	// should be 16 bytes
	// constexpr size_t bytes = 16;
	void* tmp = malloc(bytes);
	memcpy(tmp, &a.timestamp_sec, bytes);
	memcpy(&a.timestamp_sec, &b.timestamp_sec, bytes);
	memcpy(&b.timestamp_sec, tmp, bytes);
	free(tmp);
#else	// safer alternative
	std::swap(a.timestamp_sec, b.timestamp_sec);
	std::swap(a.timestamp_nsec, b.timestamp_nsec);
	std::swap(a.segmentIndex, b.segmentIndex);
	std::swap(a.telegramCnt, b.telegramCnt);
#endif
}

static inline const uint32_t convertNumThreads(const int32_t input_num, const int32_t reserved = 0) {
	const int32_t _max = (int32_t)std::thread::hardware_concurrency() - reserved;
	return input_num < 1 ?
		(uint32_t)std::max(_max + input_num, 1) :
		(uint32_t)std::min(_max, input_num)
	;
}

};





/** Main interface namespace */

namespace ldrp {

#ifndef LOG_DEBUG
  #define LOG_DEBUG true
#endif
#ifndef LDRP_SAFETY_CHECKS
  #define LDRP_SAFETY_CHECKS true
#endif

#ifndef _LDRP_DISABLE_LOG
  #if USING_WPILIB
    #define LDRP_LOG(__condition, ...)		if(__condition) { wpi::DataLogManager::Log( fmt::format(__VA_ARGS__) ); }
  #else
    #define LDRP_LOG(__condition, ...)		if(__condition) { std::printf("%s\n", fmt::format(__VA_ARGS__) ); }		// DLM automatically appends newlines automatically
  #endif
  #define LOG_LEVEL_GLOBAL(__inst, __min_lvl)	(__inst->_state.log_level >= __min_lvl)
  #define LOG_LEVEL(__min_lvl)					LOG_LEVEL_GLOBAL(this, __min_lvl)
  #define LOG_ALWAYS			true
  #define LOG_STANDARD			LOG_LEVEL(1)
  #define LOG_VERBOSE			LOG_LEVEL(2)
#else
  #define LDRP_LOG_GLOBAL(...)
  #define LDRP_LOG_S_GLOBAL(...)
  #define LOG_LEVEL_GLOBAL(...)
  #define LDRP_LOG(...)
  #define LDRP_LOG_S(...)
  #define LOG_LEVEL(...)
  #define LOG_ALWAYS
  #define LOG_STANDARD
  #define LOG_VERBOSE
#endif


	const LidarConfig LidarConfig::STATIC_DEFAULT{};

	/** Interfacing and Filtering Implementation (singleton usage) */
	struct LidarImpl {
	public:
		LidarImpl(const LidarConfig& config = LidarConfig::STATIC_DEFAULT) :
			_state{ .log_level				= config.log_level },
			_config{

				.lidar_hostname				= config.lidar_hostname,
				.lidar_udp_port				= config.lidar_udp_port,
				.use_msgpack				= config.use_msgpack,
				.enabled_segments			= config.enabled_segments_bits,
				.buffered_frames			= config.buffered_scan_frames,
				.max_filter_threads			= ldru::convertNumThreads(config.max_filter_threads, 1),
				.points_logging_mode		= config.points_logging_mode,
				.points_log_fname			= config.points_tar_fname,
				.pose_history_range			= config.pose_history_period_s,
				.skip_invalid_transform_ts	= config.skip_invalid_transform_ts,

				.obstacle_point_color		= config.obstacle_point_color,
				.standard_point_color		= config.standard_point_color,

				.lidar_offset_xyz			= Eigen::Vector3f{ config.lidar_offset_xyz },
				.lidar_offset_quat			= Eigen::Quaternionf{ config.lidar_offset_quat },	// I no longer trust reinterpret_cast<> with Eigen::Quaternionf :|

				.fpipeline{
					.min_scan_theta_deg		= config.min_scan_theta_degrees,
					.max_scan_theta_deg		= config.max_scan_theta_degrees,
					.min_scan_range_cm		= config.min_scan_range_cm,
					.max_pmf_range_cm		= config.max_pmf_range_cm,
					.max_z_thresh_cm		= config.max_z_thresh_cm,
					.min_z_thresh_cm		= config.min_z_thresh_cm,
					.voxel_size_cm			= config.voxel_size_cm,
					.map_resolution_cm		= config.map_resolution_cm,
					.pmf_window_base		= config.pmf_window_base,
					.pmf_max_window_size_cm	= config.pmf_max_window_size_cm,
					.pmf_cell_size_cm		= config.pmf_cell_size_cm,
					.pmf_init_distance_cm	= config.pmf_init_distance_cm,
					.pmf_slope				= config.pmf_slope
				},

			}
		{
			wpi::DataLogManager::Start(
				config.datalog_subdirectory,
				config.datalog_fname,
				config.datalog_flush_period_s
			);
			this->initNT(config);

			LDRP_LOG( LOG_ALWAYS, "LDRP global instance initialized." )
		}
		~LidarImpl() {
			this->closeLidar();
			this->shutdownNT();

			LDRP_LOG( LOG_ALWAYS, "LDRP global instance destroyed." )
			wpi::DataLogManager::Stop();
		}

		LidarImpl(const LidarImpl&) = delete;
		LidarImpl(LidarImpl&&) = delete;


		/** launch the lidar processing thread */
		status_t startLidar() {
			if(!this->lidar_thread || !this->lidar_thread->joinable()) {
				this->_state.enable_threads.store(true);
				this->lidar_thread.reset(
					new std::thread{ &LidarImpl::lidarWorker, this }
				);
				return STATUS_SUCCESS;
			}
			return STATUS_ALREADY_SATISFIED;
		}
		/** join the lidar thread and release resources */
		status_t closeLidar() {
			status_t s = STATUS_ALREADY_SATISFIED;
			this->_state.enable_threads.store(false);
			if(this->lidar_thread && this->lidar_thread->joinable()) {
				if(this->udp_fifo) this->udp_fifo->Shutdown();		// break out of a possible infinite block when the scanner isn't connected
				this->lidar_thread->join();
				s = STATUS_SUCCESS;
			}
			this->lidar_thread.reset(nullptr);		// don't need to do this but probably good to explicitly signify the thread isn't valid
			return s;
		}

	protected:
		void initNT(const LidarConfig& config) {
			this->_nt.instance = nt::NetworkTableInstance::GetDefault();

			bool started_client = false;
			if(config.nt_use_client) {
				if(config.nt_client_server != nullptr) {
					this->_nt.instance.StartClient4("perception");
					this->_nt.instance.SetServer(
						config.nt_client_server,
						config.nt_client_port
					);
					started_client = true;
				}
				if(config.nt_client_team >= 0) {
					this->_nt.instance.StartClient4("perception");
					this->_nt.instance.SetServerTeam(
						static_cast<unsigned int>(config.nt_client_team),
						config.nt_client_port
					);
					started_client = true;
				}
			}
			if(!started_client) {
				this->_nt.instance.StartServer();	// config or auto-detect for server/client
			}
			this->_nt.base = this->_nt.instance.GetTable("Perception");

			this->_nt.last_parsed_seg_idx	= this->_nt.base->GetIntegerTopic("last segment").GetEntry(-1);
			this->_nt.aquisition_cycles		= this->_nt.base->GetIntegerTopic("aquisition loop count").GetEntry(0);
			this->_nt.aquisition_ftime		= this->_nt.base->GetDoubleTopic("aquisition frame time").GetEntry(0.0);
			this->_nt.raw_scan_points		= this->_nt.base->GetRawTopic("raw scan points").GetEntry( "PointXYZ_[]", {} );
			this->_nt.test_filtered_points	= this->_nt.base->GetRawTopic("filtered points").GetEntry( "PointXYZ_[]", {} );

#ifndef DISABLE_NT_TUNING
			this->_nt.tuning.max_pmf_range			= this->_nt.base->GetFloatTopic("tuning/pmf/max range (cm)").GetEntry( this->_config.fpipeline.max_pmf_range_cm );
			this->_nt.tuning.max_z_thresh			= this->_nt.base->GetFloatTopic("tuning/max z thresh (cm)").GetEntry( this->_config.fpipeline.max_z_thresh_cm );
			this->_nt.tuning.min_z_thresh			= this->_nt.base->GetFloatTopic("tuning/min z thresh (cm)").GetEntry( this->_config.fpipeline.min_z_thresh_cm );
			this->_nt.tuning.pmf_window_base		= this->_nt.base->GetFloatTopic("tuning/pmf/window base").GetEntry( this->_config.fpipeline.pmf_window_base );
			this->_nt.tuning.pmf_max_window_size	= this->_nt.base->GetFloatTopic("tuning/pmf/max window (cm)").GetEntry( this->_config.fpipeline.pmf_max_window_size_cm );
			this->_nt.tuning.pmf_cell_size			= this->_nt.base->GetFloatTopic("tuning/pmf/cell size (cm)").GetEntry( this->_config.fpipeline.pmf_cell_size_cm );
			this->_nt.tuning.pmf_init_distance		= this->_nt.base->GetFloatTopic("tuning/pmf/init distance (cm)").GetEntry( this->_config.fpipeline.pmf_init_distance_cm );
			this->_nt.tuning.pmf_max_distance		= this->_nt.base->GetFloatTopic("tuning/pmf/max distance (cm)").GetEntry( this->_config.fpipeline.pmf_max_distance_cm );
			this->_nt.tuning.pmf_slope				= this->_nt.base->GetFloatTopic("tuning/pmf/slope (cm)").GetEntry( this->_config.fpipeline.pmf_slope );

			this->_nt.tuning.max_pmf_range.Set( this->_config.fpipeline.max_pmf_range_cm );
			this->_nt.tuning.max_z_thresh.Set( this->_config.fpipeline.max_z_thresh_cm );
			this->_nt.tuning.min_z_thresh.Set( this->_config.fpipeline.min_z_thresh_cm );
			this->_nt.tuning.pmf_window_base.Set( this->_config.fpipeline.pmf_window_base );
			this->_nt.tuning.pmf_max_window_size.Set( this->_config.fpipeline.pmf_max_window_size_cm );
			this->_nt.tuning.pmf_cell_size.Set( this->_config.fpipeline.pmf_cell_size_cm );
			this->_nt.tuning.pmf_init_distance.Set( this->_config.fpipeline.pmf_init_distance_cm );
			this->_nt.tuning.pmf_max_distance.Set( this->_config.fpipeline.pmf_max_distance_cm );
			this->_nt.tuning.pmf_slope.Set( this->_config.fpipeline.pmf_slope );
#endif

		}
		void shutdownNT() {
			this->_nt.instance.Flush();
			this->_nt.instance.StopServer();
			this->_nt.instance.StopClient();
		}

	public:
		/** add a world pose + linked timestamp */
		status_t addWorldRef(const float* xyz, const float* qxyzw, const uint64_t ts_us);
		/** export obstacles from the accumulator grid */
		status_t exportObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t));
		/** wait for the next accumulator update (or pull cached updates) and export obstacles */
		status_t exportNextObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), double timeout_ms);

	protected:
		status_t gridExportInternal(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), std::unique_lock<std::timed_mutex>& lock);


	public:	// global inst, constant values, configs, states
		inline static std::unique_ptr<LidarImpl> _global{ nullptr };

		static constexpr size_t
			MS100_SEGMENTS_PER_FRAME = 12U,
			MS100_POINTS_PER_SEGMENT_ECHO = 900U,	// points per segment * segments per frame = 10800 points per frame (with 1 echo)
			MS100_MAX_ECHOS_PER_POINT = 3U;			// echos get filterd when we apply different settings in the web dashboard

		struct {	// configured constants/parameters
			const std::string lidar_hostname		= LidarConfig::STATIC_DEFAULT.lidar_hostname;	// always fails when we give a specific hostname?
			const int lidar_udp_port				= LidarConfig::STATIC_DEFAULT.lidar_udp_port;
			/* "While MSGPACK can be integrated easily using existing
			 * libraries and is easy to parse, it requires more computing power and bandwidth than
			 * the compact data format due to the descriptive names. Compact is significantly more
			 * efficient and has a lower bandwidth." */
			const bool use_msgpack					= LidarConfig::STATIC_DEFAULT.use_msgpack;

			// NOTE: points within the same segment are not 'rotationally' aligned! (only temporally aligned)
			const uint64_t enabled_segments			= LidarConfig::STATIC_DEFAULT.enabled_segments_bits;	// 12 sections --> first 12 bits enabled (enable all section)
			const uint32_t buffered_frames			= LidarConfig::STATIC_DEFAULT.buffered_scan_frames;		// how many samples of each segment we require per aquisition
			const uint32_t max_filter_threads		= ldru::convertNumThreads(LidarConfig::STATIC_DEFAULT.max_filter_threads, 1);	// minimum of 1 thread, otherwise reserve the main thread and some extra margin for other processes
			const uint64_t points_logging_mode		= LidarConfig::STATIC_DEFAULT.points_logging_mode;
			const char* points_log_fname			= LidarConfig::STATIC_DEFAULT.points_tar_fname;
			const double pose_history_range			= LidarConfig::STATIC_DEFAULT.pose_history_period_s;		// seconds --> the window only gets applied when we add new poses so this just needs account for the greatest difference between new poses getting added and points getting transformed in a thread
			const bool skip_invalid_transform_ts	= LidarConfig::STATIC_DEFAULT.skip_invalid_transform_ts;	// skip points for which we don't have a pose from localization (don't use the default pose of nothing!)

			const uint32_t obstacle_point_color		= LidarConfig::STATIC_DEFAULT.obstacle_point_color;
			const uint32_t standard_point_color		= LidarConfig::STATIC_DEFAULT.standard_point_color;

			const Eigen::Vector3f lidar_offset_xyz{ LidarConfig::STATIC_DEFAULT.lidar_offset_xyz };			// TODO: actual lidar offset!
			const Eigen::Quaternionf lidar_offset_quat{ LidarConfig::STATIC_DEFAULT.lidar_offset_quat };	// identity quat for now

			struct {
				float
					min_scan_theta_deg		= LidarConfig::STATIC_DEFAULT.min_scan_theta_degrees,		// max scan theta angle used for cutoff -- see ms100 operating manual for coordinate system
					max_scan_theta_deg		= LidarConfig::STATIC_DEFAULT.max_scan_theta_degrees,		// min scan theta angle used for cutoff
					min_scan_range_cm		= LidarConfig::STATIC_DEFAULT.min_scan_range_cm,		// the minimum scan range
					max_pmf_range_cm		= LidarConfig::STATIC_DEFAULT.max_pmf_range_cm,		// max range for points used in PMF
					max_z_thresh_cm			= LidarConfig::STATIC_DEFAULT.max_z_thresh_cm,		// for the "mid cut" z-coord filter
					min_z_thresh_cm			= LidarConfig::STATIC_DEFAULT.min_z_thresh_cm,
					// filter by intensity? (test this)

					voxel_size_cm			= LidarConfig::STATIC_DEFAULT.voxel_size_cm,		// voxel cell size used during voxelization filter
					map_resolution_cm		= LidarConfig::STATIC_DEFAULT.map_resolution_cm,		// the resolution of each grid cell
					pmf_window_base			= LidarConfig::STATIC_DEFAULT.pmf_window_base,
					pmf_max_window_size_cm	= LidarConfig::STATIC_DEFAULT.pmf_max_window_size_cm,
					pmf_cell_size_cm		= LidarConfig::STATIC_DEFAULT.pmf_cell_size_cm,
					pmf_init_distance_cm	= LidarConfig::STATIC_DEFAULT.pmf_init_distance_cm,
					pmf_max_distance_cm		= LidarConfig::STATIC_DEFAULT.pmf_max_distance_cm,
					pmf_slope				= LidarConfig::STATIC_DEFAULT.pmf_slope;

			} fpipeline;

		} _config;

		struct {	// states to be communicated across threads
			int32_t log_level = LidarConfig::STATIC_DEFAULT.log_level;

			std::atomic<bool> enable_threads{ false };
			size_t obstacle_updates{ 0U };

			std::mutex
				finished_queue_mutex{};
			std::timed_mutex
				accumulation_mutex{};
			std::shared_mutex
				localization_mutex{};
			std::condition_variable_any
				obstacles_updated{};

		} _state;

	protected:	// nt pointers and filter instance struct
		struct {	// networktables
			nt::NetworkTableInstance instance;	// = nt::NetworkTableInstance::GetDefault()
			std::shared_ptr<nt::NetworkTable> base;

			nt::IntegerEntry
				last_parsed_seg_idx,
				aquisition_cycles;
			nt::DoubleEntry aquisition_ftime;
			nt::RawEntry
				raw_scan_points,
				test_filtered_points;

#ifndef DISABLE_NT_TUNING
			struct {
				nt::FloatEntry
					max_pmf_range,
					max_z_thresh,
					min_z_thresh,
					pmf_window_base,
					pmf_max_window_size,
					pmf_cell_size,
					pmf_init_distance,
					pmf_max_distance,
					pmf_slope
				;
			} tuning;
#endif

		} _nt;

		using SampleBuffer = std::vector< std::deque< sick_scansegment_xd::ScanSegmentParserOutput > >;
		struct FilterInstance {	// storage for each filter instance that needs to be synced between threads
			FilterInstance(const uint32_t f_idx) : index{f_idx} {}
			FilterInstance(const FilterInstance&) = delete;
			FilterInstance(FilterInstance&&) = default;

			const uint32_t index{ 0 };
			SampleBuffer samples{};

			std::unique_ptr<std::thread> thread{ nullptr };
			// std::mutex link_mutex;
			std::condition_variable link_condition{};
			std::atomic<uint32_t> link_state{ 0 };

			struct {
				nt::BooleanEntry is_active;
				nt::IntegerEntry proc_step;
			} nt;
		};

	protected:	// main internal entities
		std::unique_ptr<std::thread>
			lidar_thread{ nullptr };
		sick_scansegment_xd::PayloadFifo*
			udp_fifo{ nullptr };
		std::vector<std::unique_ptr<FilterInstance> >
			filter_threads{};	// need pointers since filter instance doesn't like to be copied (vector issue)
		std::deque<uint32_t>
			finished_queue{};
		frc::TimeInterpolatableBuffer<Eigen::Isometry3f>
			transform_map{
				units::time::second_t{ _config.pose_history_range },
				&lerpClosest<const Eigen::Isometry3f&>
			};
		QuantizedRatioGrid<ObstacleGrid::Quant_T, float>
			accumulator{};
		PCDTarWriter
			pcd_writer{};

		/** The main lidar I/O worker */
		void lidarWorker();
		/** The filter instance worker */
		void filterWorker(FilterInstance* f_inst);


	};	// LidarImpl





/** Static API */

	// const char* pclVer() {	// these are dumb
	// 	return PCL_VERSION_PRETTY;
	// }
	// bool hasWpilib() {
	// 	return USING_WPILIB;
	// }


	status_t apiInit(const LidarConfig& config) {
		if(!LidarImpl::_global) {
			LidarImpl::_global = std::make_unique<LidarImpl>(config);
		}
		return STATUS_ALREADY_SATISFIED;
	}
	status_t apiDestroy() {
		if(LidarImpl::_global) {
			LidarImpl::_global.reset(nullptr);
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_ALREADY_SATISFIED;
	}

	status_t lidarInit() {
		if(LidarImpl::_global) {
			return LidarImpl::_global->startLidar();
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
	}
	status_t lidarShutdown() {
		if(LidarImpl::_global) {
			return LidarImpl::_global->closeLidar();
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_ALREADY_SATISFIED;
	}
	status_t lidarSetState(const bool enabled) {
		return enabled ? lidarInit() : lidarShutdown();
	}

	status_t setLogLevel(const int32_t lvl) {
		if(lvl < 0 || lvl > 3) return STATUS_BAD_PARAM | STATUS_FAIL;
		if(LidarImpl::_global) {
			LidarImpl::_global->_state.log_level = lvl;
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
	}

	status_t updateWorldPose(const float* xyz, const float* qxyzw, const uint64_t ts_us) {
		if(LidarImpl::_global) {
			return LidarImpl::_global->addWorldRef(xyz, qxyzw, ts_us);
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
	}

	status_t getObstacleGrid(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t)) {
		if(LidarImpl::_global) {
			return LidarImpl::_global->exportObstacles(grid, grid_resize);
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
	}
	status_t waitNextObstacleGrid(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), double timeout_ms) {
		if(LidarImpl::_global) {
			return LidarImpl::_global->exportNextObstacles(grid, grid_resize, timeout_ms);
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
	}





/** LidarImpl interfacing implementation */

status_t LidarImpl::addWorldRef(const float* xyz, const float* qxyzw, const uint64_t ts_us) {

	// check if lidar is running?
	const units::time::second_t timestamp{
		ts_us == 0 ?
			crno::duration<double>{ crno::hrc::now().time_since_epoch() }.count() :
			(double)ts_us / 1e6
	};

	/* >> FOR FUTURE REFERENCE!!! <<
		* "Eigen::QuaternionXX * Eigen::TranslationXX" IS NOT THE SAME AS "Eigen::TranslationXX * Eigen::QuaternionXX"
		* The first ROTATES the translation by the quaternion, whereas the second one DOES NOT!!! */
	const Eigen::Quaternionf
		r2w_quat = Eigen::Quaternionf{ qxyzw },						// robot's rotation in the world
		l2w_quat = (this->_config.lidar_offset_quat * r2w_quat);	// lidar's rotation in the world
	const Eigen::Isometry3f
		r2w_transform = (*reinterpret_cast<const Eigen::Translation3f*>(xyz)) * r2w_quat;	// compose robot's position and rotation
	const Eigen::Vector3f
		l_w_pos = r2w_transform * this->_config.lidar_offset_xyz;	// transform lidar's offset in robot space -- the same as rotating the lidar position to global coords and adding the robot's position
	const Eigen::Isometry3f
		l2w_transform = (*reinterpret_cast<const Eigen::Translation3f*>(&l_w_pos)) * l2w_quat;	// compose the lidar's global position and the lidar's global rotation

	this->_state.localization_mutex.lock();
	this->transform_map.AddSample( timestamp, l2w_transform );
	this->_state.localization_mutex.unlock();
	return STATUS_SUCCESS;

}

status_t LidarImpl::exportObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t)) {

	if(this->_state.enable_threads.load()) {
		std::unique_lock<std::timed_mutex> lock{ this->_state.accumulation_mutex };
		return this->gridExportInternal(grid, grid_resize, lock);
	}
	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;

}
status_t LidarImpl::exportNextObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), double timeout_ms) {

	if(this->_state.enable_threads.load()) {
		crno::hrc::time_point _until = crno::hrc::now() + crno::nanoseconds{ static_cast<int64_t>(timeout_ms * 1e6) };
		std::unique_lock<std::timed_mutex> lock{ this->_state.accumulation_mutex, std::defer_lock };
		if (!lock.try_lock_until(_until)) {		// cv::wait_until requires the lock to start out locked, thus we cannot "try_lock()" above -- this code accomplishes the same goal of not blocking past the specified time
			return STATUS_TIMED_OUT | STATUS_FAIL;
		}
		for(;this->_state.enable_threads.load() && this->_state.obstacle_updates < 1;) {
			//try {
				if(this->_state.obstacles_updated.wait_until(lock, _until) == std::cv_status::timeout) {
					return STATUS_TIMED_OUT | STATUS_FAIL;
				}
			/*} catch(const std::exception& e) {
				LDRP_LOG( LOG_DEBUG, "exportNextObstacles() ~ wait_until() threw an exception! -- {}", e.what() )
				return STATUS_TIMED_OUT | STATUS_FAIL;
			}*/
		}
		return this->gridExportInternal(grid, grid_resize, lock);
	}
	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;

}

status_t LidarImpl::gridExportInternal(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), std::unique_lock<std::timed_mutex>& lock) {

	if(lock.mutex() != &this->_state.accumulation_mutex) {
		std::unique_lock<std::timed_mutex> _temp{ this->_state.accumulation_mutex };
		lock.swap(_temp);
	}
	if(!lock.owns_lock()) lock.lock();

	const size_t _area = static_cast<size_t>(this->accumulator.area());
	const Eigen::Vector2f& _origin = this->accumulator.origin();
	const Eigen::Vector2i& _grid_size = this->accumulator.size();

	grid.cell_resolution_m = this->accumulator.cellRes();
	grid.origin_x_m = _origin.x();
	grid.origin_y_m = _origin.y();
	grid.cells_x = _grid_size.x();
	grid.cells_y = _grid_size.y();
	grid.grid = grid_resize(_area);

	memcpy(grid.grid, this->accumulator.buffData(), _area * sizeof(ObstacleGrid::Quant_T));

	// grid.cell_resolution_m = 0.f;
	// grid.origin_x_m = 0.f;
	// grid.origin_y_m = 0.f;
	// grid.cells_x = 0;
	// grid.cells_y = 0;
	// grid.grid = nullptr;

	this->_state.obstacle_updates = 0;
	return STATUS_SUCCESS;
	// return STATUS_FAIL;

}





/** LidarImpl threads implementation */

void LidarImpl::lidarWorker() {

	wpi::DataLogManager::SignalNewDSDataOccur();

	// (the next 30 lines or so are converted from scansegment_threads.cpp: sick_scansegment_xd::MsgPackThreads::runThreadCb())
	// init udp receiver
	sick_scansegment_xd::UdpReceiver* udp_receiver = nullptr;
	for(;!udp_receiver && this->_state.enable_threads.load();) {
		udp_receiver = new sick_scansegment_xd::UdpReceiver{};
		if(udp_receiver->Init(
			this->_config.lidar_hostname,	// udp receiver
			this->_config.lidar_udp_port,	// udp port
			this->_config.buffered_frames * MS100_SEGMENTS_PER_FRAME,	// udp fifo length -- really we should only need 1 or 2?
			LOG_VERBOSE,						// verbose logging when our log level is verbose
			false,								// should export to file?
			this->_config.use_msgpack ? SCANDATA_MSGPACK : SCANDATA_COMPACT,	// scandata format (1 for msgpack)
			nullptr	// fifo for payload data (for sharing a fifo)
		)) {
			LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver successfully connected to host {} on port {}!", this->_config.lidar_hostname, this->_config.lidar_udp_port )
		} else {
			LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver failed to connect to host {} on port {}. Retrying after 3 seconds...", this->_config.lidar_hostname, this->_config.lidar_udp_port )
			delete udp_receiver;
			udp_receiver = nullptr;
			std::this_thread::sleep_for(crno::seconds{3});	// keep trying until successful
		}
	}
	// launch udp receiver -- if successful continue to main loop
	if(udp_receiver->Start()) {
		LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver thread successfully launched!" )

		// defaults for crap we don't use
		static sick_scan_xd::SickCloudTransform no_transform{};
		static sick_scansegment_xd::MsgPackValidatorData default_validator_data_collector{};
		static sick_scansegment_xd::MsgPackValidator default_validator{};
		static sick_scansegment_xd::ScanSegmentParserConfig default_parser_config{};

		// a queue for each segment we are sampling from so we can have a rolling set of aquired samples (when configured)
		SampleBuffer frame_segments{ 0 };

		this->udp_fifo = udp_receiver->Fifo();		// store to global so that we aren't ever stuck waiting for scan data if exit is called
		std::vector<uint8_t> udp_payload_bytes{};
		sick_scansegment_xd::ScanSegmentParserOutput parsed_segment{};
		fifo_timestamp scan_timestamp{};
		size_t scan_count{0};	// not used but we need for a param

		if(this->_config.points_logging_mode & POINT_LOGGING_TAR) {
			this->pcd_writer.setFile(this->_config.points_log_fname);
		}

		this->accumulator.reset(this->_config.fpipeline.map_resolution_cm * 1e-2f);

		// main loop!
		LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: Succesfully initialized all resources. Begining aquisition and filtering..." )
		for(;this->_state.enable_threads.load();) {

			// wpi::DataLogManager::GetLog().Flush();

			frame_segments.resize( ::countBits(this->_config.enabled_segments) );
			const crno::hrc::time_point aquisition_start = crno::hrc::now();
			size_t aquisition_loop_count = 0;
			for(uint64_t filled_segments = 0; this->_state.enable_threads; aquisition_loop_count++) {	// loop until thread exit is called... (internal break allows for exit as well)

				if(filled_segments >= this->_config.enabled_segments) {	// if we have aquired sufficient samples...
					LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Aquisition quota satisfied after {} loops - exporting buffer to thread...", aquisition_loop_count )

					// attempt to find or create a thread for processing the frame
					this->_state.finished_queue_mutex.lock();	// aquire mutex for queue
					if(this->finished_queue.size() > 0) {

						const uint32_t filter_idx = this->finished_queue.front();	// maybe check that this is valid?
						this->finished_queue.pop_front();
						this->_state.finished_queue_mutex.unlock();
						FilterInstance& f_inst = *this->filter_threads[filter_idx];
						std::swap(f_inst.samples, frame_segments);		// figure out where we want to clear the buffer that is swapped in so we don't start with old data in the queues
						f_inst.link_state.store(true);
						f_inst.link_condition.notify_all();

						LDRP_LOG( LOG_DEBUG, "LDRP Worker: Exported complete scan to processing instance {} after aquiring {} segments", filter_idx, aquisition_loop_count )
						break;

					} else {
						this->_state.finished_queue_mutex.unlock();
						if(this->filter_threads.size() < this->_config.max_filter_threads) {	// start a new thread

							// create a new thread and swap in the sample
							this->filter_threads.emplace_back( std::make_unique<FilterInstance>( static_cast<uint32_t>(this->filter_threads.size()) ) );
							FilterInstance& f_inst = *this->filter_threads.back();
							std::swap(f_inst.samples, frame_segments);
							f_inst.thread.reset( new std::thread{&LidarImpl::filterWorker, this, &f_inst} );

							LDRP_LOG( LOG_DEBUG, "LDRP Worker: Created new processing instance {} after aquiring {} segments", f_inst.index, aquisition_loop_count )
							break;

						}
					}
				}	// insufficient samples or no thread available... (keep updating the current framebuff)
#define POINT_SOURCE_MODE 2
#if POINT_SOURCE_MODE == 0	// live sensor operation
				if(this->udp_fifo->Pop(udp_payload_bytes, scan_timestamp, scan_count)) {	// blocks until packet is received

					if(this->_config.use_msgpack ?	// parse based on format
						sick_scansegment_xd::MsgPackParser::Parse(
							udp_payload_bytes, scan_timestamp, no_transform, parsed_segment,
							default_validator_data_collector, default_validator,
							false, false, true, LOG_VERBOSE)
						:
						sick_scansegment_xd::CompactDataParser::Parse(default_parser_config,
							udp_payload_bytes, scan_timestamp, no_transform, parsed_segment,
							true, LOG_VERBOSE)
					) {	// if successful parse
#elif POINT_SOURCE_MODE == 1	// internal simulation
				{
					static constexpr float const
						phi_angles_deg[] = { -22.2, -17.2, -12.3, -7.3, -2.5, 2.2, 7.0, 12.9, 17.2, 21.8, 26.6, 31.5, 36.7, 42.2 },
						dense_phi_angles_deg[] = { 0, 34.2 },
						deg_to_rad = std::numbers::pi_v<float> / 180.f,
						GENERATED_POINTS_MAX_RANGE_METERS = 10.f;

					const float base_theta = aquisition_loop_count * 30;

					crno::hrc::time_point s = crno::hrc::now();
					parsed_segment.scandata.resize(16);
					for(size_t h = 0; h < 14; h++) {
						auto& group = parsed_segment.scandata[h];
						group.scanlines.resize(1);
						auto& line = group.scanlines[0];
						line.points.resize(30);
						const float
							phi = phi_angles_deg[h] * deg_to_rad,
							sin_phi = sin(phi),
							cos_phi = cos(phi);
						for(size_t v = 0; v < 30; v++) {
							const float
								theta = (base_theta + v - 180.f) * deg_to_rad,
								sin_theta = sin(theta),
								cos_theta = cos(theta),

								range = (float)std::rand() / RAND_MAX * GENERATED_POINTS_MAX_RANGE_METERS;

							line.points[v] = sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint{
								range * cos_phi * cos_theta,
								range * cos_phi * sin_theta,
								range * sin_phi,
								1.f, range, theta, phi, (int)h, 0U, (int)v, 0U
							};
						}
					}
					for(size_t h = 14; h < 16; h++) {
						auto& group = parsed_segment.scandata[h];
						group.scanlines.resize(1);
						auto& line = group.scanlines[0];
						line.points.resize(240);
						const float
							phi = dense_phi_angles_deg[h - 14] * deg_to_rad,
							sin_phi = sin(phi),
							cos_phi = cos(phi);
						for(size_t v = 0; v < 240; v++) {
							const float
								theta = (base_theta + (v / 8.f) - 180.f) * deg_to_rad,
								sin_theta = sin(theta),
								cos_theta = cos(theta),

								range = (float)std::rand() / RAND_MAX * GENERATED_POINTS_MAX_RANGE_METERS;

							line.points[v] = sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint{
								range * cos_phi * cos_theta,
								range * cos_phi * sin_theta,
								range * sin_phi,
								1.f, range, theta, phi, (int)h, 0U, (int)v, 0U
							};
						}
					}
					parsed_segment.segmentIndex = aquisition_loop_count;
					std::this_thread::sleep_until(s + 4900us);
					if constexpr(true) {
#elif POINT_SOURCE_MODE == 2	// pull data from nt (uesim)
				{
					crno::hrc::time_point s = crno::hrc::now();
					
					static nt::RawEntry _entry = this->_nt.instance.GetRawTopic("uesim/points").GetEntry("PointXYZ_[]", {});
					std::vector<nt::TimestampedRaw> points_queue = _entry.ReadQueue();
					bool valid = false;
					if(points_queue.size() > 0) {
						nt::TimestampedRaw& ts_raw = points_queue[0];
						parsed_segment.scandata.resize(1);
						parsed_segment.scandata[0].scanlines.resize(1);
						auto& pts_out = parsed_segment.scandata[0].scanlines[0].points;
						
						const size_t len = ts_raw.value.size() / 16U;
						pts_out.resize(len);
						const pcl::PointXYZ* _points = reinterpret_cast<const pcl::PointXYZ*>(ts_raw.value.data());
						for(size_t i = 0; i < len; i++) {
							sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& lpt = pts_out[i];
							const pcl::PointXYZ& pt = _points[i];
							lpt.x = pt.x;
							lpt.y = pt.y;
							lpt.z = pt.z;
							lpt.range = 100.f;
							lpt.azimuth = 0.f;
							lpt.elevation = 0.f;
						}
						parsed_segment.segmentIndex = 0;
						parsed_segment.timestamp_sec = (ts_raw.time % 1000000U);
						parsed_segment.timestamp_nsec = (ts_raw.time - (ts_raw.time % 1000000U)) * 1000U;
						filled_segments = this->_config.enabled_segments;	// export immediately
						valid = true;
					}

					std::this_thread::sleep_until(s + 4900us);
					if(valid) {
#else
				{
					std::this_thread::sleep_for(5ms);
					if constexpr(false) {
#endif
						LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Successfully parsed scan segment idx {}!", parsed_segment.segmentIndex )

						const uint64_t seg_bit = (1ULL << parsed_segment.segmentIndex);
						if(this->_config.enabled_segments & seg_bit) {	// if the segment'th bit is set
							size_t idx = ::countBitsBeforeN(this->_config.enabled_segments, parsed_segment.segmentIndex);	// get the index of the enabled bit (for our buffer)

							frame_segments[idx].emplace_front();	// create empty segment buffer
							ldru::swapSegmentsNoIMU(frame_segments[idx].front(), parsed_segment);		// efficient buffer transfer (no deep copying!)
							if(frame_segments[idx].size() >= this->_config.buffered_frames) {
								frame_segments[idx].resize(this->_config.buffered_frames);	// cut off oldest scans beyond the buffer size (resize() removes from the back)
								filled_segments |= seg_bit;		// save this segment as finished by enabling it's bit
							}
						}
						this->_nt.last_parsed_seg_idx.Set( log2(seg_bit) );

					} else {
						LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Failed to parse bytes from UdpReceiver." )
						this->_nt.last_parsed_seg_idx.Set( -1 );
					}

				}
			}

			this->_nt.aquisition_ftime.Set( crno::duration<double>{crno::hrc::now() - aquisition_start}.count() );
			this->_nt.aquisition_cycles.Set( aquisition_loop_count );

			wpi::DataLogManager::SignalNewDSDataOccur();

		}

		LDRP_LOG( LOG_DEBUG, "LDRP Worker [Exit]: Aquisition loop ended. Collecting resources..." )

	} else {	// udp receiver launch thread
		LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver thread failed to start. Exitting..." )
	}

	this->pcd_writer.closeIO();		// doesn't do anything if we never initialized

	// join and delete all filter instances
	for(std::unique_ptr<FilterInstance>& f_inst : this->filter_threads) {
		const uint32_t idx = f_inst->index;
		if(f_inst->thread && f_inst->thread->joinable()) {
			LDRP_LOG( LOG_DEBUG, "LDRP Worker [Exit]: Waiting for filter instance {} to join...", idx )
			f_inst->link_condition.notify_all();
			f_inst->thread->join();
		}
		delete f_inst->thread.release();
		LDRP_LOG( LOG_DEBUG, "LDRP Worker [Exit]: Closed filter instance {}.", idx )
	}

	// close and deallocate udp receiver
	this->udp_fifo->Shutdown();
	udp_receiver->Close();
	delete udp_receiver;

}	// LidarImpl::lidarWorker()

void LidarImpl::filterWorker(LidarImpl::FilterInstance* f_inst) {
	// this function represents the alternative thread that filters the newest collection of points

	std::shared_ptr<nt::NetworkTable> nt_inst = this->_nt.base->GetSubTable( fmt::format("Filter Threads/inst {}", f_inst->index) );
	f_inst->nt.is_active = nt_inst->GetBooleanTopic("activity").GetEntry(false);
	f_inst->nt.proc_step = nt_inst->GetIntegerTopic("state").GetEntry(0);

	f_inst->nt.is_active.Set(false);
	f_inst->nt.proc_step.Set(0);	// 0 = init (pre looping)
	
	// precalulcate values
	const size_t max_points{ 
		(MS100_POINTS_PER_SEGMENT_ECHO * MS100_MAX_ECHOS_PER_POINT)
			* ::countBits(this->_config.enabled_segments)
			* this->_config.buffered_frames
	};
	static const Eigen::Isometry3f DEFAULT_NO_POSE = Eigen::Isometry3f::Identity();
	static const pcl::Indices DEFAULT_NO_SELECTION = pcl::Indices{};

	// buffers are reused between loops
	pcl::PointCloud<pcl::PointXYZ>
		point_cloud,
		voxelized_points;
	std::vector<float>
		// point_ranges,	// currently don't need since we voxelize and have to regenerate anyway
		voxelized_ranges;
	pcl::Indices
		z_high_filtered{},
		z_low_subset_filtered{},
		z_mid_filtered_obstacles{},
		pre_pmf_range_filtered{},
		pmf_filtered_ground{},
		pmf_filtered_obstacles{},
		combined_obstacles{};

	point_cloud.points.reserve( max_points );
	// point_ranges.reserve( max_points );

	LDRP_LOG( LOG_STANDARD, "LDRP Filter Instance {} [Init]: Resources initialized - running filter loop...", f_inst->index )

	// thread loop
	for(;this->_state.enable_threads.load();) {

		f_inst->nt.is_active.Set(true);
		f_inst->nt.proc_step.Set(10);	// 1 = step 1 (combine points)

		point_cloud.clear();	// clear the vector and set w,h to 0
		// point_ranges.clear();	// << MEMORY LEAK!!! (it was)

		Eigen::Vector3f avg_origin{ 0.f, 0.f, 0.f };
		size_t origin_samples = 0;

		// 1. transform points based on timestamp
		for(size_t i = 0; i < f_inst->samples.size(); i++) {			// we could theoretically multithread this part -- just use a mutex for inserting points into the master collection
			for(size_t j = 0; j < f_inst->samples[i].size(); j++) {
				const sick_scansegment_xd::ScanSegmentParserOutput& segment = f_inst->samples[i][j];

				this->_state.localization_mutex.lock_shared();	// other threads can read from the buffer as well
				std::optional<Eigen::Isometry3f> ts_transform = this->transform_map.Sample(
					units::time::second_t{ segment.timestamp_sec + (segment.timestamp_nsec * 1E-9) } );
				this->_state.localization_mutex.unlock_shared();

				if(this->_config.skip_invalid_transform_ts && !ts_transform.has_value()) continue;
				const Eigen::Isometry3f& transform = ts_transform.has_value() ? *ts_transform : DEFAULT_NO_POSE;

				avg_origin += transform.translation();
				origin_samples++;

				for(const sick_scansegment_xd::ScanSegmentParserOutput::Scangroup& scan_group : segment.scandata) {		// "ms100 transmits 16 groups"
#define USE_FIRST_ECHO_ONLY		// make a config option or grouping for this instead
#ifndef USE_FIRST_ECHO_ONLY
					for(const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line: scan_group.scanlines) {	// "each group has up to 3 echos"
#else
					if(scan_group.scanlines.size() > 0) {
						const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line = scan_group.scanlines[0];
#endif
						for(const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& lidar_point : scan_line.points) {

// #define DISABLE_PRELIM_POINT_FILTERING	// option for this as well
#ifndef DISABLE_PRELIM_POINT_FILTERING
							const float azimuth_deg = lidar_point.azimuth * 180.f / std::numbers::pi_v<float>;
							if(
								(azimuth_deg <= this->_config.fpipeline.max_scan_theta_deg && azimuth_deg >= this->_config.fpipeline.min_scan_theta_deg)
									&& (lidar_point.range * 1e2f > this->_config.fpipeline.min_scan_range_cm)
								// ...apply any other filters that benefit from points in lidar scan coord space as well
							) {
#else
							{
#endif
								point_cloud.points.emplace_back();
								reinterpret_cast<Eigen::Vector4f&>(point_cloud.points.back()) = transform * Eigen::Vector4f{lidar_point.x, lidar_point.y, lidar_point.z, 1.f};	// EEEEEE :O
								// point_ranges.push_back(lidar_point.range);
							}
						}
					}
				} // loop points per segment

			}

			f_inst->samples[i].clear();		// clear the queue so that when the buffer gets swapped back to main, the queues are fresh

		} // loop segments

		LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Filter Instance {} [Filter Loop]: Collected {} points", f_inst->index, point_cloud.size() )

		point_cloud.width = point_cloud.points.size();
		point_cloud.height = 1;
		point_cloud.is_dense = true;

		if(this->_config.points_logging_mode & POINT_LOGGING_INCLUDE_RAW) {
			f_inst->nt.proc_step.Set(11);	// 11 = export raw cloud
			if(this->_config.points_logging_mode & POINT_LOGGING_TAR) {
				this->pcd_writer.addCloud(point_cloud);
			}
			if(this->_config.points_logging_mode & POINT_LOGGING_NT) {
				this->_nt.raw_scan_points.Set(
					std::span<const uint8_t>{
						reinterpret_cast<uint8_t*>( point_cloud.points.data() ),
						reinterpret_cast<uint8_t*>( point_cloud.points.data() + point_cloud.points.size() )
					}
				);
			}
		}

		// 2. run filtering on points
		{

			f_inst->nt.proc_step.Set(20);	// 20 = filtering (init)

#ifndef DISABLE_NT_TUNING
			const float
				_max_pmf_range			= this->_nt.tuning.max_pmf_range.Get() * 1e-2f,
				_max_z_thresh			= this->_nt.tuning.max_z_thresh.Get() * 1e-2f,
				_min_z_thresh			= this->_nt.tuning.min_z_thresh.Get() * 1e-2f,
				_pmf_window_base		= this->_nt.tuning.pmf_window_base.Get(),
				_pmf_max_window_size	= this->_nt.tuning.pmf_max_window_size.Get() * 1e-2f,
				_pmf_cell_size			= this->_nt.tuning.pmf_cell_size.Get() * 1e-2f,
				_pmf_init_distance		= this->_nt.tuning.pmf_init_distance.Get() * 1e-2f,
				_pmf_max_distance		= this->_nt.tuning.pmf_max_distance.Get() * 1e-2f,
				_pmf_slope				= this->_nt.tuning.pmf_slope.Get()
			;
#endif

			voxelized_points.clear();
			voxelized_ranges.clear();

			z_high_filtered.clear();
			z_low_subset_filtered.clear();
			z_mid_filtered_obstacles.clear();
			pre_pmf_range_filtered.clear();
			pmf_filtered_ground.clear();
			pmf_filtered_obstacles.clear();
			combined_obstacles.clear();

			if(origin_samples > 1) avg_origin /= origin_samples;

			// voxelize points
			f_inst->nt.proc_step.Set(21);	// 21 = filtering (voxelize)
			voxel_filter(
				point_cloud, DEFAULT_NO_SELECTION, voxelized_points,
				this->_config.fpipeline.voxel_size_cm * 1e-2f,
				this->_config.fpipeline.voxel_size_cm * 1e-2f,
				this->_config.fpipeline.voxel_size_cm * 1e-2f
			);

			// filter points under "high cut" thresh
			f_inst->nt.proc_step.Set(22);	// 22 = filtering (z-high)
			carteZ_filter(
				voxelized_points, DEFAULT_NO_SELECTION, z_high_filtered,
				-std::numeric_limits<float>::infinity(),
				_max_z_thresh
			);
			// further filter points below "low cut" thresh
			f_inst->nt.proc_step.Set(23);	// 23 = filtering (z-low)
			carteZ_filter(
				voxelized_points, z_high_filtered, z_low_subset_filtered,
				-std::numeric_limits<float>::infinity(),
				_min_z_thresh
			);
			// get the points inbetween high and low thresholds --> treated as wall obstacles
			f_inst->nt.proc_step.Set(24);	// 24 = filtering (z-mid)
			pc_negate_selection(
				z_high_filtered,
				z_low_subset_filtered,
				z_mid_filtered_obstacles
			);

			// filter close enough points for PMF
			f_inst->nt.proc_step.Set(25);	// 25 = filtering (pmf-pre)
			pc_filter_distance(
				voxelized_points.points,
				z_low_subset_filtered,
				pre_pmf_range_filtered,
				0.f, _max_pmf_range,
				avg_origin
			);

			// apply pmf to selected points
			f_inst->nt.proc_step.Set(26);	// 26 = filtering (pmf)
			progressive_morph_filter(
				voxelized_points, pre_pmf_range_filtered, pmf_filtered_ground,
				_pmf_window_base,
				_pmf_max_window_size,
				_pmf_cell_size,
				_pmf_init_distance,
				_pmf_max_distance,
				_pmf_slope,
				false
			);
			// obstacles = (base - ground)
			f_inst->nt.proc_step.Set(27);	// 27 = filtering (obstacles)
			pc_negate_selection(
				pre_pmf_range_filtered,
				pmf_filtered_ground,
				pmf_filtered_obstacles
			);

			// export filter results
			if(this->_config.points_logging_mode & (POINT_LOGGING_NT | POINT_LOGGING_INCLUDE_FILTERED)) {

				f_inst->nt.proc_step.Set(28);	// 28 = filtering (export)

				// combine all obstacle points into a single selection
				pc_combine_sorted(
					z_mid_filtered_obstacles,
					pmf_filtered_obstacles,
					combined_obstacles
				);

				write_interlaced_selection_bytes<4, 3>(
					std::span<uint32_t>{
						reinterpret_cast<uint32_t*>( voxelized_points.points.data() ),
						reinterpret_cast<uint32_t*>( voxelized_points.points.data() + voxelized_points.points.size() ),
					},
					combined_obstacles,
					this->_config.obstacle_point_color,
					this->_config.standard_point_color
				);

				this->_nt.test_filtered_points.Set(
					std::span<const uint8_t>{
						reinterpret_cast<uint8_t*>( voxelized_points.points.data() ),
						reinterpret_cast<uint8_t*>( voxelized_points.points.data() + voxelized_points.points.size() )
					}
				);

			}

		}

		// 3. update accumulator
		{
			f_inst->nt.proc_step.Set(30);	// 30 = update grid (locking)

			this->_state.accumulation_mutex.lock();

			f_inst->nt.proc_step.Set(31);	// 31 = update grid (pmf insert)
			this->accumulator.incrementRatio(	// insert PMF obstacles
				voxelized_points,
				pre_pmf_range_filtered,		// base
				pmf_filtered_obstacles		// subset
			);
			f_inst->nt.proc_step.Set(32);	// 32 = update grid (z-insert)
			this->accumulator.incrementRatio(	// insert z-thresh obstacles
				voxelized_points,
				z_mid_filtered_obstacles,	// base
				DEFAULT_NO_SELECTION		// use all of base
			);

			f_inst->nt.proc_step.Set(33);	// 33 = update grid (cleanup)
			this->_state.obstacle_updates++;
			this->_state.obstacles_updated.notify_all();

			LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Filter Instance {} [Filter Loop]: Successfully added points to accumulator. Map size: {}x{}, origin: ({}, {})",
				f_inst->index,
				this->accumulator.size().x(), this->accumulator.size().y(),
				this->accumulator.origin().x(), this->accumulator.origin().y()
			)

			this->_state.accumulation_mutex.unlock();
		}
		// done!!!

		// processing finished, push instance idx to queue
		f_inst->nt.is_active.Set(false);
		f_inst->nt.proc_step.Set(40);	// 40 = finished
		f_inst->link_state = false;
		std::unique_lock<std::mutex> lock{ this->_state.finished_queue_mutex };
		this->finished_queue.push_back(f_inst->index);
		// wait for signal to continue...
		f_inst->nt.proc_step.Set(41);	// 41 = waiting for update
		for(;this->_state.enable_threads.load() && !f_inst->link_state;) {
			f_inst->link_condition.wait(lock);
		}

	}	// thread loop

}	// LidarImpl::filterWorker()


};	// namespace ldrp
