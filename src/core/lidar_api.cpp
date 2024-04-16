/** Copyright (c) Cardinal Space Mining 2024 */

/** Default macro definitions -- configurable via CMake */
#ifndef LDRP_ENABLE_LOGGING		// enable/disable all logging output
  #define LDRP_ENABLE_LOGGING					true
#endif
#ifndef LDRP_DEBUG_LOGGING		// enable/disable debug level logging
  #define LDRP_DEBUG_LOGGING					false
#endif
#ifndef LDRP_SAFETY_CHECKS		// enable/disable additional safety checking (ex. bound checking)
  #define LDRP_SAFETY_CHECKS					true
#endif
#ifndef LDRP_USE_WPILIB			// whether or not WPILib is being compiled into the library - for build system internal use only
  #define LDRP_USE_WPILIB						false
#endif
#ifndef LDRP_USE_SIM_MODE		// enable/disable using simulation as the source of points, and set which simulation source to use (1 = internal, 2 = UE simulator)
  #define LDRP_USE_SIM_MODE						false
#endif
#ifndef LDRP_USE_ALL_ECHOS			// whether or not to use all echo points from the live scanner
  #define LDRP_USE_ALL_ECHOS					false
#endif
#ifndef LDRP_ENABLE_PRELIM_POINT_FILTERING		// enable/disable preliminary filtering of points (azimuth angle and minimum range)
  #define LDRP_ENABLE_PRELIM_POINT_FILTERING	true
#endif
#ifndef LDRP_ENABLE_NT_TUNING	// enable/disable live tuning using networktables (requires WPILib)
  #define LDRP_ENABLE_NT_TUNING					false
#endif
#ifndef LDRP_ENABLE_NT_PROFILING	// enable/disable live and logged filter pipeline profiling over networktables
  #define LDRP_ENABLE_NT_PROFILING				false
#endif


#include "lidar_api.h"
#if !LDRP_SAFETY_CHECKS
  #define GRID_IMPL_SKIP_BOUND_CHECKING		// disables array bound checking within QRG
#endif
#include "filtering.hpp"
#include "grid.hpp"
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
#include <tuple>
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

#if LDRP_USE_WPILIB		// TODO: actually enforce this macro in code when disabled?
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
#if LDRP_USE_WPILIB
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
	std::swap(a.timestamp_sec, b.timestamp_sec);
	std::swap(a.timestamp_nsec, b.timestamp_nsec);
	std::swap(a.segmentIndex, b.segmentIndex);
	std::swap(a.telegramCnt, b.telegramCnt);
}

static inline const uint32_t convertNumThreads(const int32_t input_num, const int32_t reserved = 0) {
	const int32_t _max = (int32_t)std::thread::hardware_concurrency() - reserved;
	return input_num < 1 ?
		(uint32_t)std::max(_max + input_num, 1) :
		(uint32_t)std::min(_max, input_num)
	;
}

template<typename FT = float>
static inline int64_t floatSecondsToIntMicros(const FT v) {
	static_assert(std::is_floating_point_v<FT>, "");
	return static_cast<int64_t>(v * static_cast<FT>(1e6));
}

template<typename T = uint32_t>
static inline int64_t constructTimestampMicros(const T seconds, const T nanoseconds) {
	return (static_cast<int64_t>(seconds) * 1000000L) + (static_cast<int64_t>(nanoseconds) / 1000L);
}

static inline int64_t rectifyTimestampMicros(const uint64_t ts_us) {
	return (ts_us == 0 ?
		static_cast<int64_t>(crno::duration_cast<crno::microseconds>(crno::hrc::now().time_since_epoch()).count()) :
		static_cast<int64_t>(ts_us)
	);
}


// template<typename Time_t>
// struct timebase_traits {
// 	inline static constexpr bool
// 		is_float = std::is_floating_point<Time_t>::value,
// 		is_integral = std::is_integral<Time_t>::value,
// 		is_chrono = crno::is_clock<Time_t>::value;
// };

/** TimestampSampler stores an ordered list of time-element pairs, and can efficiently look up the closes element given a target timestamp. */
template<typename T, typename Time_t>
class TimestampSampler {
public:
	using This_T = TimestampSampler<T, Time_t>;
	using Type = T;
	using TimeT = Time_t;
	using ElemT = std::pair<TimeT, Type>;

private:
	static bool t_less__(TimeT t, const ElemT& elem) {
		return t < elem.first;
	}
	static bool t_greater__(const ElemT& elem, TimeT t) {
		return t > elem.first;
	}

public:
	TimestampSampler() = default;
	~TimestampSampler() = default;


	/** Update the minimum bound timestamp such as to limit the number of entities stored in the buffer */
	void updateMin(TimeT min) {
		this->absolute_bound = min;
		this->enforceBound();
	}

	/** Add a new sample to the buffer */
	inline void insert(TimeT time, const Type& sample)
		{ this->insert(time, sample); }
	/** Add a new sample to the buffer */
	void insert(TimeT time, Type&& sample) {
		if(this->samples.size() <= 0 || time > this->samples.back().first) {
			this->samples.emplace_back(time, std::forward<Type>(sample));
		} else {
			auto after = std::upper_bound(this->samples.begin(), this->samples.end(), time, &This_T::t_less__);
			auto before = after - 1;
			if(after == this->samples.begin()) {
				this->samples.insert(after, std::pair{time, std::forward<Type>(sample)});
			} else {
				if(before == this->samples.begin() || before->first < time) {
					this->samples.insert(after, std::pair{time, std::forward<Type>(sample)});
				} else {
					before->second = sample;
				}
			}
		}
	}

	/** Erase all the elements */
	inline void clear() { this->samples.clear(); }

	/** Sample the element with a timestamp closest to that given as a parameter */
	inline const Type* sample(TimeT time) const {
		const ElemT* elem = this->sampleTimestamped(time);
		return elem ? elem->second : nullptr;
	}
	/** Sample the closest element, but returns the full pair so that the element's timestamp can be accessed */
	const ElemT* sampleTimestamped(TimeT time) const {
		if( this->samples.empty() )					return nullptr;
		if( time <= this->samples.front().first )	return &this->samples.front();
		if( time >= this->samples.back().first )	return &this->samples.back();
		if( this->samples.size() == 1 )				return &this->samples[0];

		auto greater = std::lower_bound(this->samples.begin(), this->samples.end(), time, &This_T::t_greater__);	// "lower bound" of times greater than t
		if( greater == this->samples.begin() )		return &*greater;

		auto less = greater - 1;
		return abs(time - less->first) < abs(greater->first - time) ? &*less : &*greater;		// return closer sample
	}

	/** Return a const reference to the internal element buffer */
	inline const std::vector<ElemT>& getSamples() const { return this->samples; }


protected:
	void enforceBound() {
		auto last = this->samples.begin();
		for(; last < this->samples.end() && last->first < this->absolute_bound; last++);
		this->samples.erase(this->samples.begin(), last);
	}

protected:
	TimeT absolute_bound = static_cast<TimeT>(0);	// the absolute minimum bounding time
	std::vector<ElemT> samples{};	// use a tree for better search performance


};

};





/** Main interface namespace */

namespace ldrp {

#if LDRP_ENABLE_LOGGING
  #if LDRP_USE_WPILIB
    #define LDRP_LOG(__condition, ...)		if(__condition) { wpi::DataLogManager::Log( fmt::format(__VA_ARGS__) ); }
  #else
    #define LDRP_LOG(__condition, ...)		if(__condition) { std::printf("%s\n", fmt::format(__VA_ARGS__) ); }		// DLM automatically appends newlines automatically
  #endif
  #define LOG_LEVEL_GLOBAL(__inst, __min_lvl)	(__inst->_state.log_level >= __min_lvl)
  #define LOG_LEVEL(__min_lvl)					LOG_LEVEL_GLOBAL(this, __min_lvl)
  #define LOG_ALWAYS			true
  #define LOG_STANDARD			LOG_LEVEL(1)
  #define LOG_VERBOSE			LOG_LEVEL(2)
  #define LOG_DEBUG				LDRP_DEBUG_LOGGING
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
  #define LOG_DEBUG
#endif


const LidarConfig LidarConfig::STATIC_DEFAULT{};


template<typename F>
using Vec3_ = Eigen::Vector3<F>;
template<typename F>
using Trl3_ = Eigen::Translation<F, 3>;
template<typename F>
using Quat_ = Eigen::Quaternion<F>;
template<typename F>
using Isometry3_ = Eigen::Transform<F, 3, Eigen::Isometry>;

using Vec3f = Vec3_<float>;
using Quatf = Quat_<float>;
using Isometry3f = Isometry3_<float>;

using Vec3d = Vec3_<double>;
using Quatd = Quat_<double>;
using Isometry3d = Isometry3_<double>;

using Vec3m = Vec3_<ldrp::measure_t>;
using Quatm = Quat_<ldrp::measure_t>;
using Isometry3m = Isometry3_<ldrp::measure_t>;



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
			.enable_segment_transforms	= config.enable_segment_transforms,
			.max_filter_threads			= ldru::convertNumThreads(config.max_filter_threads, 1),
			.points_logging_mode		= config.points_logging_mode,
			.points_log_fname			= config.points_tar_fname,
			.pose_matching_history			= ldru::floatSecondsToIntMicros(config.pose_matching_history_range_s),
			.pose_matching_max_delta		= ldru::floatSecondsToIntMicros(config.pose_matching_max_delta_s),
			.pose_matching_wait_increment	= ldru::floatSecondsToIntMicros(config.pose_matching_wait_increment_s),
			.pose_matching_wait_limit		= ldru::floatSecondsToIntMicros(config.pose_matching_wait_limit_s),
			.pose_matching_skip_invalid		= config.pose_matching_skip_invalid,

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

		this->_nt.last_parsed_seg_idx	= this->_nt.base->GetIntegerTopic( "last segment" )				.GetEntry( -1 );
		this->_nt.aquisition_cycles		= this->_nt.base->GetIntegerTopic( "aquisition loop count" )	.GetEntry( 0 );
		this->_nt.aquisition_ftime		= this->_nt.base->GetDoubleTopic( "aquisition frame time" )		.GetEntry( 0.0 );
		this->_nt.raw_scan_points		= this->_nt.base->GetRawTopic( "raw scan points" )				.GetEntry( "PointXYZ_[]", {} );
		this->_nt.test_filtered_points	= this->_nt.base->GetRawTopic( "filtered points" )				.GetEntry( "PointXYZ_[]", {} );

#if LDRP_ENABLE_NT_TUNING
		this->_nt.tuning.max_scan_theta			= this->_nt.base->GetFloatTopic( "tuning/max scan theta (deg)" )	.GetEntry( this->_config.fpipeline.max_scan_theta_deg );
		this->_nt.tuning.min_scan_theta			= this->_nt.base->GetFloatTopic( "tuning/min scan theta (deg)" )	.GetEntry( this->_config.fpipeline.min_scan_theta_deg );
		this->_nt.tuning.min_scan_range			= this->_nt.base->GetFloatTopic( "tuning/min scan range (cm)" )		.GetEntry( this->_config.fpipeline.min_scan_range_cm );
		this->_nt.tuning.voxel_size				= this->_nt.base->GetFloatTopic( "tuning/voxel size (cm)" )			.GetEntry( this->_config.fpipeline.voxel_size_cm );
		this->_nt.tuning.max_pmf_range			= this->_nt.base->GetFloatTopic( "tuning/pmf/max range (cm)" )		.GetEntry( this->_config.fpipeline.max_pmf_range_cm );
		this->_nt.tuning.max_z_thresh			= this->_nt.base->GetFloatTopic( "tuning/max z thresh (cm)" )		.GetEntry( this->_config.fpipeline.max_z_thresh_cm );
		this->_nt.tuning.min_z_thresh			= this->_nt.base->GetFloatTopic( "tuning/min z thresh (cm)" )		.GetEntry( this->_config.fpipeline.min_z_thresh_cm );
		this->_nt.tuning.pmf_window_base		= this->_nt.base->GetFloatTopic( "tuning/pmf/window base" )			.GetEntry( this->_config.fpipeline.pmf_window_base );
		this->_nt.tuning.pmf_max_window_size	= this->_nt.base->GetFloatTopic( "tuning/pmf/max window (cm)" )		.GetEntry( this->_config.fpipeline.pmf_max_window_size_cm );
		this->_nt.tuning.pmf_cell_size			= this->_nt.base->GetFloatTopic( "tuning/pmf/cell size (cm)" )		.GetEntry( this->_config.fpipeline.pmf_cell_size_cm );
		this->_nt.tuning.pmf_init_distance		= this->_nt.base->GetFloatTopic( "tuning/pmf/init distance (cm)" )	.GetEntry( this->_config.fpipeline.pmf_init_distance_cm );
		this->_nt.tuning.pmf_max_distance		= this->_nt.base->GetFloatTopic( "tuning/pmf/max distance (cm)" )	.GetEntry( this->_config.fpipeline.pmf_max_distance_cm );
		this->_nt.tuning.pmf_slope				= this->_nt.base->GetFloatTopic( "tuning/pmf/slope (cm)" )			.GetEntry( this->_config.fpipeline.pmf_slope );

		this->_nt.tuning.max_scan_theta			.Set( this->_config.fpipeline.max_scan_theta_deg );
		this->_nt.tuning.min_scan_theta			.Set( this->_config.fpipeline.min_scan_theta_deg );
		this->_nt.tuning.min_scan_range			.Set( this->_config.fpipeline.min_scan_range_cm );
		this->_nt.tuning.voxel_size				.Set( this->_config.fpipeline.voxel_size_cm );
		this->_nt.tuning.max_pmf_range			.Set( this->_config.fpipeline.max_pmf_range_cm );
		this->_nt.tuning.max_z_thresh			.Set( this->_config.fpipeline.max_z_thresh_cm );
		this->_nt.tuning.min_z_thresh			.Set( this->_config.fpipeline.min_z_thresh_cm );
		this->_nt.tuning.pmf_window_base		.Set( this->_config.fpipeline.pmf_window_base );
		this->_nt.tuning.pmf_max_window_size	.Set( this->_config.fpipeline.pmf_max_window_size_cm );
		this->_nt.tuning.pmf_cell_size			.Set( this->_config.fpipeline.pmf_cell_size_cm );
		this->_nt.tuning.pmf_init_distance		.Set( this->_config.fpipeline.pmf_init_distance_cm );
		this->_nt.tuning.pmf_max_distance		.Set( this->_config.fpipeline.pmf_max_distance_cm );
		this->_nt.tuning.pmf_slope				.Set( this->_config.fpipeline.pmf_slope );
#endif

	}
	void shutdownNT() {
		this->_nt.instance.Flush();
		this->_nt.instance.StopServer();
		this->_nt.instance.StopClient();
	}

public:
	// /** get the pose data that is most closely correlated with the provided timestamp */
	// status_t getMatchingSample(measure_t* pose, const uint64_t ts_us);
	/** add a world pose + linked timestamp */
	status_t addWorldRef(const measure_t* xyz, const measure_t* qxyzw, const uint64_t ts_us);
	/** export obstacles from the accumulator grid */
	status_t exportObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t));
	/** wait for the next accumulator update and export obstacles */
	status_t exportNextObstacles(ObstacleGrid& grid, ObstacleGrid::Quant_T*(*grid_resize)(size_t), double timeout_ms);
	/** export the newest localization pose */
	status_t exportLocalizationPose(measure_t* pose);
	/** wait for the next localization update and export the pose */
	status_t exportNextLocalizationPose(measure_t* pose, double timeout_ms);

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
		const bool enable_segment_transforms	= LidarConfig::STATIC_DEFAULT.enable_segment_transforms;
		const uint32_t max_filter_threads		= ldru::convertNumThreads(LidarConfig::STATIC_DEFAULT.max_filter_threads, 1);	// minimum of 1 thread, otherwise reserve the main thread and some extra margin for other processes
		const uint64_t points_logging_mode		= LidarConfig::STATIC_DEFAULT.points_logging_mode;
		const char* points_log_fname			= LidarConfig::STATIC_DEFAULT.points_tar_fname;
		const int64_t pose_matching_history			= ldru::floatSecondsToIntMicros(LidarConfig::STATIC_DEFAULT.pose_matching_history_range_s);		// MICROSECONDS --> the window only gets applied when we add new poses so this just needs account for the greatest difference between new poses getting added and points getting transformed in a thread
		const int64_t pose_matching_max_delta		= ldru::floatSecondsToIntMicros(LidarConfig::STATIC_DEFAULT.pose_matching_max_delta_s);			// ^ convert to from seconds as specified in header
		const int64_t pose_matching_wait_increment	= ldru::floatSecondsToIntMicros(LidarConfig::STATIC_DEFAULT.pose_matching_wait_increment_s);
		const int64_t pose_matching_wait_limit		= ldru::floatSecondsToIntMicros(LidarConfig::STATIC_DEFAULT.pose_matching_wait_limit_s);
		const bool pose_matching_skip_invalid		= LidarConfig::STATIC_DEFAULT.pose_matching_skip_invalid;	// skip points for which we don't have a pose from localization (don't use the default pose of nothing!)

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

#if LDRP_ENABLE_NT_TUNING
		struct {
			nt::FloatEntry
				max_scan_theta,
				min_scan_theta,
				min_scan_range,
				voxel_size,
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
			nt::FloatArrayEntry ts_offsets;
		} nt;
	};

protected:	// main internal entities
	std::unique_ptr<std::thread>
		lidar_thread{ nullptr };
	sick_scansegment_xd::PayloadFifo*
		udp_fifo{ nullptr };
	std::vector< std::unique_ptr<FilterInstance> >
		filter_threads{};	// need pointers since filter instance doesn't like to be copied (vector issue)
	std::deque<uint32_t>
		finished_queue{};
	ldru::TimestampSampler< std::tuple<Eigen::Isometry3f, Eigen::Vector3f, Eigen::Quaternionf>, int64_t >	// >>> int64_t timebase representing MICROSECONDS SINCE EPOCH <<<
		transform_sampler{};	// ^^ stores the transform as well as the source position and orientation! (NEW!)
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

uint64_t featureBits() {
	return (
		(!LDRP_USE_SIM_MODE)					<< 0 |
		(LDRP_USE_SIM_MODE & 2)					<< 1 |
		(LDRP_USE_SIM_MODE & 1)					<< 2 |
		(LDRP_USE_WPILIB)						<< 3 |
		(LDRP_SAFETY_CHECKS)					<< 4 |
		(LDRP_ENABLE_LOGGING)					<< 5 |
		(LDRP_DEBUG_LOGGING)					<< 6 |
		(LDRP_USE_ALL_ECHOS)					<< 7 |
		(LDRP_ENABLE_PRELIM_POINT_FILTERING)	<< 8 |
		(LDRP_ENABLE_NT_TUNING)					<< 9 |
		(LDRP_ENABLE_NT_PROFILING)				<< 10
	);
}


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

// status_t updateWorldPosition(const measure_t* xyz, const uint64_t ts_us) {
// 	if(LidarImpl::_global) {
// 		float _pose[7];
// 		if(status_t s = LidarImpl::_global->getMatchingSample(_pose, ts_us)) return (s | STATUS_FAIL);
// 		return LidarImpl::_global->addWorldRef(xyz, _pose + 3, ts_us);
// 	}
// 	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
// }
// status_t updateWorldOrientation(const measure_t* qxyzw, const uint64_t ts_us) {
// 	if(LidarImpl::_global) {
// 		float _pose[7];
// 		if(status_t s = LidarImpl::_global->getMatchingSample(_pose, ts_us)) return (s | STATUS_FAIL);
// 		return LidarImpl::_global->addWorldRef(_pose, qxyzw, ts_us);
// 	}
// 	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
// }
status_t updateWorldPose(const measure_t* xyz, const measure_t* qxyzw, const uint64_t ts_us) {
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

status_t getCalculatedPose(measure_t* pose) {
	if(LidarImpl::_global) {
		return LidarImpl::_global->exportLocalizationPose(pose);
	}
	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
}
status_t waitNextCalculatedPose(measure_t* pose, double timeout_ms) {
	if(LidarImpl::_global) {
		return LidarImpl::_global->exportNextLocalizationPose(pose, timeout_ms);
	}
	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
}





/** LidarImpl interfacing implementation */

// status_t LidarImpl::getMatchingSample(measure_t* pose, const uint64_t ts_us) {

// 	if(!pose) return STATUS_BAD_PARAM | STATUS_FAIL;

// 	this->_state.localization_mutex.lock_shared();
// 	const auto* ts_sample = this->transform_sampler.sampleTimestamped(ldru::rectifyTimestampMicros(ts_us));
// 	memcpy(pose, &std::get<1>(ts_sample->second), 3 * sizeof(measure_t));
// 	memcpy(pose + 3, &std::get<2>(ts_sample->second), 4 * sizeof(measure_t));
// 	this->_state.localization_mutex.unlock_shared();
// 	return STATUS_SUCCESS;

// }
status_t LidarImpl::addWorldRef(const measure_t* xyz, const measure_t* qxyzw, const uint64_t ts_us) {

	if(!xyz && !qxyzw) return STATUS_BAD_PARAM | STATUS_FAIL;	// if both are null there is nothing to correlate with

	const int64_t timestamp = ldru::rectifyTimestampMicros(ts_us);
	Eigen::Vector3f r2w_pos = Eigen::Vector3f::Identity();			// robot's position in the world
	Eigen::Quaternionf r2w_quat = Eigen::Quaternionf::Identity();	// robot's rotation in the world
	if(!xyz || !qxyzw) {
		this->_state.localization_mutex.lock_shared();
		const auto* ts_sample = this->transform_sampler.sampleTimestamped(timestamp);
		if(xyz) {
			r2w_pos = *reinterpret_cast<const Eigen::Vector3f*>(xyz);
		} else if(ts_sample) {
			r2w_pos = std::get<1>(ts_sample->second);
			// prev_timestamp = ts_sample->first;
		}
		if(qxyzw) {
			r2w_quat = Eigen::Quaternionf{ qxyzw };
		} else if(ts_sample) {
			r2w_quat = std::get<2>(ts_sample->second);
			// prev_timestamp = ts_sample->first;
		}
		this->_state.localization_mutex.unlock_shared();
	} else {	// seems redundant but we want to avoid locking/unlocking whenever we can
		r2w_pos = *reinterpret_cast<const Eigen::Vector3f*>(xyz);
		r2w_quat = Eigen::Quaternionf{ qxyzw };
	}

	/* >> FOR FUTURE REFERENCE!!! <<
		* "Eigen::QuaternionXX * Eigen::TranslationXX" IS NOT THE SAME AS "Eigen::TranslationXX * Eigen::QuaternionXX"
		* The first ROTATES the translation by the quaternion, whereas the second one DOES NOT!!! 
		* ALSO -- Eigen::QuaternionXX is stored internally as XYZW but has constructors that take various orderings! (make sure to check!) */
	const Eigen::Quaternionf
		l2w_quat = (this->_config.lidar_offset_quat * r2w_quat);	// lidar's rotation in the world
	const Eigen::Isometry3f
		r2w_transform = (*reinterpret_cast<const Eigen::Translation3f*>(&r2w_pos)) * r2w_quat;	// compose robot's position and rotation
	const Eigen::Vector3f
		l2w_pos = r2w_transform * this->_config.lidar_offset_xyz;	// transform lidar's offset in robot space -- the same as rotating the lidar position to global coords and adding the robot's position
	const Eigen::Isometry3f
		l2w_transform = (*reinterpret_cast<const Eigen::Translation3f*>(&l2w_pos)) * l2w_quat;	// compose the lidar's global position and the lidar's global rotation

	this->_state.localization_mutex.lock();
	this->transform_sampler.insert( timestamp, std::make_tuple( l2w_transform, r2w_pos, r2w_quat ) );
	this->transform_sampler.updateMin( timestamp - this->_config.pose_matching_history );		// TODO: "dumb" management for now -- add a more robust tracking mechanism later
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

status_t LidarImpl::exportLocalizationPose(measure_t* pose) {
	// TODO
	return STATUS_NOT_IMPLEMENTED;
}
status_t LidarImpl::exportNextLocalizationPose(measure_t* pose, double timeout_ms) {
	// TODO
	return STATUS_NOT_IMPLEMENTED;
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

	this->_state.obstacle_updates = 0;
	return STATUS_SUCCESS;

}











/** LidarImpl threads implementation */

// main thread
void LidarImpl::lidarWorker() {

	wpi::DataLogManager::SignalNewDSDataOccur();

	// (the next 30 lines or so are converted from scansegment_threads.cpp: sick_scansegment_xd::MsgPackThreads::runThreadCb())
	// init udp receiver
	sick_scansegment_xd::UdpReceiver* udp_receiver = nullptr; // Maybe change this to stack allocation
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

		nt::FloatArrayEntry
			test_imu_raw = this->_nt.base->GetFloatArrayTopic("multiscan imu/raw data").GetEntry({}),
			test_imu_pose = this->_nt.base->GetFloatArrayTopic("multiscan imu/pose").GetEntry({});

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
#if !LDRP_USE_SIM_MODE	// live sensor operation
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
#elif LDRP_USE_SIM_MODE == 1	// internal simulation
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
					// add timestamp!
					std::this_thread::sleep_until(s + 4900us);
					if constexpr(true) {
#elif LDRP_USE_SIM_MODE == 2	// pull data from nt (uesim)
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
						parsed_segment.timestamp_sec = (ts_raw.time / 1000000ULL);		// microseconds to seconds
						parsed_segment.timestamp_nsec = (ts_raw.time % 1000000ULL) * 1000ULL;	// leftover micros to nanoseconds
						filled_segments = this->_config.enabled_segments;	// export immediately
						valid = true;
					}

					std::this_thread::sleep_until(s + 4900us);
					if(valid) {
#else
				static_assert(false, "LDRP_USE_SIM_MODE macro must be in range [0, 2]! or true/false")
#endif
						LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Successfully parsed scan segment idx {}!", parsed_segment.segmentIndex )

						const uint64_t seg_bit = (1ULL << parsed_segment.segmentIndex);
						if(this->_config.enabled_segments & seg_bit) {	// if the segment'th bit is set
							size_t idx = ::countBitsBeforeN(this->_config.enabled_segments, parsed_segment.segmentIndex);	// get the index of the enabled bit (for our buffer)

							// log imu data
							if(parsed_segment.imudata.valid) {
								test_imu_raw.Set(std::span<float>{
									&parsed_segment.imudata.acceleration_x,
									&parsed_segment.imudata.acceleration_x + 10
								});
								float _pose[7], _quat[4];
								_pose[0] = 0;
								_pose[1] = 0;
								_pose[2] = 0;
								memcpy(_pose + 3, &parsed_segment.imudata.orientation_w, 16);
								test_imu_pose.Set(std::span<float>{
									_pose,
									_pose + 7
								});

								memcpy(_quat, &parsed_segment.imudata.orientation_x, 12);
								_quat[3] = parsed_segment.imudata.orientation_w;
								this->addWorldRef(nullptr, _quat, ldru::constructTimestampMicros(parsed_segment.timestamp_sec, parsed_segment.timestamp_nsec));
							}

							frame_segments[idx].emplace_front();	// create empty segment buffer
							ldru::swapSegmentsNoIMU(frame_segments[idx].front(), parsed_segment);		// efficient buffer transfer (no deep copying!)
							if(frame_segments[idx].size() >= this->_config.buffered_frames) {
								frame_segments[idx].resize(this->_config.buffered_frames);	// cut off oldest scans beyond the buffer size (resize() removes from the back)
								filled_segments |= seg_bit;		// save this segment as finished by enabling it's bit
							}
							// TODO: update min time bound for transform sampler
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





// filter thread(s)
void LidarImpl::filterWorker(LidarImpl::FilterInstance* f_inst) {
	// this function represents the alternative thread that filters the newest collection of points

	std::shared_ptr<nt::NetworkTable> nt_inst = this->_nt.base->GetSubTable( fmt::format("Filter Threads/inst {}", f_inst->index) );
	f_inst->nt.is_active = nt_inst->GetBooleanTopic("activity").GetEntry(false);
#if LDRP_ENABLE_NT_PROFILING
	f_inst->nt.proc_step = nt_inst->GetIntegerTopic("state").GetEntry(0);
	f_inst->nt.ts_offsets = nt_inst->GetFloatArrayTopic("ts matching debug").GetEntry({});

	#define _NT_PROFILE_STAGE(n)		f_inst->nt.proc_step.Set((n));
#else
	#define _NT_PROFILE_STAGE(...)
#endif

	f_inst->nt.is_active.Set(false);
	_NT_PROFILE_STAGE(0);	// 0 = init (pre looping)

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
		_NT_PROFILE_STAGE(10);	// 10 = step 1 init

		point_cloud.clear();	// clear the vector and set w,h to 0
		// point_ranges.clear();	// << MEMORY LEAK!!! (it was)

		Eigen::Vector3f avg_sample_origin{ 0.f, 0.f, 0.f };
		int64_t avg_sample_timestamp = 0;
		size_t avg_samples = 0;

#if LDRP_ENABLE_NT_TUNING
		const float		// all "unitted" parameters are normalized to be in meters and radians!
			_max_scan_theta			= this->_nt.tuning.max_scan_theta.Get() * (std::numbers::pi_v<float> / 180.f),
			_min_scan_theta			= this->_nt.tuning.min_scan_theta.Get() * (std::numbers::pi_v<float> / 180.f),
			_min_scan_range			= this->_nt.tuning.min_scan_range.Get() * 1e-2f,
			_voxel_size				= this->_nt.tuning.voxel_size.Get() * 1e-2f,
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
#else
		const float		// all "unitted" parameters are normalized to be in meters and radians!
			_max_scan_theta			= this->_config.fpipeline.max_scan_theta_deg * (std::numbers::pi_v<float> / 180.f),
			_min_scan_theta			= this->_config.fpipeline.min_scan_theta_deg * (std::numbers::pi_v<float> / 180.f),
			_min_scan_range			= this->_config.fpipeline.min_scan_range_cm * 1e-2f,
			_voxel_size				= this->_config.fpipeline.voxel_size_cm * 1e-2f,
			_max_pmf_range			= this->_config.fpipeline.max_pmf_range_cm * 1e-2f,
			_max_z_thresh			= this->_config.fpipeline.max_z_thresh_cm * 1e-2f,
			_min_z_thresh			= this->_config.fpipeline.min_z_thresh_cm * 1e-2f,
			_pmf_window_base		= this->_config.fpipeline.pmf_window_base,
			_pmf_max_window_size	= this->_config.fpipeline.pmf_max_window_size_cm * 1e-2f,
			_pmf_cell_size			= this->_config.fpipeline.pmf_cell_size_cm * 1e-2f,
			_pmf_init_distance		= this->_config.fpipeline.pmf_init_distance_cm * 1e-2f,
			_pmf_max_distance		= this->_config.fpipeline.pmf_max_distance_cm * 1e-2f,
			_pmf_slope				= this->_config.fpipeline.pmf_slope
		;
#endif

		_NT_PROFILE_STAGE(11);	// 11 = step 1 loop

		int64_t waited_accum_us = this->_config.pose_matching_wait_limit;

		// 1. transform points based on timestamp
		for(size_t i = 0; i < f_inst->samples.size(); i++) {			// we could theoretically multithread this part -- just use a mutex for inserting points into the master collection
			// TODO: need to have the option to transform all segments of the same frame equally (for SLAM) -- "this->_config.enable_segment_transforms"
			for(size_t j = 0; j < f_inst->samples[i].size(); j++) {
				const sick_scansegment_xd::ScanSegmentParserOutput& segment = f_inst->samples[i][j];
				const int64_t seg_ts = ldru::constructTimestampMicros(segment.timestamp_sec, segment.timestamp_nsec);	// microseconds since epoch in local timebase (internally generated using system clock)

				this->_state.localization_mutex.lock_shared();	// other threads can read from the buffer as well
				const auto* ts_transform = this->transform_sampler.sampleTimestamped( seg_ts );		// TODO: !!! THIS WILL LIKELY CAUSE PROBLEMS !!! >> if the vector gets realloced while we don't have mutex control, our pointer is no longer valid!
				while( (waited_accum_us > 0LL) && ((!ts_transform) || (ts_transform && (seg_ts - ts_transform->first) > this->_config.pose_matching_max_delta)) ) {	// invalid result or closest match is earlier than current, and outside of range... (not after, since we can't do anything about that!)
					
					_NT_PROFILE_STAGE(12);	// 12 = ts buffering
					this->_state.localization_mutex.unlock_shared();	// release temporarily
					
					crno::hrc::time_point a = crno::hrc::now();
					std::this_thread::sleep_for(crno::microseconds( this->_config.pose_matching_wait_increment ));	// attempt to wait for new data
					waited_accum_us -= crno::duration_cast<crno::microseconds>(crno::hrc::now() - a).count();	// subtract the amount we waited for
					_NT_PROFILE_STAGE(11);	// back to state for external scope

					this->_state.localization_mutex.lock_shared();		// relock
					ts_transform = this->transform_sampler.sampleTimestamped( seg_ts );
				
				}
				if(this->_config.pose_matching_skip_invalid && !ts_transform) {
					this->_state.localization_mutex.unlock_shared();
					continue;
				}

				const Eigen::Isometry3f transform = ts_transform ? std::get<0>(ts_transform->second) : DEFAULT_NO_POSE;	// we have to copy since the pointer may be invalidated once we release the mutex
				const int64_t sample_ts = ts_transform ? ts_transform->first : 0ULL;
				const size_t total_samples = this->transform_sampler.getSamples().size();
				this->_state.localization_mutex.unlock_shared();

#if LDRP_ENABLE_NT_PROFILING
				const float _debug[4] = {
					seg_ts,
					sample_ts,
					static_cast<float>(seg_ts - sample_ts),
					static_cast<float>(total_samples)
				};
				f_inst->nt.ts_offsets.Set(std::span<const float>{ _debug, _debug + 4 });
#endif

				avg_sample_origin += transform.translation();
				avg_sample_timestamp += sample_ts;
				avg_samples++;

				for(const sick_scansegment_xd::ScanSegmentParserOutput::Scangroup& scan_group : segment.scandata) {		// "ms100 transmits 16 groups"
#if LDRP_USE_ALL_ECHOS
					for(const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line: scan_group.scanlines) {	// "each group has up to 3 echos"
#else
					if(scan_group.scanlines.size() > 0) {
						const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line = scan_group.scanlines[0];
#endif
						for(const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& lidar_point : scan_line.points) {
#if LDRP_ENABLE_PRELIM_POINT_FILTERING
							if(								// if angle in range...									// and range greater than minimum
								lidar_point.azimuth <= _max_scan_theta && lidar_point.azimuth >= _min_scan_theta && lidar_point.range > _min_scan_range
								// ...apply any other filters that benefit from points in lidar scan coord space as well
							) {
#else
							{
#endif
								point_cloud.points.emplace_back();
								reinterpret_cast<Eigen::Vector4f&>(point_cloud.points.back()) = transform * Eigen::Vector4f{lidar_point.x, lidar_point.y, lidar_point.z, 1.f};	// EEEEEE :O
								// point_ranges.push_back(lidar_point.range);
							}	// prelim filtering (or blank scope)
						}	// loop points
					}	// loop echos (or use first)
				} // loop points per segment

			}

			f_inst->samples[i].clear();		// clear the queue so that when the buffer gets swapped back to main, the queues are fresh

		} // loop segments

		LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Filter Instance {} [Filter Loop]: Collected {} points", f_inst->index, point_cloud.size() )

		point_cloud.width = point_cloud.points.size();
		point_cloud.height = 1;
		point_cloud.is_dense = true;

		if(this->_config.points_logging_mode & POINT_LOGGING_INCLUDE_RAW) {
			_NT_PROFILE_STAGE(13);	// 13 = export raw cloud
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

			_NT_PROFILE_STAGE(20);	// 20 = filtering (init)

			voxelized_points.clear();
			voxelized_ranges.clear();

			z_high_filtered.clear();
			z_low_subset_filtered.clear();
			z_mid_filtered_obstacles.clear();
			pre_pmf_range_filtered.clear();
			pmf_filtered_ground.clear();
			pmf_filtered_obstacles.clear();
			combined_obstacles.clear();

			if(avg_samples > 1) {
				avg_sample_origin /= avg_samples;
				avg_sample_timestamp /= avg_samples;
			}

			// voxelize points
			_NT_PROFILE_STAGE(21);	// 21 = filtering (voxelize)
			voxel_filter(
				point_cloud, DEFAULT_NO_SELECTION, voxelized_points,
				_voxel_size, _voxel_size, _voxel_size
			);

			// filter points under "high cut" thresh
			_NT_PROFILE_STAGE(22);	// 22 = filtering (z-high)
			carteZ_filter(
				voxelized_points, DEFAULT_NO_SELECTION, z_high_filtered,
				-std::numeric_limits<float>::infinity(),
				_max_z_thresh
			);
			// further filter points below "low cut" thresh
			_NT_PROFILE_STAGE(23);	// 23 = filtering (z-low)
			carteZ_filter(
				voxelized_points, z_high_filtered, z_low_subset_filtered,
				-std::numeric_limits<float>::infinity(),
				_min_z_thresh
			);
			// get the points inbetween high and low thresholds --> treated as wall obstacles
			_NT_PROFILE_STAGE(24);	// 24 = filtering (z-mid)
			pc_negate_selection(
				z_high_filtered,
				z_low_subset_filtered,
				z_mid_filtered_obstacles
			);

			// filter close enough points for PMF
			_NT_PROFILE_STAGE(25);	// 25 = filtering (pmf-pre)
			pc_filter_distance(
				voxelized_points.points,
				z_low_subset_filtered,
				pre_pmf_range_filtered,
				0.f, _max_pmf_range,
				avg_sample_origin
			);

			// apply pmf to selected points
			_NT_PROFILE_STAGE(26);	// 26 = filtering (pmf)
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
			_NT_PROFILE_STAGE(27);	// 27 = filtering (obstacles)
			pc_negate_selection(
				pre_pmf_range_filtered,
				pmf_filtered_ground,
				pmf_filtered_obstacles
			);

			// export filter results
			if(this->_config.points_logging_mode & (POINT_LOGGING_NT | POINT_LOGGING_INCLUDE_FILTERED)) {

				_NT_PROFILE_STAGE(28);	// 28 = filtering (export)

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

		// 3. localization refinement
		{
			// INTEGRATED LOCALIZATION HERE!!!
		}
		// 4. full transform to global space
		{
			// only applies to filtered obstacle points :)
		}

		// 5. update accumulator
		{
			_NT_PROFILE_STAGE(30);	// 30 = update grid (locking)

			this->_state.accumulation_mutex.lock();

			_NT_PROFILE_STAGE(31);	// 31 = update grid (pmf insert)
			this->accumulator.incrementRatio(	// insert PMF obstacles
				voxelized_points,
				pre_pmf_range_filtered,		// base
				pmf_filtered_obstacles		// subset
			);
			_NT_PROFILE_STAGE(32);	// 32 = update grid (z-insert)
			this->accumulator.incrementRatio(	// insert z-thresh obstacles
				voxelized_points,
				z_mid_filtered_obstacles,	// base
				DEFAULT_NO_SELECTION		// use all of base
			);

			_NT_PROFILE_STAGE(33);	// 33 = update grid (cleanup)
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
		_NT_PROFILE_STAGE(40);	// 40 = finished
		f_inst->link_state = false;
		std::unique_lock<std::mutex> lock{ this->_state.finished_queue_mutex };
		this->finished_queue.push_back(f_inst->index);
		// wait for signal to continue...
		_NT_PROFILE_STAGE(41);	// 41 = waiting for update
		for(;this->_state.enable_threads.load() && !f_inst->link_state;) {
			f_inst->link_condition.wait(lock);
		}

	}	// thread loop

#undef _NT_PROFILE_STAGE

}	// LidarImpl::filterWorker()


};	// namespace ldrp
