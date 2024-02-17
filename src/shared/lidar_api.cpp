#include "lidar_api.h"

#include "mem_utils.h"
#include "pcd_streaming.h"

#include <condition_variable>
#include <shared_mutex>
#include <type_traits>
#include <algorithm>
#include <iostream>
#include <numbers>
#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>
#include <utility>
#include <deque>
#include <mutex>
#include <vector>
#include <span>

#include <pcl/pcl_config.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

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


/** Util Definitions */

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


/** Other helpers */

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


	/** Interfacing and Filtering Implementation (singleton usage) */
	struct LidarImpl {
	public:
		LidarImpl(const char* dlm_dir = "", const char* dlm_fname = "", double dlm_period = 0.25) {
			wpi::DataLogManager::Start(dlm_dir, dlm_fname, dlm_period);
			this->initNT();

			LDRP_LOG( LOG_ALWAYS, "LDRP global instance initialized." )
		}
		~LidarImpl() {
			this->_state.enable_threads.store(false);
			if(this->_lidar_thread->joinable()) {
				this->_lidar_thread->join();
			}
			this->shutdownNT();

			LDRP_LOG( LOG_ALWAYS, "LDRP global instance destroyed." )
			wpi::DataLogManager::Stop();
		}

		LidarImpl(const LidarImpl&) = delete;
		LidarImpl(LidarImpl&&) = delete;


		void initNT() {
			this->_nt.instance = nt::NetworkTableInstance::GetDefault();

			this->_nt.instance.StartServer();	// config or auto-detect for server/client
			this->_nt.base = this->_nt.instance.GetTable("Perception");

			this->_nt.last_parsed_seg_idx = this->_nt.base->GetIntegerTopic("last segment").GetEntry(-1);
			this->_nt.aquisition_cycles = this->_nt.base->GetIntegerTopic("aquisition loop count").GetEntry(0);
			this->_nt.aquisition_ftime = this->_nt.base->GetDoubleTopic("aquisition frame time").GetEntry(0.0);
			this->_nt.raw_scan_points = this->_nt.base->GetRawTopic("raw scan points").GetEntry( "PointXYZ_[]", {} );
		}
		void shutdownNT() {
			this->_nt.instance.Flush();
			this->_nt.instance.StopServer();
		}


	public:	// global inst, constant values, configs
		inline static std::unique_ptr<LidarImpl> _global{ nullptr };

		enum : uint32_t {
			PCD_LOGGING_NONE	= 0,
			PCD_LOGGING_TAR		= (1 << 1),
			PCD_LOGGING_NT		= (1 << 2),
		};
		static constexpr size_t
			MS100_SEGMENTS_PER_FRAME = 12U,
			MS100_POINTS_PER_SEGMENT_ECHO = 900U,	// points per segment * segments per frame = 10800 points per frame (with 1 echo)
			MS100_MAX_ECHOS_PER_POINT = 3U;			// echos get filterd when we apply different settings in the web dashboard
		struct {	// configured constants/parameters
			const std::string lidar_hostname{ "" };	// always fails when we give a specific hostname?
			const int lidar_udp_port{ 2115 };
			/* "While MSGPACK can be integrated easily using existing
			 * libraries and is easy to parse, it requires more computing power and bandwidth than
			 * the compact data format due to the descriptive names. Compact is significantly more
			 * efficient and has a lower bandwidth." */
			const bool use_msgpack{ false };

			// NOTE: points within the same segment are not 'rotationally' aligned! (only temporally aligned)
			const uint64_t enabled_segments{ 0b111111111111 };	// 12 sections --> first 12 bits enabled (enable all section)
			const uint32_t buffered_frames{ 1 };				// how many samples of each segment we require per aquisition
			const uint32_t max_filter_threads{ (uint32_t)std::max((int)std::thread::hardware_concurrency() - 2, 1) };		// minimum of 1 thread, otherwise reserve the main thread and some extra margin for other processes
			const uint32_t pcd_logging_mode{ PCD_LOGGING_NT };
			const char* pcd_log_fname{ "lidar_points.tar" };
			const double pose_storage_window{ 0.1 };	// how many seconds
			const bool skip_invalid_pose_ts{ false };	// skip points for which we don't have a pose directly from localization

			struct {
				float
					min_scan_theta_deg =	-90.f,			// max scan theta angle used for cutoff -- see ms100 operating manual for coordinate system
					max_scan_theta_deg =	+90.f,			// min scan theta angle used for cutoff
					min_scan_range_cm =		10.f,			// the minimum scan range
					// filter by intensity? (test this)

					voxel_size_cm =			3.f,			// voxel cell size used during voxelization filter
					map_resolution_cm =		5.f,		// the resolution of each grid cell
					pmf_window_base_cm =	0.4f,
					pmf_cell_size_cm =		5.f,
					pmf_init_distance_cm =	5.f,
					pmf_max_distance_cm =	12.f,
					pmf_slope =				0.2f;
				int32_t
					pmf_max_window_size =	40;

			} fpipeline;
		} _config;

		struct {	// states to be connunicated across threads
			int32_t log_level{ 1 };

			std::atomic<bool> enable_threads{false};
		} _state;

	protected:	// nt pointers and filter instance struct
		struct {	// networktables
			nt::NetworkTableInstance instance;
			std::shared_ptr<nt::NetworkTable> base;

			nt::IntegerEntry
				last_parsed_seg_idx,
				aquisition_cycles;
			nt::DoubleEntry aquisition_ftime;
			nt::RawEntry raw_scan_points;
		} _nt;

		using SampleBuffer = std::vector< std::deque< sick_scansegment_xd::ScanSegmentParserOutput > >;
		struct FilterInstance {	// storage for each filter instance that needs to be synced between threads
			FilterInstance(const uint32_t f_idx) : index{f_idx} {}
			FilterInstance(const FilterInstance&) = delete;
			FilterInstance(FilterInstance&&) = default;

			const uint32_t index;
			SampleBuffer samples{};

			std::unique_ptr<std::thread> thread{ nullptr };
			std::mutex link_mutex;
			std::condition_variable link_condition;
			std::atomic<uint32_t> link_state{ 0 };

			struct {
				nt::BooleanEntry is_active;
			} nt;
		};

	public:	// main instance members
		std::unique_ptr<std::thread> _lidar_thread{ nullptr };
		std::vector<std::unique_ptr<FilterInstance> > _filter_threads{};	// need pointers since filter instance doesn't like to be copied (vector impl)
		std::deque<uint32_t> _finished_queue{};
		std::mutex
			_finished_queue_mutex{};
		std::shared_mutex
			_pose_mutex{};
		frc::TimeInterpolatableBuffer<frc::Pose3d> _pose_buffer{
			units::time::second_t{ _config.pose_storage_window },
			&lerpClosest<const frc::Pose3d&>
		};
		PCDTarWriter pcd_writer{};


	public:
		void filterWorker(FilterInstance* f_inst) {
			// this function represents the alternative thread that filters the newest collection of points

			f_inst->nt.is_active = this->_nt.base->GetSubTable("Filter Theads")->GetBooleanTopic( fmt::format("inst {} activity", f_inst->index) ).GetEntry(false);

			// precalulcate values
			const size_t max_points{ 
				(MS100_POINTS_PER_SEGMENT_ECHO * MS100_MAX_ECHOS_PER_POINT)
					* ::countBits(this->_config.enabled_segments)
					* this->_config.buffered_frames
			};
			const frc::Pose3d default_pose{};
			pcl::PointCloud<pcl::PointXYZ> point_cloud;		// reuse the buffer
			point_cloud.points.reserve( max_points );

			LDRP_LOG( LOG_STANDARD, "LDRP Filter Instance {} [Init]: Resources initialized - running filter loop...", f_inst->index )

			for(;this->_state.enable_threads.load();) {

				f_inst->nt.is_active.Set(true);

				// 1. transform points based on timestamp
				point_cloud.clear();	// clear the vector and set w,h to 0

				for(size_t i = 0; i < f_inst->samples.size(); i++) {			// we could theoretically multithread this part -- just use a mutex for inserting points into the master collection
					for(size_t j = 0; j < f_inst->samples[i].size(); j++) {
						const sick_scansegment_xd::ScanSegmentParserOutput& segment = f_inst->samples[i][j];

						_pose_mutex.lock_shared();	// other threads can read from the buffer as well
						std::optional<frc::Pose3d> ts_pose = this->_pose_buffer.Sample(
							units::time::second_t{ segment.timestamp_sec + (segment.timestamp_nsec * 1E-9) } );
						_pose_mutex.unlock_shared();

						if(this->_config.skip_invalid_pose_ts && !ts_pose.has_value()) continue;
						const frc::Pose3d& pose = ts_pose.has_value() ? *ts_pose : default_pose;	// need to convert to transform matrix

						for(const sick_scansegment_xd::ScanSegmentParserOutput::Scangroup& scan_group : segment.scandata) {		// "ms100 transmits 16 groups"
#define USE_FIRST_ECHO_ONLY		// make a config option or grouping for this instead
#ifndef USE_FIRST_ECHO_ONLY
							for(const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line: scan_group.scanlines) {	// "each group has up to 3 echos"
#else
							if(scan_group.scanlines.size() > 0) {
								const sick_scansegment_xd::ScanSegmentParserOutput::Scanline& scan_line = scan_group.scanlines[0];
#endif
								for(const sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint& lidar_point : scan_line.points) {

#define DISABLE_PRELIM_POINT_FILTERING	// option for this as well
#ifndef DISABLE_PRELIM_POINT_FILTERING
									const float azimuth_deg = lidar_point.azimuth * 180.f / std::numbers::pi_v<float>;
									if(
										(azimuth_deg <= this->_config.fpipeline.max_scan_theta_deg && azimuth_deg >= this->_config.fpipeline.max_scan_theta_deg)
											&& (lidar_point.range * 1e2f > this->_config.fpipeline.min_scan_range_cm)
									) {
#else
									{
#endif
										// TODO: transform point!
										point_cloud.points.emplace_back(lidar_point.x, lidar_point.y, lidar_point.z);
									}
								}
							}
						} // loop points per segment

					}

					f_inst->samples[i].clear();		// clear the queue so that when the buffer gets swapped back to main, the queues are fresh

				} // loop segments

				LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Filter Instance {} [Filter Loop]: Collected {} points", f_inst->index, point_cloud.size() )

				if(this->_config.pcd_logging_mode) {
					point_cloud.width = point_cloud.points.size();
					point_cloud.height = 1;
					point_cloud.is_dense = true;
					// point_cloud.header.stamp = {};	// somehow average all the segment timestamps?

					if(this->_config.pcd_logging_mode & PCD_LOGGING_TAR) {
						this->pcd_writer.addCloud(point_cloud);
					}
					if(this->_config.pcd_logging_mode & PCD_LOGGING_NT) {
						this->_nt.raw_scan_points.Set(
							std::span<const uint8_t>{
								reinterpret_cast<uint8_t*>( point_cloud.points.data() ),
								reinterpret_cast<uint8_t*>( point_cloud.points.data() + point_cloud.points.size() )
							}
						);
					}
				}

				// 2. run filtering on points
				// 3. update accumulator
				// 4. [when configured] update map

				// processing finished, push instance idx to queue
				f_inst->nt.is_active.Set(false);
				f_inst->link_state = false;
				std::unique_lock<std::mutex> lock{ this->_finished_queue_mutex };
				this->_finished_queue.push_back(f_inst->index);
				// wait for signal to continue...
				for(;this->_state.enable_threads.load() && !f_inst->link_state;) {
					f_inst->link_condition.wait(lock);
				}

			}

		}
		void lidarWorker() {

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

				sick_scansegment_xd::PayloadFifo* udp_fifo = udp_receiver->Fifo();
				std::vector<uint8_t> udp_payload_bytes{};
				sick_scansegment_xd::ScanSegmentParserOutput parsed_segment{};
				fifo_timestamp scan_timestamp{};
				size_t scan_count{0};	// not used but we need for a param

				if(this->_config.pcd_logging_mode & PCD_LOGGING_TAR) {
					this->pcd_writer.setFile(this->_config.pcd_log_fname);
				}

				// main loop!
				LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: Succesfully initialized all resources. Begining aquisition and filtering..." )
				for(;this->_state.enable_threads.load();) {

					// wpi::DataLogManager::GetLog().Flush();

					frame_segments.resize( ::countBits(this->_config.enabled_segments) );
					const crno::hrc::time_point aquisition_start = crno::hrc::now();
					size_t aquisition_loop_count = 0;
					for(uint64_t filled_segments = 0; this->_state.enable_threads; aquisition_loop_count++) {	// loop until thread exit is called... (internal break allows for exit as well)

						if(filled_segments >= this->_config.enabled_segments) {	// if we have aquired sufficient samples...
							LDRP_LOG( LOG_DEBUG, "LDRP Worker [Aquisition Loop]: Aquisition quota satisfied after {} loops - exporting buffer to thread...", aquisition_loop_count )

							// attempt to find or create a thread for processing the frame
							this->_finished_queue_mutex.lock();	// aquire mutex for queue
							if(this->_finished_queue.size() > 0) {

								const uint32_t filter_idx = this->_finished_queue.front();	// maybe check that this is valid?
								this->_finished_queue.pop_front();
								this->_finished_queue_mutex.unlock();
								FilterInstance& f_inst = *this->_filter_threads[filter_idx];
								std::swap(f_inst.samples, frame_segments);		// figure out where we want to clear the buffer that is swapped in so we don't start with old data in the queues
								f_inst.link_state.store(true);
								f_inst.link_condition.notify_all();
								break;

							} else {
								this->_finished_queue_mutex.unlock();
								if(this->_filter_threads.size() < this->_config.max_filter_threads) {	// start a new thread

									// create a new thread and swap in the sample
									this->_filter_threads.emplace_back( std::make_unique<FilterInstance>( static_cast<uint32_t>(this->_filter_threads.size()) ) );
									FilterInstance& f_inst = *this->_filter_threads.back();
									std::swap(f_inst.samples, frame_segments);
									f_inst.thread.reset( new std::thread{&LidarImpl::filterWorker, this, &f_inst} );
									break;

								}
							}
						}	// insufficient samples or no thread available... (keep updating the current framebuff)
// #define SIM_GENERATE_POINTS
#ifndef SIM_GENERATE_POINTS
						if(udp_fifo->Pop(udp_payload_bytes, scan_timestamp, scan_count)) {	// blocks until packet is received

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
#else
						{
							crno::hrc::time_point s = crno::hrc::now();
							parsed_segment.scandata.resize(16);
							for(size_t h = 0; h < 14; h++) {
								auto& group = parsed_segment.scandata[h];
								group.scanlines.resize(1);
								auto& line = group.scanlines[0];
								line.points.resize(30);
								for(size_t v = 0; v < 30; v++) {
									line.points[v] = sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint{
										(float)std::rand() / RAND_MAX * 100.f,
										(float)std::rand() / RAND_MAX * 100.f,
										(float)std::rand() / RAND_MAX * 100.f,
										0.f, 0.f, 0.f, 0.f, 0U, 0U, 0U, 0U
									};
								}
							}
							for(size_t h = 14; h < 16; h++) {
								auto& group = parsed_segment.scandata[h];
								group.scanlines.resize(1);
								auto& line = group.scanlines[0];
								line.points.resize(240);
								for(size_t v = 0; v < 240; v++) {
									line.points[v] = sick_scansegment_xd::ScanSegmentParserOutput::LidarPoint{
										(float)std::rand() / RAND_MAX * 100.f,
										(float)std::rand() / RAND_MAX * 100.f,
										(float)std::rand() / RAND_MAX * 100.f,
										0.f, 0.f, 0.f, 0.f, 0U, 0U, 0U, 0U
									};
								}
							}
							parsed_segment.segmentIndex = aquisition_loop_count;
							std::this_thread::sleep_until(s + (50ms / 12));
							if constexpr(true) {
#endif
								LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Successfully parsed scan segment idx {}!", parsed_segment.segmentIndex )

								const uint64_t seg_bit = (1ULL << parsed_segment.segmentIndex);
								if(this->_config.enabled_segments & seg_bit) {	// if the segment'th bit is set
									size_t idx = ::countBitsBeforeN(this->_config.enabled_segments, parsed_segment.segmentIndex);	// get the index of the enabled bit (for our buffer)

									frame_segments[idx].emplace_front();	// create empty segment buffer
									::swapSegmentsNoIMU(frame_segments[idx].front(), parsed_segment);		// efficient buffer transfer (no deep copying!)
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

				}

			} else {	// udp receiver launch thread
				LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver thread failed to start. Exitting..." )
			}

			this->pcd_writer.closeIO();		// doesn't do anything if we never initialized

			// join and delete all filter instances
			for(std::unique_ptr<FilterInstance>& f_inst : this->_filter_threads) {
				if(f_inst->thread && f_inst->thread->joinable()) {
					f_inst->thread->join();
				}
				delete f_inst->thread.release();
			}

			// close and deallocate udp receiver
			udp_receiver->Fifo()->Shutdown();
			udp_receiver->Close();
			delete udp_receiver;

		}	// lidarWorker


	};	// LidarImpl





/** Static API */

	const char* pclVer() {
		return PCL_VERSION_PRETTY;
	}
	const bool hasWpilib() {
		return USING_WPILIB;
	}

	const status_t apiInit(const char* dlm_dir, const char* dlm_fname, double dlm_period, const int32_t log_lvl) {
		if(!LidarImpl::_global) {
			LidarImpl::_global = std::make_unique<LidarImpl>(dlm_dir, dlm_fname, dlm_period);
			return setLogLevel(log_lvl);
		}
		return STATUS_ALREADY_SATISFIED;
	}
	const status_t apiDestroy() {
		if(LidarImpl::_global) {
			LidarImpl::_global.reset( nullptr );
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_ALREADY_SATISFIED;
	}

	const status_t lidarInit() {
		if(LidarImpl::_global) {
			if(!LidarImpl::_global->_lidar_thread || !LidarImpl::_global->_lidar_thread->joinable()) {
				LidarImpl::_global->_state.enable_threads.store(true);
				LidarImpl::_global->_lidar_thread.reset(
					new std::thread{ &LidarImpl::lidarWorker, LidarImpl::_global.get() }
				);
				return STATUS_SUCCESS;
			}
			return STATUS_ALREADY_SATISFIED;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarShutdown() {
		if(LidarImpl::_global) {
			status_t s = STATUS_ALREADY_SATISFIED;
			LidarImpl::_global->_state.enable_threads.store(false);
			if(LidarImpl::_global->_lidar_thread && LidarImpl::_global->_lidar_thread->joinable()) {
				LidarImpl::_global->_lidar_thread->join();
				s = STATUS_SUCCESS;
			}
			LidarImpl::_global->_lidar_thread.reset(nullptr);		// don't need to do this but probably good to explicitly signify the thread isn't valid
			return s;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarSetState(const bool enabled) {
		return enabled ? lidarInit() : lidarShutdown();
	}

	const status_t setLogLevel(const int32_t lvl) {
		if(lvl < 0 || lvl > 3) return STATUS_BAD_PARAM;
		if(LidarImpl::_global) {
			LidarImpl::_global->_state.log_level = lvl;
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}

	const status_t updateWorldPose(const float* xyz, const float* qxyz, const float qw, const uint64_t ts_ms) {
		if(LidarImpl::_global) {
			const units::time::second_t timestamp{
				ts_ms == 0 ?
					crno::duration<double>{ crno::hrc::now().time_since_epoch() }.count() :
					(double)ts_ms / 1e6
			};
			LidarImpl::_global->_pose_mutex.lock();
			LidarImpl::_global->_pose_buffer.AddSample(
				timestamp,
				frc::Pose3d{
					units::meter_t{ xyz[0] },
					units::meter_t{ xyz[1] },
					units::meter_t{ xyz[2] },
					frc::Rotation3d{
						frc::Quaternion{
							qw,
							qxyz[1],
							qxyz[2],
							qxyz[3]
						}
					}
				}
			);
			LidarImpl::_global->_pose_mutex.unlock();
			return STATUS_SUCCESS;
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