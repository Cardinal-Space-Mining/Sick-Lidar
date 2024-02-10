#include "lidar_api.h"

#include "mem_utils.h"

#include <condition_variable>
#include <type_traits>
#include <algorithm>
#include <iostream>
#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>
#include <utility>
#include <deque>
#include <mutex>
#include <vector>

#include <pcl/pcl_config.h>

#ifndef USING_WPILIB
  #define USING_WPILIB false
#endif

#if USING_WPILIB
#include <frc/geometry/Pose3d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/IntegerTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <DataLogManager.h>
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
#if __cplusplus <= 201703L		// sourced from C++20 definition
		template <class _Clock, class = void>
		inline constexpr bool _Is_clock_v = false;

		template <class _Clock>
		inline constexpr bool
			_Is_clock_v<_Clock, std::void_t<typename _Clock::rep, typename _Clock::period, typename _Clock::duration,
									typename _Clock::time_point, decltype(_Clock::is_steady), decltype(_Clock::now())>> =
				true;

		template <class _Clock>
		struct is_clock : bool_constant<_Is_clock_v<_Clock>> {};
		template <class _Clock>
		inline constexpr bool is_clock_v = _Is_clock_v<_Clock>;
#endif
	}
}
namespace crno = std::chrono;





/** TimestampedQueue utility datastructure */

template<typename V, typename C = crno::high_resolution_clock>
class TimestampedQueue {	// use wpilib (wpimath) frc::TimeInterpolatableBuffer instead
	static_assert(crno::is_clock<C>::value, "");
public:
	using Value_T = V;
	using Clock_T = C;
	using Duration_T = typename Clock_T::duration;
	using TimeStamp_T = crno::time_point<Clock_T>;
	using Data_T = std::pair<TimeStamp_T, Value_T>;

public:
	TimestampedQueue() {}
	~TimestampedQueue() {}

	void add(const Value_T& value, const TimeStamp_T ts = Clock_T::now()) {
		this->_queue.push_front(
			Data_T{ value, ts }
		);
	}
	void add(Value_T&& value, const TimeStamp_T ts = Clock_T::now()) {
		this->_queue.push_front(
			Data_T{ std::move(value), ts }
		);
	}

	Value_T* getClosest(const TimeStamp_T ts = Clock_T::now()) {
		// DO NOT USE -- TODO: implement a functional algorithm
		if(this->_queue.empty()) {
			return nullptr;
		} else if(ts > this->_queue.front().first) {
			return &(this->_queue.front().second);
		}
		for(size_t i = this->_queue.size() / 2;;) {
			if(i + 1 >= this->_queue.size()) {
				return &this->_queue.back().second;
			}
			const TimeStamp_T
				&f = this->_queue[i].first,
				&b = this->_queue[i + 1].first;
			const Duration_T
				_f = (f - ts),
				_b = (ts - b);
			if(_f >= 0 && _b >= 0) {
				return _f < _b ? &this->_queue[i].second : &this->_queue[i + 1].second;	// return value that is closest
			} else if(_f < 0) {
				i /= 2;		// go forwards
			} else {	// _b < 0
				i += (i / 2);	// go backwards
			}
		}
	}
	// const Value_T& getClosest(const TimeStamp_T ts = Clock_T::now()) const {

	// }

protected:
	void truncate(const Duration_T window_size) {
		const TimeStamp_T min = this->_queue.front().first - window_size;
		for(;this->_queue.back().first < min;) {
			this->_queue.pop_back();
		}
	}

protected:
	std::deque<Data_T> _queue;


};





/** Wpilib TimeInterpolatableBuffer helpers */
#if USING_WPILIB
template<typename T>
static T&& lerpClosest(T&& a, T&& b, double t) {	// t is nominally in [0, 1]
	return (abs(t) < abs(1.0 - t)) ? std::forward<T>(a) : std::forward<T>(b);
}
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
		}
		void shutdownNT() {
			this->_nt.instance.Flush();
			this->_nt.instance.StopServer();
		}


	public:
		inline static std::unique_ptr<LidarImpl> _global{ nullptr };


		static constexpr size_t SEGMENTS_PER_FRAME = 12;
		struct {	// configured constants/parameters
			const std::string lidar_hostname{ "" };	// always fails when we give a specific hostname?
			const int lidar_udp_port{ 2115 };
			const bool use_msgpack{ true };

			const uint64_t enabled_segments{ 0b111111111111 };	// 12 sections --> first 12 bits enabled (enable all section)
			const uint32_t buffered_scans{ 1 };		// how many samples of each segment we require per aquisition
			const uint32_t max_filter_threads{ 1 };
			const bool enable_pcd_logging{ true };
		} _config;

		struct {	// states to be connunicated across threads
			int32_t log_level{ 1 };

			std::atomic<bool> enable_threads{false};
		} _state;

	protected:
		struct {	// networktables
			nt::NetworkTableInstance instance;
			std::shared_ptr<nt::NetworkTable> base;

			nt::IntegerEntry
				last_parsed_seg_idx,
				aquisition_cycles;
			nt::DoubleEntry aquisition_ftime;
		} _nt;

		using SampleBuffer = std::vector< std::deque< sick_scansegment_xd::ScanSegmentParserOutput > >;
		struct FilterContext {	// storage for each filter instance that needs to be synced between threads
			std::unique_ptr<std::thread> thread{ nullptr };
			SampleBuffer samples{};
			std::condition_variable link_state;
			std::mutex link_mutex;
			uint32_t index;
		};

	public:
		std::unique_ptr<std::thread> _lidar_thread{ nullptr };
		std::vector<ldrp::LidarImpl::FilterContext> _filter_threads{ 0 };
		std::deque<uint32_t> _finished_threads{};
		std::mutex
			_finished_queue_mutex{};
			_pose_mutex{};

		struct {	// filtering static buffers
			
		} _processing;


	public:
		void filterWorker() {
			// this function represents the alternative thread that filters the newest collection of points

			// 1. transform points based on timestamp
			// 2. run filtering on points
			// 3. update accumulator
			// 4. [when configured] update map
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
					this->_config.buffered_scans * SEGMENTS_PER_FRAME,	// udp fifo length -- really we should only need 1 or 2?
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
			// launch udp receiver
			if(udp_receiver->Start()) {
				LDRP_LOG( LOG_STANDARD, "LDRP Worker [Init]: UdpReceiver thread successfully launched!" )

				// defaults for crap we don't use
				static sick_scan_xd::SickCloudTransform no_transform{};
				static sick_scansegment_xd::MsgPackValidatorData default_validator_data_collector{};
				static sick_scansegment_xd::MsgPackValidator default_validator{};
				static sick_scansegment_xd::ScanSegmentParserConfig default_parser_config{};

				// a queue for each segment we are sampling from so we can have a rolling set of aquired samples (when configured)
				SampleBuffer frame_segments{ ::countBits(this->_config.enabled_segments) };	// init size to the number of enabled segments

				sick_scansegment_xd::PayloadFifo* udp_fifo = udp_receiver->Fifo();
				std::vector<uint8_t> udp_payload_bytes{};
				sick_scansegment_xd::ScanSegmentParserOutput parsed_segment{};
				fifo_timestamp scan_timestamp{};
				size_t scan_count{0};	// not used but we need for a param

				// main loop!
				for(;this->_state.enable_threads.load();) {

					const crno::hrc::time_point aquisition_start = crno::hrc::now();
					size_t aquisition_loop_count = 0;
					for(uint64_t filled_segments = 0; this->_state.enable_threads; aquisition_loop_count++) {	// loop until thread exit is called... (internal break allows for exit as well)

						if(filled_segments >= this->_config.enabled_segments) {	// if we have aquired sufficient samples...
							LDRP_LOG( LOG_DEBUG, "LDRP Worker [Aquisition Loop]: Aquisition quota satisfied after {} loops - exporting buffer to thread...", aquisition_loop_count )
							// attempt to find or create a thread for processing the frame

							// THREAD POOL RAAAAAHHHHH :O
							this->_finished_queue_mutex.lock();	// aquire mutex for queue
							if(this->_finished_threads.size() > 0) {
								const uint32_t filter_idx = this->_finished_threads.pop_back();	// maybe check that this is valid?
								this->_finished_queue_mutex.unlock();
								FilterContext& ctx = this->_filter_threads[filter_idx];
								std::swap(ctx.samples, frame_segments);		// figure out where we want to clear the buffer that is swapped in so we don't start with old data in the queues
								ctx.link_state.notify_all();
								break;
							} else if(this->_filter_threads.size() < this->_config.max_filter_threads) {	// start a new thread
								// create a new thread a swap in the sample
								this->_filter_threads.push_back();
								FilterContext& ctx = this->_filter_threads.back();
								ctx.index = this->_filter_threads.size() - 1;
								std::swap(ctx.samples, frame_segments);
								ctx.thread.reset( new std::thread{&LidarImpl::filterWorker, this, &ctx} );
								break;
							}
						}	// insufficient samples or no thread available... (keep updating the current frame)
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
								LDRP_LOG( LOG_DEBUG && LOG_VERBOSE, "LDRP Worker [Aquisition Loop]: Successfully parsed scan segment idx {}!", parsed_segment.segmentIndex )

								const uint64_t seg_bit = (1ULL << parsed_segment.segmentIndex);
								if(this->_config.enabled_segments & seg_bit) {	// if the segment'th bit is set
									size_t idx = ::countBitsBeforeN(this->_config.enabled_segments, parsed_segment.segmentIndex);	// get the index of the enabled bit (for our buffer)

									frame_segments[idx].emplace_front();	// create empty segment buffer
									::swapSegmentsNoIMU(frame_segments[idx].front(), parsed_segment);		// efficient buffer transfer (no deep copying!)
									if(frame_segments[idx].size() >= this->_config.buffered_scans) {
										frame_segments[idx].resize(this->_config.buffered_scans);	// cut off oldest scans beyond the buffer size (resize() removes from the back)
										filled_segments |= seg_bit;		// save this segment as finished by enabling it's bit
									}
								}
								this->_nt.last_parsed_seg_idx.Set( parsed_segment.segmentIndex );

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

	const status_t updateWorldPose(const float* xyz, const float* qxyz, const float qw) {
		if(LidarImpl::_global) {
			LidarImpl::_global->_pose_mutex.lock();
			{
				// TODO - add to timestamped queue or whatever we callin it
			}
			LidarImpl::_global->_pose_mutex.unlock();
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