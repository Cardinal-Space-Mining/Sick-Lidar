#include "lidar_api.h"

#include <type_traits>
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <utility>
#include <deque>
#include <mutex>
#include <vector>

#include <pcl/pcl_config.h>
// #include <sick_scan_xd_api/sick_scan_api.h>
#include "sick_scansegment_xd/config.h"
#include "sick_scansegment_xd/udp_receiver.h"
#include "sick_scansegment_xd/msgpack_converter.h"
#include "sick_scansegment_xd/msgpack_exporter.h"
#include "sick_scansegment_xd/scansegment_parser_output.h"
#include "sick_scan/sick_cloud_transform.h"

//#define LOG_DEBUG  false


namespace std {
	namespace chrono {
		using hrc = high_resolution_clock;
	}
}
namespace crno = std::chrono;


template<typename V, typename C = crno::high_resolution_clock>
class TimestampedQueue {
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


namespace ldrp {

#ifndef LOG_DEBUG
#define LOG_DEBUG	true
#endif
#ifndef _LDRP_DISABLE_LOG
#define LDRP_LOG_GLOBAL(__inst, __condition, __output)		if(__condition) { __inst->logs() << __output; }
#define LOG_LEVEL_GLOBAL(__inst, __min_lvl)					(__inst->_log_level >= __min_lvl)
#define LDRP_LOG(__condition, __output)		LDRP_LOG_GLOBAL(this, __condition, __output)
#define LOG_LEVEL(__min_lvl)				LOG_LEVEL_GLOBAL(this, __min_lvl)
#define LOG_ALWAYS				true
#define LOG_STANDARD			LOG_LEVEL(1)
#define LOG_VERBOSE				LOG_LEVEL(2)
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

	// static const status_t cvtStatus(const int32_t sick_ec) {
	// 	switch(sick_ec) {
	// 		case SICK_SCAN_API_SUCCESS:
	// 			return STATUS_SUCCESS;
	// 		case SICK_SCAN_API_NOT_LOADED:
	// 		case SICK_SCAN_API_NOT_INITIALIZED:
	// 		case SICK_SCAN_API_NOT_IMPLEMENTED:
	// 			return STATUS_PREREQ_UNINITIALIZED | STATUS_EXTERNAL_ERROR;
	// 		case SICK_SCAN_API_ERROR:
	// 		case SICK_SCAN_API_TIMEOUT:
	// 		default:
	// 			return STATUS_EXTERNAL_ERROR;
	// 	}
	// }



	template<typename PointT>
	struct ScanSlice {
		crno::hrc::time_point _tstamp{};
		std::vector<PointT> _points{};
		uint32_t
			_width,
			_height;
		bool _dense;

		// template<typename _PointT>
		// static bool fromSickScanMsg(ScanSlice<_PointT>* buff, const SickScanPointCloudMsg* msg) {
		// 	if(!buff || !msg) return false;
		// 	if(sizeof(_PointT) != msg->point_step) return false;
		// 	buff->_tstamp = crno::hrc::time_point{
		// 		crno::seconds{ msg->header->timestamp_sec } +
		// 		crno::nanoseconds{ msg->header->timestamp_nsec }
		// 	};
		// 	buff->_points.resize(msg->width * msg->height);
		// 	memcpy(buff->_points.data(), msg->data->buffer,
		// 		msg->width * msg->height * msg->point_step);
		// 	buff->_dense = msg->is_dense;
		// 	buff->_width = msg->width;
		// 	buff->_height = msg->height;
		// }
	};


	/** Interfacing and Filtering Implementation */
	struct LidarImpl {
	public:
		LidarImpl() {
			LDRP_LOG( LOG_ALWAYS, "LDRP global instance initialized." << std::endl )
		}
		~LidarImpl() {
			this->_enable_thread.store(false);
			if(this->_thread->joinable()) this->_thread->join();

			// this->sickDeinit();		// make sure that this is the last call since it tends to stall

			LDRP_LOG( LOG_ALWAYS, "LDRP global instance destroyed." << std::endl )
		}

		LidarImpl(const LidarImpl&) = delete;
		LidarImpl(LidarImpl&&) = delete;


		inline std::ostream& logs() {
#ifdef LDRP_SAFETY_CHECKS
			return ((this->_log_output == nullptr) ? std::cout : *this->_log_output);
#else
			return (*this->_log_output);
#endif
		}


		/** SickScanApiCreate() --> generate handle + error handling/logging */
		// const status_t sickInit(int ss_argc = 0, char** ss_argv = nullptr) {
		// 	this->sickDeinit();
		// 	this->_handle = SickScanApiCreate(ss_argc, ss_argv);
		// 	if(this->_handle) {
		// 		LDRP_LOG( LOG_STANDARD, "Sick Scan API created successfully: " << this->_handle << std::endl )
		// 		return STATUS_SUCCESS;
		// 	}
		// 	LDRP_LOG( LOG_STANDARD, "Sick Scan API creation failed: " << this->_handle << std::endl )
		// 	return STATUS_EXTERNAL_ERROR | STATUS_FAIL;
		// }
		/** SickScanApi- Close & Release + nullify handle + error handling/logging */
		// const status_t sickDeinit() {
		// 	if(this->_handle) {
		// 		status_t s = STATUS_SUCCESS;
		// 		s |= this->lidarClose() & ~(STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL);		// mask uninited fail since this the meaning isn't relevant to the outer function
		// 		s |= cvtStatus( this->_sick_status = SickScanApiClose(this->_handle) );
		// 		LDRP_LOG( LOG_STANDARD && this->_sick_status,
		// 			"SickScanApiClose error: " << this->_sick_status << std::endl )
		// 		s |= cvtStatus( this->_sick_status = SickScanApiRelease(this->_handle) );
		// 		LDRP_LOG( LOG_STANDARD && this->_sick_status,
		// 			"SickScanApiRelease error: " << this->_sick_status << std::endl )
		// 		this->_handle = nullptr;
		// 		return s;
		// 	}
		// 	return STATUS_PREREQ_UNINITIALIZED;
		// }

		/** Init by launch file, register point cloud callback + error handling/logging */
		// const status_t lidarInit(const char* config_file) {
		// 	if(this->_handle) {
		// 		status_t s = STATUS_SUCCESS;
		// 		s |= cvtStatus( this->_sick_status = SickScanApiInitByLaunchfile(this->_handle, config_file) );
		// 		if(!s) {
		// 			s |= cvtStatus( this->_sick_status = SickScanApiRegisterCartesianPointCloudMsg(this->_handle, &cartesianPointCloudCallbackWrapper) );
		// 			LDRP_LOG( LOG_STANDARD && this->_sick_status, "SickScanApiRegisterCartesianPointCloudMsg error: " << this->_sick_status << std::endl)
		// 		} else {
		// 			LDRP_LOG( LOG_STANDARD, "SickScanApiInitByLaunchfile error: " << this->_sick_status << " with config file: " << config_file << std::endl)
		// 		}
		// 		return s;
		// 	}
		// 	return STATUS_PREREQ_UNINITIALIZED | STATUS_FAIL;
		// }
		/** Deregister point cloud callback + error handling/logging */
		// const status_t lidarClose() {
		// 	if(this->_handle) {
		// 		if( this->_sick_status = SickScanApiDeregisterCartesianPointCloudMsg(this->_handle, &cartesianPointCloudCallbackWrapper) ) {
		// 			LDRP_LOG( LOG_STANDARD, "SickScanApiDeregisterCartesianPointCloudMsg error: " << this->_sick_status << std::endl )
		// 			return cvtStatus(this->_sick_status) | STATUS_FAIL;
		// 		}
		// 		return STATUS_SUCCESS;
		// 	}
		// 	return STATUS_PREREQ_UNINITIALIZED;
		// }


	protected:
		// SickScanApiHandle _handle{ nullptr };
		// int32_t _sick_status{ 0 };

	public:
		std::ostream* _log_output{ &std::cout };
		int32_t _log_level{ 1 };

		/** PARAMS, STATES
		 * - log level
		 * - loop frequency (bounds)
		 * - use full OR partial scan data
		 * - thread enabled
		 * - filtering source event -- on pose update, clocked internal, external, on map access (bad idea)
		 * - accumulator and map resolution -- need to deal with updates while running
		 * - filter params (see header)
		*/

		struct {
			std::atomic<crno::hrc::duration> min_loop_duration{ crno::milliseconds(50) };
			std::atomic<bool> use_full_scans{ false };

			PipelineConfig pipeline_config{};
		} _config;

		std::unique_ptr<std::thread> _thread;
		std::atomic<bool> _enable_thread{false};
		// std::deque<ScanSlice<pcl::PointXYZ> > _points_queue;
		std::mutex
			// _points_mutex{},
			_pose_mutex{};
		struct {
			
		} _processing;


	public:
		inline static std::unique_ptr<LidarImpl> _global{ nullptr };

// 		static void cartesianPointCloudCallbackWrapper(SickScanApiHandle handle, const SickScanPointCloudMsg* msg) {
// 			// NOTE: SickScanPointCloudMsg buffers are freed immediately after all callbacks finish --> will need to copy data if we want to keep it!
// 			// filter by scan segment (only accept full frames or only accept partial segments)
// #ifdef LDRP_SAFETY_CHECKS
// 			if(!LidarImpl::_global || handle != LidarImpl::_global->_handle) return;		// add macro for disabling extra safety checks
// #endif
// 			LidarImpl::_global->_points_mutex.lock();
// 			{
// 				// append points to queue
// 			}
// 			LidarImpl::_global->_points_mutex.unlock();
// 		}

		void lidarWorker() {

			// (the next 30 lines or so are converted from scansegment_threads.cpp: sick_scansegment_xd::MsgPackThreads::runThreadCb())
			// init udp receiver
			sick_scansegment_xd::UdpReceiver* udp_receiver = nullptr;
			for(;!udp_receiver && _enable_thread.load();) {
				udp_receiver = new sick_scansegment_xd::UdpReceiver{};
				if(udp_receiver->Init(	// use actual configuration parameter!
					"10.11.11.3",	// udp receiver
					2115	// udp port
					// 20,	// udp fifo length
					// false,	// verbose
					// false,	// file export
					// 1,	// scandata format (1 for msgpack)
					// 0,	// fifo for payload data (for sharing a fifo)
				)) {
					LDRP_LOG( LOG_STANDARD, "LDRP Worker: UdpReceiver successfully connected to host " << "10.11.11.3" << " on port " << 2115 << "!" << std::endl )
				} else {
					LDRP_LOG( LOG_STANDARD, "LDRP Worker: UdpReceiver failed to connect to host " << "10.11.11.3" << " on port " << 2115 << ". Retrying in 1 second..." << std::endl )
					delete udp_receiver;
					udp_receiver = nullptr;
					std::this_thread::sleep_for(crno::seconds{1});	// keep trying until successful
				}
			}
			// init msgpack converter
			sick_scansegment_xd::MsgPackConverter msgpack_converter{
				sick_scansegment_xd::ScanSegmentParserConfig{},	// config (useless)
				sick_scan_xd::SickCloudTransform{},				// cloud transform (probably not use since the internal calculations are not optimized)
				udp_receiver->Fifo(),	// udp fifo
				1,	// scandata format (1=msgpack)
				20,	// fifo length (PARAM!)
				false	// verbose output
			};
			// launch runners
			if(msgpack_converter.Start() && udp_receiver->Start()) {
				LDRP_LOG( LOG_STANDARD, "LDRP Worker: UdpReceiver and MsgPackConverter threads successfully launched!" << std::endl )
			} else {
				LDRP_LOG( LOG_STANDARD, "LDRP Worker: UdpReceiver and/or MsgPackConverter threads failed to launch." << std::endl )
			}
			// SOPAS services, msgpack validator (not necessary)

			sick_scansegment_xd::ScanSegmentParserOutput scan_segment{};
			fifo_timestamp scan_timestamp{};
			size_t scan_counter{0};
			for(;_enable_thread.load();) {
				// crno::hrc::time_point n = crno::hrc::now();

				for(int i = 0; i < 12; i++) {
					if(msgpack_converter.Fifo()->Pop(scan_segment, scan_timestamp, scan_counter)) {
						LDRP_LOG( LOG_DEBUG, "Fifo loop: popped scan segment with id " << scan_segment.segmentIndex << std::endl )
					}
				}

				// export from fifo!
				//LDRP_LOG( LOG_DEBUG, "lidar processing internal worker called!" << std::endl)
				// 1. update accumulated point globule
				// 2. run filtering on points
				// 3. update accumulator
				// 4. [when configured] update map

				// std::this_thread::sleep_until(n + this->_config.min_loop_duration.load());
			}

			// close runners and deallocate
			msgpack_converter.Fifo()->Shutdown();
			udp_receiver->Fifo()->Shutdown();
			msgpack_converter.Close();
			udp_receiver->Close();
			delete udp_receiver;

		}
		// void lidarThreadWrapper() {

		// 	for(;this->_enable_thread.load();) {
		// 		crno::hrc::time_point n = crno::hrc::now();

		// 		this->lidarWorker();

		// 		std::this_thread::sleep_until(n + this->_config.min_loop_duration.load());
		// 	}
		// 	LDRP_LOG( LOG_DEBUG, "lidar processing thread finished and exitting." << std::endl)

		// }


	};




	/** Static API */

	char const* pclVer() {
		return PCL_VERSION_PRETTY;
	}
	const PipelineConfig& getDefaultPipelineConfig() {
		return PipelineConfig{};
	}

	const status_t apiInit() {
		if(!LidarImpl::_global) {
			LidarImpl::_global = std::make_unique<LidarImpl>();
			return STATUS_SUCCESS;
		}
		return STATUS_ALREADY_SATISFIED;
		// return LidarImpl::_global->sickInit(ss_argc, ss_argv);
	}
	// const status_t apiClose() {
	// 	if(LidarImpl::_global) {
	// 		return LidarImpl::_global->sickDeinit();
	// 	}
	// 	return STATUS_PREREQ_UNINITIALIZED;
	// }
	const status_t apiDestroy() {
		if(LidarImpl::_global) {
			// const status_t s = LidarImpl::_global->sickDeinit();
			delete LidarImpl::_global.release();
			// return s;
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED | STATUS_ALREADY_SATISFIED;
	}
	// const status_t lidarInit(const char* config_file) {
	// 	if(LidarImpl::_global) {
	// 		return LidarImpl::_global->lidarInit(config_file);
	// 	}
	// 	return STATUS_PREREQ_UNINITIALIZED;
	// }
	const status_t lidarInit() {
		if(LidarImpl::_global) {
			if(!LidarImpl::_global->_thread || !LidarImpl::_global->_thread->joinable()) {
				LidarImpl::_global->_enable_thread.store(true);
				LidarImpl::_global->_thread.reset(
					new std::thread{ &LidarImpl::lidarWorker, LidarImpl::_global.get() }
				);
				return STATUS_SUCCESS;
			}
			return STATUS_ALREADY_SATISFIED;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	// const status_t lidarClose() {
	// 	if(LidarImpl::_global) {
	// 		return LidarImpl::_global->lidarClose();
	// 	}
	// 	return STATUS_PREREQ_UNINITIALIZED;
	// }
	const status_t lidarShutdown() {
		if(LidarImpl::_global) {
			status_t s = STATUS_ALREADY_SATISFIED;
			LidarImpl::_global->_enable_thread.store(false);
			if(LidarImpl::_global->_thread && LidarImpl::_global->_thread->joinable()) {
				LidarImpl::_global->_thread->join();
				s = STATUS_SUCCESS;
			}
			LidarImpl::_global->_thread.reset(nullptr);		// don't need to do this but probably good to explicitly signify the thread isn't valid
			return s;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t lidarSetState(const bool enabled) {
		return enabled ? lidarInit() : lidarShutdown();
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

	// const status_t enablePipeline(const bool enable) {
	// 	if(LidarImpl::_global) {
	// 		// check for sick api inited? (not super necessary?)
	// 		LidarImpl::_global->_enable_thread.store(enable);
	// 		const bool joinable = LidarImpl::_global->_thread.joinable();
	// 		if(enable) {
	// 			if(!joinable) {
	// 				LidarImpl::_global->_thread = std::thread(&LidarImpl::lidarThreadWrapper, LidarImpl::_global.get());
	// 				return STATUS_SUCCESS;
	// 			}
	// 		} else if(joinable) {
	// 			LidarImpl::_global->_thread.join();
	// 			return STATUS_SUCCESS;
	// 		}
	// 		return STATUS_ALREADY_SATISFIED;
	// 	}
	// 	return STATUS_PREREQ_UNINITIALIZED;
	// }
	const status_t setMaxFrequency(const size_t f_hz) {
		if(LidarImpl::_global) {
			LidarImpl::_global->_config.min_loop_duration.store( crno::nanoseconds((int64_t)(1e9 / f_hz)) );	// be as precise as realisticly possible
			return STATUS_SUCCESS;
		}
		return STATUS_PREREQ_UNINITIALIZED;
	}
	const status_t applyPipelineConfig(const PipelineConfig& params) {
		if(LidarImpl::_global) {
			// TODO
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