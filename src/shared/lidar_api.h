#pragma once

#include <cstdint>
#include <memory>

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

	struct __API PointBuff {

	};
	struct __API MapBuff {

	};


	/** Get the pcl version string of the currently linked library */
	__API char const* pclVer();

	/** Create a new lidar processing instance */
	__API const LidarApi create();	// add params
	/** Intialize the lidar connection via config file */
	__API void lidarInit(LidarApi l, const char* file);

	/** Get the status of the last call made to the internal sick_scan interface */
	__API int32_t internalStatus(LidarApi l);



	/** Shutdown and deallocate the given lidar processing instance */
	// void shutdown(ApiRef);


};