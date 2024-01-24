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


// "LiDaR Proccessing"
namespace ldrp {

	using ApiRef = std::shared_ptr<void>;

	struct PointBuff {

	};
	struct MapBuff {

	};


	/** Create a new lidar processing instance */
	__API const ApiRef create();	// add params




	/** Shutdown and deallocate the given lidar processing instance */
	// void shutdown(ApiRef);


};