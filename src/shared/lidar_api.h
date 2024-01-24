#pragma once

#include <cstdint>
#include <memory>

// create inst
//		V
// multiscan
//		V
// filter caller
//		V
// store result
//		V
// public api
//		V
// get result, logging, network telemetry, configs


// "LiDaR Proccessing"
namespace ldrp {

	using ApiRef = std::shared_ptr<void>;

	struct PointBuff {

	};
	struct MapBuff {

	};


	/** Create a new lidar processing instance */
	const ApiRef create();	// add params




	/** Shutdown and deallocate the given lidar processing instance */
	// void shutdown(ApiRef);


};