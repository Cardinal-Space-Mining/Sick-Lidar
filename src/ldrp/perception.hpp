#pragma once

#include "filtering.hpp"
#include "grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"


class PerceptionNode : public rclcpp::Node {
public:
	PerceptionNode() {
		// subscribe to raw cloud (sick)
		// ...and transform (dlio)

		// publish topic for map
		this->accumulator.reset();	// << pass in resolution param
	}


protected:
	void process();

protected:
	// last pointcloud and transform?
	QuantizedRatioGrid<ObstacleGrid::Quant_T, float> accumulator{};


};
