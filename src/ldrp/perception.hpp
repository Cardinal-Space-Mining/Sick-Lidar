#pragma once

#include "./filtering.hpp"
#include "./grid.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>


class PerceptionNode : public rclcpp::Node {
public:
	PerceptionNode();
	~PerceptionNode();


protected:
	void scan_cb(const sensor_msgs::msg::PointCloud2::SharedPtr scan);
	void trfm_cb(const geometry_msgs::msg::TransformStamped::SharedPtr trfm);

	void process();


protected:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr trfm_sub;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;

	// last pointcloud and transform?
	QuantizedRatioGrid<uint8_t, float> accumulator{};

	struct {
		// defaults -- copy from original
	} _config;


};
