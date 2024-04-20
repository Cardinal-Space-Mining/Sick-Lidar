#pragma once

#include "./filtering.hpp"
#include "./grid.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


class PerceptionNode : public rclcpp::Node {
public:
	PerceptionNode();
	~PerceptionNode();


protected:
	void scan_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
	void trfm_cb(const geometry_msgs::msg::TransformStamped::ConstSharedPtr& trfm);

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
