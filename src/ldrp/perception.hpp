#pragma once

#include "./grid.hpp"
#include "./timestamp_sampler.hpp"

#include <memory>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

	void process(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin);
	void process_and_export(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin);


protected:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr trfm_sub;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;

	// last pointcloud and transform?
	TimestampSampler<pcl::PointCloud<pcl::PointXYZ>::Ptr, int64_t> scan_sampler{};
	QuantizedRatioGrid<uint8_t, float> accumulator{};

	struct {

		// bool
		// 	scan_matching_skip_invalid = false;
		double
			scan_matching_history_range_s = 0.25,
			map_resolution_cm		= 5.,
			max_pmf_range_cm		= 250.,
			max_z_thresh_cm			= 100.,
			min_z_thresh_cm			= 25.,
			voxel_size_cm			= 3.,
			pmf_window_base			= 2.,
			pmf_max_window_size_cm	= 48.,
			pmf_cell_size_cm		= 5.,
			pmf_init_distance_cm	= 5.,
			pmf_max_distance_cm		= 12.,
			pmf_slope				= 2.;

	} _config;


};
