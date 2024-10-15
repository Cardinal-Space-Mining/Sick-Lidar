#pragma once

/** Default macro definitions -- configurable via CMake */
#ifndef LDRP_ENABLE_LOGGING		// enable/disable all logging output
#define LDRP_ENABLE_LOGGING		true
#endif
#ifndef LDRP_DEBUG_LOGGING		// enable/disable debug level logging
#define LDRP_DEBUG_LOGGING		false
#endif
#ifndef LDRP_SAFETY_CHECKS		// enable/disable additional safety checking (ex. bound checking)
#define LDRP_SAFETY_CHECKS		true
#endif
#ifndef LDRP_USE_UESIM			// whether or not WPILib is being compiled into the library - for build system internal use only
#define LDRP_USE_UESIM			false
#endif
#ifndef LDRP_USE_UESIM			// enable/disable using simulation as the source of points, and set which simulation source to use (1 = internal, 2 = UE simulator)
#define LDRP_USE_UESIM			false
#endif
#ifndef LDRP_ENABLE_TUNING		// enable/disable live tuning using networktables (requires WPILib)
#define LDRP_ENABLE_TUNING		false
#endif
#ifndef LDRP_ENABLE_PROFILING	// enable/disable live and logged filter pipeline profiling over networktables
#define LDRP_ENABLE_PROFILING		false
#endif

#if !LDRP_SAFETY_CHECKS
#define GRID_IMPL_SKIP_BOUND_CHECKING		// disables array bound checking within QRG
#endif
#include "./grid.hpp"
#include "./timestamp_sampler.hpp"

#include <string>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();
    ~PerceptionNode();


protected:
    void scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);

    void process(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin);
    void process_and_export(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Vector3f& origin);


protected:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub;

    QuantizedRatioGrid<uint8_t, float> accumulator{};

    struct {

        double
            max_tf_wait_time_s = 0.25,
            map_resolution_cm		= 5.,
            voxel_size_cm			= 3.,
            max_z_thresh_cm			= 100.,
            min_z_thresh_cm			= 25.,
            pmf_max_range_cm		= 250.,
            pmf_window_base			= 2.,
            pmf_max_window_size_cm	= 48.,
            pmf_cell_size_cm		= 5.,
            pmf_init_distance_cm	= 5.,
            pmf_max_distance_cm		= 12.,
            pmf_slope				= 2.;

        std::string
            map_frame = "map",
            lidar_origin_frame = "lidar_link";

    } _config;

#if LDRP_ENABLE_PROFILING
    struct {
        
    } _profile;
#endif


};
