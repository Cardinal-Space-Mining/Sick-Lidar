/** FOR REFERENCE
 * pub/sub example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 * params example: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
*/

#include "./perception.hpp"

#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


PerceptionNode::PerceptionNode() : rclcpp::Node("perception_node") {

	// setup subs/pubs
	this->scan_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("input scan", 1,
		std::bind(&PerceptionNode::scan_cb, this, std::placeholders::_1));
	this->trfm_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>("input trfm", 1,
		std::bind(&PerceptionNode::trfm_cb, this, std::placeholders::_1));
	this->grid_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle grid", 1);
	// declare params
	// this->declare_parameter("", /* default value*/);
	// this->get_parameter("", /* param */);

	this->accumulator.reset(1.f);	// use params

}

PerceptionNode::~PerceptionNode() {}


void PerceptionNode::scan_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan) {

	static const pcl::Indices DEFAULT_NO_SELECTION = pcl::Indices{};

	// TODO: add back all these params!
	const float		// all "unitted" parameters are normalized to be in meters and radians!
		_max_scan_theta			= this->_config.fpipeline.max_scan_theta_deg * (std::numbers::pi_v<float> / 180.f),
		_min_scan_theta			= this->_config.fpipeline.min_scan_theta_deg * (std::numbers::pi_v<float> / 180.f),
		_min_scan_range			= this->_config.fpipeline.min_scan_range_cm * 1e-2f,
		_voxel_size				= this->_config.fpipeline.voxel_size_cm * 1e-2f,
		_max_pmf_range			= this->_config.fpipeline.max_pmf_range_cm * 1e-2f,
		_max_z_thresh			= this->_config.fpipeline.max_z_thresh_cm * 1e-2f,
		_min_z_thresh			= this->_config.fpipeline.min_z_thresh_cm * 1e-2f,
		_pmf_window_base		= this->_config.fpipeline.pmf_window_base,
		_pmf_max_window_size	= this->_config.fpipeline.pmf_max_window_size_cm * 1e-2f,
		_pmf_cell_size			= this->_config.fpipeline.pmf_cell_size_cm * 1e-2f,
		_pmf_init_distance		= this->_config.fpipeline.pmf_init_distance_cm * 1e-2f,
		_pmf_max_distance		= this->_config.fpipeline.pmf_max_distance_cm * 1e-2f,
		_pmf_slope				= this->_config.fpipeline.pmf_slope
	;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::PointCloud<pcl::PointXYZ>
		point_cloud,
		voxel_cloud;
	pcl::Indices
		z_high_filtered{},
		z_low_subset_filtered{},
		z_mid_filtered_obstacles{},
		pre_pmf_range_filtered{},
		pmf_filtered_ground{},
		pmf_filtered_obstacles{},
		combined_obstacles{};

	pcl::fromROSMsg(*scan, cloud);

	// transform somehow

	// NOTE: probably make the pipline a static method that gets called

	// voxelize points
	voxel_filter(
		point_cloud, DEFAULT_NO_SELECTION, voxelized_points,
		_voxel_size, _voxel_size, _voxel_size
	);

	// filter points under "high cut" thresh
	carteZ_filter(
		voxelized_points, DEFAULT_NO_SELECTION, z_high_filtered,
		-std::numeric_limits<float>::infinity(),
		_max_z_thresh
	);
	// further filter points below "low cut" thresh
	carteZ_filter(
		voxelized_points, z_high_filtered, z_low_subset_filtered,
		-std::numeric_limits<float>::infinity(),
		_min_z_thresh
	);
	// get the points inbetween high and low thresholds --> treated as wall obstacles
	pc_negate_selection(
		z_high_filtered,
		z_low_subset_filtered,
		z_mid_filtered_obstacles
	);

	// filter close enough points for PMF
	pc_filter_distance(
		voxelized_points.points,
		z_low_subset_filtered,
		pre_pmf_range_filtered,
		0.f, _max_pmf_range,
		avg_sample_origin
	);

	// apply pmf to selected points
	progressive_morph_filter(
		voxelized_points, pre_pmf_range_filtered, pmf_filtered_ground,
		_pmf_window_base,
		_pmf_max_window_size,
		_pmf_cell_size,
		_pmf_init_distance,
		_pmf_max_distance,
		_pmf_slope,
		false
	);
	// obstacles = (base - ground)
	pc_negate_selection(
		pre_pmf_range_filtered,
		pmf_filtered_ground,
		pmf_filtered_obstacles
	);


	this->accumulator.incrementRatio(	// insert PMF obstacles
		voxelized_points,
		pre_pmf_range_filtered,		// base
		pmf_filtered_obstacles		// subset
	);
	this->accumulator.incrementRatio(	// insert z-thresh obstacles
		voxelized_points,
		z_mid_filtered_obstacles,	// base
		DEFAULT_NO_SELECTION		// use all of base
	);

	// run a pass over the grid to trim ratio magnitudes

	// convert grid to occupancy map
	// publish occupancy map


}

void PerceptionNode::trfm_cb(const geometry_msgs::msg::TransformStamped::ConstSharedPtr& trfm) {

	// do something!

}