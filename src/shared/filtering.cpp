#include "pcd_streaming.h"

#include <type_traits>
#include <memory>
#include <chrono>

#include <pcl/pcl_config.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "mem_utils.h"


template<typename T>
inline static T& unconst(const T& v) {
	return const_cast<T&>(v);
}


/** for reimplementation */
void pc_voxelize(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<pcl::PointXYZ>& out,
	const Eigen::Vector4f& leaf_size
) {

	static thread_local pcl::VoxelGrid<pcl::PointXYZ>
		filter{};
	static thread_local pcl::PointCloud<pcl::PointXYZ>::Ptr
		cloud{ new pcl::PointCloud<pcl::PointXYZ> },
		filtered{ new pcl::PointCloud<pcl::PointXYZ> };
	static thread_local pcl::IndicesPtr
		select{ new pcl::Indices };

	// ::pointSwap(unconst(points), *cloud);
	// ::pointSwap(out, *filtered);

	filter.setInputCloud(cloud);
	filter.setLeafSize(leaf_size);

	// if filter inst gets saved between calls, we need to figure out how to reset indices!
	if(!selection.empty()) {
		std::swap(*select, unconst(selection));
		filter.setIndices(select);
		filter.filter(*filtered);
		std::swap(*select, unconst(selection));
	} else {
		filter.filter(*filtered);
	}

	// ::pointSwap(unconst(points), *cloud);
	// ::pointSwap(out, *filtered);

}
void pc_filter_plane(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& out,
	Eigen::Vector4f& out_fitment,
	const Eigen::Vector3f& target_norm,
	float fit_dist_threshold,
	float fit_theta_threshold
) {}
void pc_filter_cartesian(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& out,
	const Eigen::Vector3f& min,
	const Eigen::Vector3f& max
) {}
void pc_filter_range(
	const std::vector<float>& ranges,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& out,
	const float max, const float min = 0.f
) {}
void pc_filter_PM(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& out_ground,
	const float base,
	const int max_window_size,
	const float cell_size,
	const float initial_distance,
	const float max_distance,
	const float slope,
	const bool exponential
) {}
void pc_remove_selection(
	std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection
) {
	// assert sizes
	size_t last = points.size() - 1;
	for(size_t i = 0; i < selection.size(); i++) {
		memcpy(&points[selection[i]], &points[last], sizeof(pcl::PointXYZ));
		last--;
	}
	points.resize(last + 1);
}
/** prereq: selection indices must be in ascending order */
void pc_negate_selection(
	const std::vector<int32_t>& base,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& negated
) {
	if(base.size() <= selection.size()) {
		return;
	}
	negated.resize(base.size() - selection.size());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for(; _base < base.size() && _negate < negated.size(); _base++) {
		if(_select < selection.size() && base[_base] == selection[_select]) {
			_select++;
		} else {
			negated[_negate] = base[_base];
			_negate++;
		}
	}
}
/** prereq: selection indices must be in ascending order */
void pc_negate_selection(
	const int32_t base_range,
	const std::vector<int32_t>& selection,
	std::vector<int32_t>& negated
) {
	if (base_range <= selection.size()) {
		return;
	}
	negated.resize(base_range - selection.size());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for (; _base < base_range && _negate < negated.size(); _base++) {
		if (_select < selection.size() && _base == selection[_select]) {
			_select++;
		} else /*if (_base < selection[_select])*/ {
			negated[_negate] = _base;
			_negate++;
		}
	}
}
void pc_generate_ranges(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<float>& out_ranges,
	const Eigen::Vector3f& origin
) {
	if (!selection.empty()) {
		out_ranges.resize(selection.size());
		for (int i = 0; i < selection.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3f*>(&points[selection[i]])).norm();
		}
	}
	else {
		out_ranges.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3f*>(&points[i])).norm();
		}
	}
}
