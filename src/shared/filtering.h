#pragma once

#include <vector>

#include <pcl/types.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/morphological_filter.h>


/** PMF Filter Reimpl -- See <pcl/filters/progressive_morphological_filter.h> */
template<
	typename PointT = pcl::PointXYZ,
	typename IntT = pcl::index_t>
void progressive_morph_filter(
	const pcl::PointCloud<PointT>& cloud_,
	const std::vector<IntT>& selection,
	std::vector<IntT>& ground,
	const float base_,
	const int max_window_size_,
	const float cell_size_,
	const float initial_distance_,
	const float max_distance_,
	const float slope_,
	const bool exponential_
) {
	// Compute the series of window sizes and height thresholds
	std::vector<float> height_thresholds;
	std::vector<float> window_sizes;
	int iteration = 0;
	float window_size = 0.0f;
	float height_threshold = 0.0f;

	while (window_size < max_window_size_)
	{
		// Determine the initial window size.
		if (exponential_)
			window_size = cell_size_ * (2.0f * std::pow(base_, iteration) + 1.0f);
		else
			window_size = cell_size_ * (2.0f * (iteration + 1) * base_ + 1.0f);

		// Calculate the height threshold to be used in the next iteration.
		if (iteration == 0)
			height_threshold = initial_distance_;
		else
			height_threshold = slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;

		// Enforce max distance on height threshold
		if (height_threshold > max_distance_)
			height_threshold = max_distance_;

		window_sizes.push_back(window_size);
		height_thresholds.push_back(height_threshold);

		iteration++;
	}

	// Ground indices are initially limited to those points in the input cloud we
	// wish to process
	if (selection.size() > 0 && selection.size() <= cloud_.size()) {
		ground = selection;
	} else {
		ground.resize(cloud_.size());
		for (std::size_t i = 0; i < cloud_.size(); i++) {
			ground[i] = i;
		}
	}

	// Progressively filter ground returns using morphological open
	for (std::size_t i = 0; i < window_sizes.size(); ++i)
	{
		// Limit filtering to those points currently considered ground returns
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT>(cloud_, ground, *cloud);

		// Create new cloud to hold the filtered results. Apply the morphological
		// opening operation at the current window size.
		typename pcl::PointCloud<PointT>::Ptr
			cloud_O(new pcl::PointCloud<PointT>),
			cloud_C(new pcl::PointCloud<PointT>);
		pcl::applyMorphologicalOperator<PointT>(cloud, window_sizes[i], pcl::MorphologicalOperators::MORPH_OPEN, *cloud_O);
		pcl::applyMorphologicalOperator<PointT>(cloud, window_sizes[i], pcl::MorphologicalOperators::MORPH_CLOSE, *cloud_C);

		// Find indices of the points whose difference between the source and
		// filtered point clouds is less than the current height threshold.
		pcl::Indices pt_indices;
		for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
		{
			float diff_O = (*cloud)[p_idx].z - (*cloud_O)[p_idx].z;
			float diff_C = (*cloud_C)[p_idx].z - (*cloud)[p_idx].z;
			if (diff_O < height_thresholds[i] && diff_C < height_thresholds[i])
				pt_indices.push_back(ground[p_idx]);
		}

		// Ground is now limited to pt_indices
		ground.swap(pt_indices);
	}

}

/** Generate a set of ranges for each point in the provided cloud */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t,
	typename FloatT = float>
void pc_generate_ranges(
	const std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection,
	std::vector<FloatT>& out_ranges,
	const Eigen::Vector3<FloatT>& origin
) {
	if (!selection.empty()) {
		out_ranges.resize(selection.size());
		for (int i = 0; i < selection.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[selection[i]])).norm();
		}
	}
	else {
		out_ranges.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			out_ranges[i] = (origin - *reinterpret_cast<const Eigen::Vector3<FloatT>*>(&points[i])).norm();
		}
	}
}
/** Remove the points at the each index in the provided set */
template<
	typename PointT = pcl::PointXYZ,
	typename AllocT = typename pcl::PointCloud<PointT>::VectorType::allocator_type,
	typename IntT = pcl::index_t>
void pc_remove_selection(
	std::vector<PointT, AllocT>& points,
	const std::vector<IntT>& selection
) {
	// assert sizes
	size_t last = points.size() - 1;
	for(size_t i = 0; i < selection.size(); i++) {
		memcpy(&points[selection[i]], &points[last], sizeof(PointT));
		last--;
	}
	points.resize(last + 1);
}

/** Filter a set of ranges to an inclusive set of indices */
template<
	typename FloatT = float,
	typename IntT = pcl::index_t>
void pc_filter_ranges(
	const std::vector<FloatT>& ranges,
	const std::vector<IntT>& selection,
	std::vector<IntT>& filtered,
	const FloatT min, const FloatT max
) {
	filtered.clear();
	if(!selection.empty()) {
		filtered.reserve(selection.size());
		for (size_t i = 0; i < selection.size(); i++) {
			const IntT idx = selection[i];
			const FloatT r = ranges[idx];
			if (r <= max && r >= min) {
				filtered.push_back(idx);
			}

		}
	} else {
		filtered.reserve(ranges.size());
		for (size_t i = 0; i < ranges.size(); i++) {
			const FloatT r = ranges[i];
			if (r <= max && r >= min) {
				filtered.push_back(i);
			}
		}
	}
}

/** Given a base set of indices A and a subset of indices B, get (A - B).
 * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void pc_negate_selection(
	const std::vector<IntT>& base,
	const std::vector<IntT>& selection,
	std::vector<IntT>& negated
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
/** Given a base set of indices A and a subset of indices B, get (A - B).
 * prereq: selection indices must be in ascending order */
template<typename IntT = pcl::index_t>
void pc_negate_selection(
	const IntT base_range,
	const std::vector<IntT>& selection,
	std::vector<IntT>& negated
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
