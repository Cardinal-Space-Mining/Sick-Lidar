#include "pcd_streaming.h"

#include <type_traits>
#include <memory>
#include <chrono>

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


/** PCDTarWriter Impl -- (external) */

PCDTarWriter::~PCDTarWriter() {
	if (this->head_buff) delete this->head_buff;
	this->closeIO();
}
bool PCDTarWriter::setFile(const char* fname) {
	this->fio.open(fname, OPEN_MODES);
	if (!this->fio.is_open()) {
		this->status_bits |= 0b1;	// fio fail
		return false;
	}
	this->fio.seekp(0, std::ios::end);
	const spos_t end = this->fio.tellp();
	if (end < 1024) {
		this->append_pos = 0;
	}
	else {    // maybe also add a "end % 512 == 0" check
		this->append_pos = end - (spos_t)1024;
	}
	return true;
}
bool PCDTarWriter::isOpen() {
	return this->fio.is_open();
}
void PCDTarWriter::closeIO() {
	if (this->isOpen()) {
		this->fio.close();
	}
	this->status_bits &= ~0b1;
}
void PCDTarWriter::addCloud(const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient, bool compress, const char* pcd_fname) {
	if (this->isOpen() && !(this->status_bits & 0b1)) {
		const spos_t start = this->append_pos;

		if (!this->head_buff) { this->head_buff = new pcl::io::TARHeader{}; }
		memset(this->head_buff, 0, sizeof(pcl::io::TARHeader));	// buffer where the header will be so that we can start writing the file data

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);	// write blank header
		const spos_t pcd_beg = this->fio.tellp();

		int status;
		if (compress) { status = this->writer.writeBinaryCompressed(this->fio, cloud, origin, orient); }
		else { status = this->writer.writeBinary(this->fio, cloud, origin, orient); }
		if (status) return;	// keep the same append position so we overwrite next time

		const spos_t pcd_end = this->fio.tellp();
		const size_t
			flen = pcd_end - pcd_beg,
			padding = (512 - flen % 512);

		this->fio.write(reinterpret_cast<char*>(this->head_buff), padding);	// pad to 512 byte chunk
		this->append_pos = this->fio.tellp();	// if we add another file, it should start here and overwrite the end padding

		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);		// append 2 zeroed chunks
		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);

		uint64_t mseconds = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		).count();

		if (pcd_fname) {
			snprintf(this->head_buff->file_name, 100, pcd_fname);
		}
		else {
			snprintf(this->head_buff->file_name, 100, "pc_%llx.pcd", mseconds);
		}
		snprintf(this->head_buff->file_mode, 8, "0100777");
		snprintf(this->head_buff->uid, 8, "0000000");
		snprintf(this->head_buff->gid, 8, "0000000");
		snprintf(this->head_buff->file_size, 12, "%011llo", (uint64_t)flen);
		snprintf(this->head_buff->mtime, 12, "%011llo", mseconds / 1000);
		sprintf_s(this->head_buff->ustar, "ustar");
		sprintf_s(this->head_buff->ustar_version, "00");
		this->head_buff->file_type[0] = '0';

		uint64_t xsum = 0;
		for (char* p = reinterpret_cast<char*>(this->head_buff); p < this->head_buff->chksum; p++)
		{
			xsum += *p & 0xff;
		}
		xsum += (' ' * 8) + this->head_buff->file_type[0];
		for (char* p = this->head_buff->ustar; p < this->head_buff->uname; p++)		// the only remaining part that we wrote to was ustar and version
		{
			xsum += *p & 0xff;
		}
		snprintf(this->head_buff->chksum, 7, "%06llo", xsum);
		this->head_buff->chksum[7] = ' ';

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff),
			(this->head_buff->uname - this->head_buff->file_name));		// only re-write the byte range that we have modified
	}
}

// end PCDWriter impl






/** for reimplementation */
void pc_voxelize(
	const std::vector<pcl::PointXYZ>& points,
	const std::vector<int32_t>& selection,
	std::vector<pcl::PointXYZ>& out,
	const Eigen::Vector4f& leaf_size
) {}
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
			// pointXYZ to vector3f ?
		}
	}
	else {
		out_ranges.resize(points.size());
		for (int i = 0; i < points.size(); i++) {
			
		}
	}
}
