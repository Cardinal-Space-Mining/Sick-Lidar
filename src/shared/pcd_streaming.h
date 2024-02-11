#pragma once

#include <fstream>

#include <Eigen/Core>
#include <pcl/io/tar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>


// class PCDTarReader {
// public:
// 	PCDTarReader();
// 	~PCDTarReader();

// public:
// 	bool setFile(const char* fname);
// 	bool isOpen();
// 	void closeIO();
	
// protected:
	


// };

class PCDTarWriter {
public:
	PCDTarWriter() {}
	~PCDTarWriter();

public:
	bool setFile(const char* fname);
	void closeIO();
	bool isOpen();
	uint32_t checkStatus();

	bool addCloud(
		const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient,
		bool compress = true, const char* pcd_fname = nullptr);

	template<typename PointT>
	inline bool addCloud(
		const pcl::PointCloud<PointT>& cloud,
		bool compress = true, const char* pcd_fname = nullptr
	) {
		pcl::toPCLPointCloud2<PointT>(cloud, this->temp_cloud);
		return this->addCloud(
			this->temp_cloud,
			cloud.sensor_origin_,
			cloud.sensor_orientation_,
			compress, pcd_fname
		);
	}

protected:
	using spos_t = std::iostream::pos_type;
	constexpr static std::ios::openmode
		OPEN_MODES = (std::ios::binary | std::ios::in | std::ios::out);

	pcl::io::TARHeader* head_buff{ nullptr };
	pcl::PCLPointCloud2 temp_cloud{};

	std::ofstream fio{};
	pcl::PCDWriter writer{};
	spos_t append_pos{ 0 };
	uint32_t status_bits{ 0 };


};
