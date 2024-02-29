#include "pcd_streaming.h"

#include <memory>
#include <chrono>


/** PCDTarWriter Impl */

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
void PCDTarWriter::closeIO() {
	if (this->isOpen()) {
		this->fio.close();
	}
	this->status_bits &= ~0b1;
}
bool PCDTarWriter::isOpen() {
	return this->fio.is_open();
}
uint32_t PCDTarWriter::checkStatus() {
	return this->status_bits;
}

bool PCDTarWriter::addCloud(const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient, bool compress, const char* pcd_fname) {
	if (this->isOpen() && !(this->status_bits & 0b1)) {
		const spos_t start = this->append_pos;

		if (!this->head_buff) { this->head_buff = new pcl::io::TARHeader{}; }
		memset(this->head_buff, 0, sizeof(pcl::io::TARHeader));	// buffer where the header will be so that we can start writing the file data

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);	// write blank header
		const spos_t pcd_beg = this->fio.tellp();

		int status;
#if PCL_VERSION >= 101300
		if (!compress) {
			status = this->writer.writeBinary(this->fio, cloud, origin, orient);
		} else
#endif
		{
			status = this->writer.writeBinaryCompressed(this->fio, cloud, origin, orient);
		}
		if (status) return false;	// keep the same append position so we overwrite next time

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-truncation"	// deal with msvc?
		if (pcd_fname) {
			snprintf(this->head_buff->file_name, 100, "%s", pcd_fname);
		}
		else {
			snprintf(this->head_buff->file_name, 100, "pc_%lx.pcd", mseconds);
		}
		snprintf(this->head_buff->file_mode, 8, "0100777");
		snprintf(this->head_buff->uid, 8, "0000000");
		snprintf(this->head_buff->gid, 8, "0000000");
		snprintf(this->head_buff->file_size, 12, "%011lo", (uint64_t)flen);
		snprintf(this->head_buff->mtime, 12, "%011lo", mseconds / 1000);
		snprintf(this->head_buff->ustar, 6, "ustar");
		snprintf(this->head_buff->ustar_version, 2, "00");
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
		snprintf(this->head_buff->chksum, 7, "%06lo", xsum);
		this->head_buff->chksum[7] = ' ';
#pragma GCC diagnostic pop

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff),
			(this->head_buff->uname - this->head_buff->file_name));		// only re-write the byte range that we have modified

		return true;
	}
	return false;
}

// end PCDWriter impl
