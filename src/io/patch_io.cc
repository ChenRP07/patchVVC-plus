/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/01 14:09
 * Last Modified : 2023/05/01 14:09
 *
 */

#include "io/patch_io.h"

namespace vvc {
namespace io {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

	void SavePatch(const common::Patch& _patch, const std::string& _name) {
		try {
			std::regex name_type{"^.*\\.patch$"};

			if (!std::regex_match(_name, name_type)) {
				throw __EXCEPT__(WRONG_FILE_FORMAT);
			}

			FILE* fp = fopen(_name.c_str(), "wb");

			if (fp == nullptr) {
				switch (errno) {
					case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
					case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
					default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
				}
			}

			if (fwrite(&_patch.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&_patch.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&_patch.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			size_t p_size = _patch.cloud->size();

			if (fwrite(&p_size, sizeof(size_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			for (auto i : *_patch.cloud) {
				float   xyz[3] = {i.x, i.y, i.z};
				uint8_t rgb[3] = {i.r, i.g, i.b};
				if (fwrite(xyz, sizeof(float), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
				if (fwrite(rgb, sizeof(uint8_t), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}
			fclose(fp);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void LoadPatch(common::Patch& _patch, const std::string& _name) {
		try {
			std::regex name_type{"^.*\\.patch$"};

			if (!std::regex_search(_name, name_type)) {
				throw __EXCEPT__(WRONG_FILE_FORMAT);
			}

			FILE* fp = fopen(_name.c_str(), "rb");

			if (fp == nullptr) {
				switch (errno) {
					case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
					case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
					default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
				}
			}

			if (fread(&_patch.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_patch.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_patch.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			size_t p_size = 0;
			if (fread(&p_size, sizeof(size_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}
			if (!p_size) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}
			_patch.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			for (int i = 0; i < p_size; ++i) {
				float   xyz[3];
				uint8_t rgb[3];
				if (fread(xyz, sizeof(float), 3, fp) != 3) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
				if (fread(rgb, sizeof(uint8_t), 3, fp) != 3) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
				pcl::PointXYZRGB point;
				point.x = xyz[0], point.y = xyz[1], point.z = xyz[2];
				point.r = rgb[0], point.g = rgb[1], point.b = rgb[2];
				_patch.cloud->emplace_back(point);
			}
			if (!feof(fp)) {
				common::PVVCLog_Mutex.lock();
				std::cout << __B_YELLOWT__([Warning]) << ' ' << _name << " seems to have extra content.\n";
				common::PVVCLog_Mutex.unlock();
			}
			fclose(fp);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

#pragma GCC diagnostic pop
}  // namespace io
}  // namespace vvc
