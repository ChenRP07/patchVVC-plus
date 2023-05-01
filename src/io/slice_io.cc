/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/01 16:33
 * Last Modified : 2023/05/01 16:33
 *
 */

#include "io/slice_io.h"

namespace vvc {
namespace io {

	void SaveSlice(const common::Slice& _slice, const std::string& _name) {
		try {
			std::regex name_type{"^.*\\.slice"};

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

			if (fwrite(&_slice.type, sizeof(uint8_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&_slice.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&_slice.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&_slice.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (_slice.type == common::PVVC_SLICE_TYPE_INTRA) {
				if (!_slice.geometry) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				size_t g_size = _slice.geometry->size();
				if (!g_size) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				if (fwrite(&g_size, sizeof(size_t), 1, fp) != 1) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
				if (fwrite(_slice.geometry->data(), sizeof(uint8_t), _slice.geometry->size(), fp) != _slice.geometry->size()) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}

			if (_slice.type == common::PVVC_SLICE_TYPE_INTRA || _slice.type == common::PVVC_SLICE_TYPE_INTER) {
				if (!_slice.color) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				size_t c_size = _slice.color->size();
				if (!c_size) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				if (fwrite(&c_size, sizeof(size_t), 1, fp) != 1) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
				if (fwrite(_slice.color->data(), sizeof(uint8_t), _slice.color->size(), fp) != _slice.color->size()) {
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

	void LoadSlice(common::Slice& _slice, const std::string& _name) {
		try {
			std::regex name_type{"^.*\\.slice"};

			if (!std::regex_match(_name, name_type)) {
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

			_slice.clear();

			if (fread(&_slice.type, sizeof(uint8_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (_slice.type == common::PVVC_SLICE_TYPE_INTRA) {
				size_t g_size = 0;
				if (fread(&g_size, sizeof(size_t), 1, fp) != 1) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
				if (!g_size) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				_slice.geometry = std::make_shared<std::vector<uint8_t>>(g_size);
				if (fread(_slice.geometry->data(), sizeof(uint8_t), g_size, fp) != g_size) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
			}

			if (_slice.type == common::PVVC_SLICE_TYPE_INTRA || _slice.type == common::PVVC_SLICE_TYPE_INTER) {
				size_t c_size = 0;
				if (fread(&c_size, sizeof(size_t), 1, fp) != 1) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
				if (!c_size) {
					throw __EXCEPT__(EMPTY_RESULT);
				}
				_slice.color = std::make_shared<std::vector<uint8_t>>(c_size);
				if (fread(_slice.color->data(), sizeof(uint8_t), c_size, fp) != c_size) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
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
}  // namespace io
}  // namespace vvc
