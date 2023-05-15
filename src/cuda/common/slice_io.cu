/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/15 10:42
 * Last Modified : 2023/05/15 10:42
 *
 */

#include "cuda/base.cuh"

namespace vvc {
namespace client {
	namespace io {
		int LoadSlice(common::Slice_t& _slice, const std::string& _name) {
			std::regex name_type{"^.*\\.slice"};
			if (!std::regex_match(_name, name_type)) {
				printf("Load %s failed, wrong file format.\n", _name.c_str());
				return 1;
			}

			FILE* fp = fopen(_name.c_str(), "rb");
			if (fp == nullptr) {
				switch (errno) {
					case ENOENT: printf("Load %s failed, file not exist.\n", _name.c_str()); return 2;
					case EACCES: printf("Load %s failed, permission denied.\n", _name.c_str()); return 3;
					default: printf("Load %s failed, unexpected file error.\n", _name.c_str()); return 4;
				}
			}

			if (_slice.size || _slice.geometry_size || _slice.color_size) {
				printf("Warning, try to load %s into a slice which contains data.\n", _name.c_str());
			}

			if (fread(&_slice.type, sizeof(uint8_t), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_VALID)) {
				printf("Load %s failed, bad slice.\n", _name.c_str());
				fclose(fp);
				return 6;
			}

			if (fread(&_slice.timestamp, sizeof(int), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(&_slice.index, sizeof(int), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(&_slice.size, sizeof(size_t), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(&_slice.qp, sizeof(uint8_t), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(_slice.mv.data, sizeof(float), 16, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_PREDICT)) {
				if (fread(&_slice.geometry_size, sizeof(size_t), 1, fp) != 1) {
					printf("Load %s failed, read error.\n", _name.c_str());
					fclose(fp);
					return 5;
				}
				if (!_slice.geometry_size) {
					printf("Load %s failed, empty data.\n", _name.c_str());
					fclose(fp);
					return 7;
				}
				_slice.geometry = (uint8_t*)malloc(sizeof(uint8_t) * _slice.geometry_size);
				if (fread(_slice.geometry, sizeof(uint8_t), _slice.geometry_size, fp) != _slice.geometry_size) {
					printf("Load %s failed, empty data.\n", _name.c_str());
					fclose(fp);
					return 5;
				}
				if (common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_GEO_ZSTD)) {
					size_t   buffer_size = ZSTD_getFrameContentSize(_slice.geometry, _slice.geometry_size);
					uint8_t* temp        = (uint8_t*)malloc(sizeof(uint8_t) * buffer_size);
					size_t   result_size = ZSTD_decompress(temp, buffer_size, _slice.geometry, _slice.geometry_size);
					free(_slice.geometry);
					_slice.geometry_size = result_size;
					_slice.geometry      = temp;
				}
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_SKIP)) {
				if (fread(&_slice.color_size, sizeof(size_t), 1, fp) != 1) {
					printf("Load %s failed, read error.\n", _name.c_str());
					fclose(fp);
					return 5;
				}
				if (!_slice.color_size) {
					printf("Load %s failed, empty data.\n", _name.c_str());
					fclose(fp);
					return 7;
				}
				_slice.color = (uint8_t*)malloc(sizeof(uint8_t) * _slice.color_size);
				if (fread(_slice.color, sizeof(uint8_t), _slice.color_size, fp) != _slice.color_size) {
					printf("Load %s failed, empty data.\n", _name.c_str());
					fclose(fp);
					return 5;
				}
				if (common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_COLOR_ZSTD)) {
					size_t   buffer_size = ZSTD_getFrameContentSize(_slice.color, _slice.color_size);
					uint8_t* temp        = (uint8_t*)malloc(sizeof(uint8_t) * buffer_size);
					size_t   result_size = ZSTD_decompress(temp, buffer_size, _slice.color, _slice.color_size);
					free(_slice.color);
					_slice.color_size = result_size;
					_slice.color      = temp;
				}
			}
			fgetc(fp);
			if (!feof(fp)) {
				printf("Warning, %s seem to have extra data.\n", _name.c_str());
			}
			fclose(fp);
			return 0;
		}
	}  // namespace io
}  // namespace client
}  // namespace vvc
