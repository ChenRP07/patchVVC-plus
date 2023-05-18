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

			if (fread(_slice.mv.data, sizeof(float), 16, fp) != 16) {
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

		int LoadFrame(common::Frame_t& _frame, const std::string& _name) {
			std::regex name_type{"^.*\\.frame"};
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

			if (_frame.slice_cnt) {
				printf("Warning, try to load %s into a frame which contains data.\n", _name.c_str());
			}
			_frame.Reset();

			if (fread(&_frame.timestamp, sizeof(int), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(&_frame.slice_cnt, sizeof(uint32_t), 1, fp) != 1) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			_frame.index         = (int*)malloc(sizeof(int) * _frame.slice_cnt);
			_frame.type          = (uint8_t*)malloc(sizeof(uint8_t) * _frame.slice_cnt);
			_frame.size          = (uint32_t*)malloc(sizeof(uint32_t) * _frame.slice_cnt);
			_frame.qp            = (uint8_t*)malloc(sizeof(uint8_t) * _frame.slice_cnt);
			_frame.mv            = (float**)malloc(sizeof(float*) * _frame.slice_cnt);
			_frame.geometry_size = (uint32_t*)malloc(sizeof(uint32_t) * _frame.slice_cnt);
			_frame.color_size    = (uint32_t*)malloc(sizeof(uint32_t) * _frame.slice_cnt);
			_frame.geometry      = (uint8_t**)malloc(sizeof(uint8_t*) * _frame.slice_cnt);
			_frame.color         = (uint8_t**)malloc(sizeof(uint8_t*) * _frame.slice_cnt);

			if (fread(_frame.index, sizeof(int), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(_frame.type, sizeof(uint8_t), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(_frame.size, sizeof(uint32_t), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			if (fread(_frame.qp, sizeof(uint8_t), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			for (int i = 0; i < _frame.slice_cnt; ++i) {
				_frame.mv[i] = (float*)malloc(sizeof(float) * 16);
				if (fread(_frame.mv[i], sizeof(float), 16, fp) != 16) {
					printf("Load %s failed, read error.\n", _name.c_str());
					fclose(fp);
					return 5;
				}
			}

			if (fread(_frame.geometry_size, sizeof(uint32_t), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			for (int i = 0; i < _frame.slice_cnt; ++i) {
				if (_frame.geometry_size[i]) {
					_frame.geometry[i] = (uint8_t*)malloc(sizeof(uint8_t) * _frame.geometry_size[i]);
					if (fread(_frame.geometry[i], sizeof(uint8_t), _frame.geometry_size[i], fp) != _frame.geometry_size[i]) {
						printf("Load %s failed, read error.\n", _name.c_str());
						fclose(fp);
						return 5;
					}
					if (common::CheckSliceType(_frame.type[i], common::PVVC_SLICE_TYPE_GEO_ZSTD)) {
						size_t   buffer_size = ZSTD_getFrameContentSize(_frame.geometry[i], _frame.geometry_size[i]);
						uint8_t* temp        = (uint8_t*)malloc(sizeof(uint8_t) * buffer_size);
						size_t   result_size = ZSTD_decompress(temp, buffer_size, _frame.geometry[i], _frame.geometry_size[i]);
						free(_frame.geometry[i]);
						_frame.geometry[i]      = temp;
						_frame.geometry_size[i] = result_size;
					}
				}
			}

			if (fread(_frame.color_size, sizeof(uint32_t), _frame.slice_cnt, fp) != _frame.slice_cnt) {
				printf("Load %s failed, read error.\n", _name.c_str());
				fclose(fp);
				return 5;
			}

			for (int i = 0; i < _frame.slice_cnt; ++i) {
				if (_frame.color_size[i]) {
					_frame.color[i] = (uint8_t*)malloc(sizeof(uint8_t) * _frame.color_size[i]);
					if (fread(_frame.color[i], sizeof(uint8_t), _frame.color_size[i], fp) != _frame.color_size[i]) {
						printf("Load %s failed, read error.\n", _name.c_str());
						fclose(fp);
						return 5;
					}
					if (common::CheckSliceType(_frame.type[i], common::PVVC_SLICE_TYPE_COLOR_ZSTD)) {
						size_t   buffer_size = ZSTD_getFrameContentSize(_frame.color[i], _frame.color_size[i]);
						uint8_t* temp        = (uint8_t*)malloc(sizeof(uint8_t) * buffer_size);
						size_t   result_size = ZSTD_decompress(temp, buffer_size, _frame.color[i], _frame.color_size[i]);
						free(_frame.color[i]);
						_frame.color[i]      = temp;
						_frame.color_size[i] = result_size;
					}
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
