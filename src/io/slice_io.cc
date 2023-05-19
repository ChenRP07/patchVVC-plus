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

	int SaveSlice(const common::Slice& _slice, const std::string& _name) {
		try {
			int stream_size = 0;

			std::regex name_type{"^.*\\.slice"};

			if (!std::regex_match(_name, name_type)) {
				throw __EXCEPT__(WRONG_FILE_FORMAT);
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_VALID)) {
				throw __EXCEPT__(BAD_SLICE);
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
			stream_size += sizeof(uint8_t) * 1;

			if (fwrite(&_slice.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			stream_size += sizeof(int) * 1;

			if (fwrite(&_slice.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			stream_size += sizeof(int) * 1;

			if (fwrite(&_slice.size, sizeof(size_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			stream_size += sizeof(size_t) * 1;

			if (fwrite(&_slice.qp, sizeof(uint8_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			stream_size += sizeof(uint8_t) * 1;

			if (fwrite(&_slice.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			stream_size += sizeof(Eigen::Matrix4f) * 1;

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_PREDICT)) {
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
				stream_size += sizeof(size_t) * 1;

				if (fwrite(_slice.geometry->data(), sizeof(uint8_t), _slice.geometry->size(), fp) != _slice.geometry->size()) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
				stream_size += sizeof(uint8_t) * _slice.geometry->size();
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_SKIP)) {
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
				stream_size += sizeof(size_t) * 1;

				if (fwrite(_slice.color->data(), sizeof(uint8_t), _slice.color->size(), fp) != _slice.color->size()) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
				stream_size += sizeof(uint8_t) * _slice.color->size();
			}
			fclose(fp);
			return stream_size;
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
			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_VALID)) {
				throw __EXCEPT__(BAD_SLICE);
			}

			if (fread(&_slice.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.index, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.size, sizeof(size_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.qp, sizeof(uint8_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (fread(&_slice.mv, sizeof(Eigen::Matrix4f), 1, fp) != 1) {
				throw __EXCEPT__(FILE_READ_ERROR);
			}

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_PREDICT)) {
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

			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_SKIP)) {
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
			fgetc(fp);
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

	void ChangeSliceToFrame(const std::filesystem::path& _input_dir, const std::string& _output_name) {
		try {
			/* Check directiory */
			if (!std::filesystem::is_directory(_input_dir)) {
				throw __EXCEPT__(INVALID_DIR);
			}

			/* Check output file format */
			std::regex format{"^.*_[0-9]+_[0-9]+\\.slice$"};
			std::regex output_format{"^.*\\.frame"};
			if (!std::regex_match(_output_name, output_format)) {
				throw __EXCEPT__(WRONG_FILE_FORMAT);
			}

			/* Search directory to find matching file name */
			std::filesystem::directory_iterator files{_input_dir};
			std::vector<std::string>            slice_names;
			for (const auto& i : files) {
				std::string file_name{i.path().c_str()};
				if (std::regex_match(file_name, format)) {
					slice_names.emplace_back(file_name);
				}
			}

			/* No matching file */
			if (slice_names.empty()) {
				throw __EXCEPT__(FILE_NOT_EXIST);
			}
			/* Load slice from matching file */
			std::vector<common::Slice> slices(slice_names.size());
			for (int i = 0; i < slice_names.size(); ++i) {
				LoadSlice(slices[i], slice_names[i]);
			}

			auto slice_cmp = [](const common::Slice& _x, const common::Slice& _y) -> bool { return _x.index < _y.index; };
			std::sort(slices.begin(), slices.end(), slice_cmp);

			/* NOTE: Just for test here */
			slices[0].index = 0;
			for (int i = 1; i < slices.size(); ++i) {
				slices[i].index = slices[0].index + i;
				slices[i].mv(2, 3) += i * 20.0f;
			}
			/* Frame time stamp */
			common::Frame frame(slices);

			FILE* fp = fopen(_output_name.c_str(), "wb");

			if (fp == nullptr) {
				switch (errno) {
					case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
					case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
					default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
				}
			}

			if (fwrite(&frame.timestamp, sizeof(int), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(&frame.slice_cnt, sizeof(uint32_t), 1, fp) != 1) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.index.data(), sizeof(int), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.type.data(), sizeof(uint8_t), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.size.data(), sizeof(uint32_t), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.qp.data(), sizeof(uint8_t), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.mv.data(), sizeof(Eigen::Matrix4f), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			if (fwrite(frame.geometry_size.data(), sizeof(uint32_t), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}
			for (auto& i : frame.geometry) {
				if (!i) {
					continue;
				}
				if (fwrite(i->data(), sizeof(uint8_t), i->size(), fp) != i->size()) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}

			if (fwrite(frame.color_size.data(), sizeof(uint32_t), frame.slice_cnt, fp) != frame.slice_cnt) {
				throw __EXCEPT__(FILE_WRITE_ERROR);
			}

			for (auto& i : frame.color) {
				if (!i) {
					continue;
				}
				if (fwrite(i->data(), sizeof(uint8_t), i->size(), fp) != i->size()) {
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
}  // namespace io
}  // namespace vvc
