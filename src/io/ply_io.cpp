/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : PLY I/O module, more details according to ply_io.h .
 * Create Time   : 2022/12/06 18:58
 * Last Modified : 2022/12/08 19:27
 *
 */

#include "io/ply_io.h"

using namespace vvc;

void io::LoadColorPlyFile(const std::string& file_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;

		if (!std::regex_search(file_name, matches, name_type)) {
			throw __EXCEPT__(WRONG_FILE_FORMAT);
		}

		FILE* fp = nullptr;
		fp       = fopen(file_name.c_str(), "r");
		if (fp == nullptr) {
			switch (errno) {
				case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
				case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
				default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
			}
			fclose(fp);
		}

		char file_line[1024];
		int  file_type = -1;
		fgets(file_line, 1024, fp);  // read line : "ply"
		fgets(file_line, 1024, fp);  // read line : "format ..."

		char* format;

		if ((format = strstr(file_line, "format ascii")) != nullptr) {
			file_type = 0;
		}
		else if ((format = strstr(file_line, "format binary")) != nullptr) {
			file_type = 1;
		}
		else {
			throw __EXCEPT__(WRONG_FILE_FORMAT);
		}

		// check points number
		int   point_cloud_size = -1;
		char* end_header;
		char* num_line;
		while (true) {
			fgets(file_line, 1024, fp);

			// header end
			if ((end_header = strstr(file_line, "end_header")) != nullptr) {
				if (point_cloud_size == -1) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				break;
			}

			// element vertex %d
			if ((num_line = strstr(file_line, "element vertex ")) != nullptr) {
				point_cloud_size = 0;
				for (int i = 15; i < strlen(file_line); i++) {
					if (file_line[i] >= '0' && file_line[i] <= '9') {
						point_cloud_size *= 10;
						point_cloud_size += (file_line[i] - '0');
					}
					else {
						break;
					}
				}
			}
		}
		// ascii
		if (file_type == 0) {
			for (size_t i = 0; i < point_cloud_size; i++) {
				pcl::PointXYZRGB point;
				float            x, y, z;
				int              r, g, b;
				if (fscanf(fp, "%f %f %f %d %d %d", &x, &y, &z, &r, &g, &b) != 6) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
				point.x = x, point.y = y, point.z = z, point.r = r, point.g = g, point.b = b;
				point_cloud->emplace_back(point);
			}
		}
		// binary
		else if (file_type == 1) {
			for (size_t i = 0; i < point_cloud_size; i++) {
				pcl::PointXYZRGB point;
                float xyz[3];
                uint8_t rgb[3];
				if (fread(xyz, sizeof(float), 3, fp) != 3) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
                if (fread(rgb, sizeof(uint8_t), 3, fp) != 3) {
					throw __EXCEPT__(FILE_READ_ERROR);
				}
                point.x = xyz[0], point.y = xyz[1], point.z = xyz[2];
                point.r = rgb[0], point.g = rgb[1], point.b = rgb[2];
				point_cloud->emplace_back(point);
			}
		}
		fclose(fp);
	}
	catch (const common::Exception& e) {
		e.Log();
        throw __EXCEPT__(ERROR_OCCURED);
	}
}

void io::SaveColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, bool binary_mode) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;
		if (!std::regex_search(file_name, matches, name_type)) {
			throw __EXCEPT__(WRONG_FILE_FORMAT);
		}

		// open file
		FILE* fp = nullptr;
		fp       = fopen(file_name.c_str(), "w");
		if (fp == nullptr) {
			switch (errno) {
				case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
				case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
				default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
			}
		}

		std::string header;
		if (binary_mode) {
			header = io::header_prev_bin + std::to_string(point_cloud->size()) + io::header_tail;
		}
		else {
			header = io::header_prev_ascii + std::to_string(point_cloud->size()) + io::header_tail;
		}

		if (fprintf(fp, "%s", header.c_str()) < 0) {
			throw __EXCEPT__(FILE_WRITE_ERROR);
		}
		pcl::PointXYZRGB temp;
		for (size_t i = 0; i < point_cloud->size(); i++) {
			temp = point_cloud->at(i);
			if (binary_mode) {
				float   xyz[3] = {temp.x, temp.y, temp.z};
				uint8_t rgb[3] = {temp.r, temp.g, temp.b};
				if (fwrite(xyz, sizeof(float), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}

				if (fwrite(rgb, sizeof(uint8_t), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}
			else {
				if (fprintf(fp, "%.3f %.3f %.3f %u %u %u\n", temp.x, temp.y, temp.z, temp.r, temp.g, temp.b) < 0) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}
		}
		fclose(fp);
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void io::SaveUniqueColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, unsigned int unique_color, bool binary_mode) {
	try {
		// file_name must be *.ply format
		std::regex  name_type{"^.*\\.ply$"};
		std::smatch matches;
		if (!std::regex_search(file_name, matches, name_type)) {
			throw __EXCEPT__(WRONG_FILE_FORMAT);
		}

		// open file
		FILE* fp = nullptr;
		fp       = fopen(file_name.c_str(), "w");
		if (fp == nullptr) {
			switch (errno) {
				case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
				case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
				default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
			}
		}

		std::string header;
		if (binary_mode) {
			header = io::header_prev_bin + std::to_string(point_cloud->size()) + io::header_tail;
		}
		else {
			header = io::header_prev_ascii + std::to_string(point_cloud->size()) + io::header_tail;
		}

		if (fprintf(fp, "%s", header.c_str()) < 0) {
			throw __EXCEPT__(FILE_WRITE_ERROR);
		}

		uint8_t blue = unique_color & 0x0000ff;
		unique_color >>= 8;
		uint8_t green = unique_color & 0x0000ff;
		unique_color >>= 8;
		uint8_t red = unique_color & 0x0000ff;

		pcl::PointXYZRGB temp;
		for (size_t i = 0; i < point_cloud->size(); i++) {
			temp   = point_cloud->at(i);
			temp.r = red, temp.g = green, temp.b = blue;

			if (binary_mode) {
				float   xyz[3] = {temp.x, temp.y, temp.z};
				uint8_t rgb[3] = {temp.r, temp.g, temp.b};
				if (fwrite(xyz, sizeof(float), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}

				if (fwrite(rgb, sizeof(uint8_t), 3, fp) != 3) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}
			else {
				if (fprintf(fp, "%.3f %.3f %.3f %u %u %u\n", temp.x, temp.y, temp.z, temp.r, temp.g, temp.b) < 0) {
					throw __EXCEPT__(FILE_WRITE_ERROR);
				}
			}
		}
		fclose(fp);
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

