/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : PLY file I/O for point cloud with point type xyzrgb.
 * Create Time   : 2022/12/06 17:20
 * Last Modified : 2022/12/08 19:18
 *
 */

#ifndef _PVVC_PLY_IO_H_
#define _PVVC_PLY_IO_H_

#include "common/exception.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <regex>
#include <stdio.h>
#include <string>

namespace vvc {
namespace io {
	/*
	 * @description: load cololed point cloud from a ply format file
	 * @param  : {const string&} file_name
	 * @return : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr} point_cloud
	 */
	extern pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoadColorPlyFile(const std::string& file_name);

	/*
	 * @description: save point_cloud to file_name, using binary_mode
	 * @param  : {const string&} file_name
	 * @param  : {const pcl::PointCloud<pcl::PointXYZRGB>::Ptr} point_cloud
	 * @param  : {bool} binary_mode
	 * @return : {}
	 */
	extern void SaveColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, bool binary_mode = false);

	/*
	 * @description: save point_cloud to file_name, set color to unique_color, using binary_mode
	 * @param  : {const string&} file_name
	 * @param  : {const pcl::PointCloud<pcl::PointXYZRGB>&} point_cloud
	 * @param  : {unsigned int} unique_color
	 * @param  : {bool} binary_mode
	 * @return : {}
	 */
	extern void SaveUniqueColorPlyFile(const std::string& file_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, unsigned int unique_color = 0x000000, bool binary_mode = false);

    /*
     * @description : Generate a file name for saving a patch.
     * @param  : {const std::string& path_name}
     * @param  : {const std::string& prev_name}
     * @param  : {int timestamp}
     * @param  : {int index}
     * @return : {std::string}
     * */
    extern std::string PatchFileName(const std::string& path_name, const std::string& prev_name, int timestamp, int index);

	static const std::string header_prev_ascii = "ply\nformat ascii 1.0\nelement vertex ";
	static const std::string header_prev_bin   = "ply\nformat binary_little_endian 1.0\nelement vertex ";
	static const std::string header_tail       = "\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

}  // namespace io
}  // namespace vvc

#endif
