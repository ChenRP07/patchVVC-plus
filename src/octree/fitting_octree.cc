/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/03/20 10:04
 * Last Modified : 2023/03/20 10:04
 *
 */

#include "octree/octree.h"

using namespace vvc;

void octree::FittingOctree::AddPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
		if (!_cloud || _cloud->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		this->encoding_clouds_.push_back(_cloud);
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void octree::FittingOctree::MakeTree() {
	try {
		/* Check whether point cloud is empty */
		if (this->encoding_clouds_.empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		for (auto& i : this->encoding_clouds_) {
			if (!i || i->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
		}

		/* Sum of all size */
		size_t total = 0;

		/* Calculate bounding box of all cloud */
		float max_x = FLT_TRUE_MIN, max_y = FLT_TRUE_MIN, max_z = FLT_TRUE_MIN;
		float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

		for (auto& ptr : this->encoding_clouds_) {
			total += ptr->size();
			for (auto& p : *ptr) {
				min_x = std::min(min_x, p.x);
				min_y = std::min(min_y, p.y);
				min_z = std::min(min_z, p.z);
				max_x = std::max(max_x, p.x);
				max_y = std::max(max_y, p.y);
				max_z = std::max(max_z, p.z);
			}
		}

		/* Average cloud size */
		this->avg_size_ = static_cast<size_t>(std::ceil(static_cast<float>(total) / static_cast<float>(this->encoding_clouds_.size())));

		/* Max range of x-y-z dimension */
		float range = std::max(std::max(max_x - min_x, max_y - min_y), max_z - min_z);
		/* tree_range_ = 2^(ceiling(log(range))) */
		this->tree_range_ = std::pow(2.0f, std::ceil(std::log2(range)));

		this->tree_center_.x = max_x / 2.0f + min_x / 2.0f;
		this->tree_center_.y = max_y / 2.0f + min_y / 2.0f;
		this->tree_center_.z = max_z / 2.0f + min_z / 2.0f;

		/* Using a point index vector to indicate points that tree node contains */
		std::vector<std::vector<size_t>> point_idx(this->encoding_clouds_.size());

		for (size_t i = 0; i < this->encoding_clouds_.size(); ++i) {
			while (point_idx.size() < this->encoding_clouds_[i]->size()) {
				point_idx.emplace_back(point_idx.size());
			}
		}

		this->GeneratePoints(point_idx, this->tree_range_);
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void octree::FittingOctree::GeneratePoints(std::vector<std::vector<size_t>>& _point_idx, float _res) {
    try {

    } catch (const common::Exception& e) {
        e.Log();
        throw __EXCEPT__(ERROR_OCCURED);
    }
}
