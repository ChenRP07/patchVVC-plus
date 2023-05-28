/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/25 14:15
 * Last Modified : 2023/05/25 14:15
 *
 */

#include "segment/segment.h"

namespace vvc {
namespace segment {

	void RefSegment::Segment() {
		try {
			if (!this->source_cloud_ || this->source_cloud_->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			if (this->reference_patches_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_centroids(new pcl::PointCloud<pcl::PointXYZRGB>());
			for (auto i : this->reference_patches_) {
				pcl::PointXYZRGB c{};
				for (auto& p : *i) {
					c.x += p.x, c.y += p.y, c.z += p.z;
				}
				int size = i.size();
				size = size == 0 ? 1 : size;
				c.x /= size, c.y /= size, c.z /= size;
				final_centroids->emplace_back(c);
			}
			this->KMeans(final_centroids);
			for (int i = 0; i < this->results_.size(); ++i) {
				if (this->results_[i]->empty()) {
					this->results_[i]->emplace_back(this->reference_patches_[i][0]);
				}
			}
			// bool size_ths = false;
			// int cur_idx = this->results_.size() - 1;
			// double r_ths = std::sqrt(this->params_->icp.radius_search_ths);
			// // double r_ths = std::sqrt(5.0f);
			// int MinPts = 4;
			//
			// std::vector<pcl::search::KdTree<pcl::PointXYZRGB>> trees(this->results_.size());
			// for (int i = 0; i < trees.size(); ++i) {
			// 	trees[i].setInputCloud(this->results_[i]);
			// }
			// while (true) {
			// 	int merge_idx = -1;
			// 	for (int k = cur_idx - 1; k >= 0; --k) {
			// 		if (this->results_[cur_idx]->size() + this->results_[k]->size() > this->params_->segment.num * (1.0f + NUM_THS)) {
			// 			continue;
			// 		}
			// 		size_ths = true;
			// 		int p_cnt{};
			// 		for (auto& p : *this->results_[cur_idx]) {
			// 			std::vector<int> idxes;
			// 			std::vector<float> dis;
			// 			trees[k].radiusSearch(p, r_ths, idxes, dis);
			// 			if (idxes.size() > MinPts) {
			// 				p_cnt++;
			// 			}
			// 			if (p_cnt > MinPts) {
			// 				merge_idx = k;
			// 				break;
			// 			}
			// 		}
			// 	}
			// 	if (merge_idx != -1) {
			// 		*this->results_[merge_idx] += *this->results_[cur_idx];
			// 		this->results_[cur_idx]->clear();
			// 		trees.erase(trees.begin() + cur_idx);
			// 		trees[merge_idx].setInputCloud(this->results_[merge_idx]);
			// 	}
			// 	if (size_ths) {
			// 		size_ths = false;
			// 		cur_idx--;
			// 	}
			// 	else {
			// 		break;
			// 	}
			// }
			// for (int i = 0; i < this->results_.size(); ++i) {
			// 	unsigned int color{static_cast<unsigned int>(rand() % 0x00ffffff)};
			// 	vvc::io::SaveUniqueColorPlyFile("./data/res_ply/res1_" + std::to_string(i) + ".ply", this->results_[i], color);
			// 	std::cout << this->results_[i]->size() << std::endl;
			// }
			// std::cout << "total : " << this->results_.size() << std::endl;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void RefSegment::SetRefPatches(std::vector<common::Patch> _patches) {
		this->reference_patches_ = _patches;
	}
}  // namespace segment
}  // namespace vvc
