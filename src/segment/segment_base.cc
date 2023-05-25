/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of segment base.
 * Create Time   : 2022/12/09 15:12
 * Last Modified : 2022/12/14 17:24
 *
 */

#include "segment/segment.h"

using namespace vvc;
namespace vvc {
namespace segment {
	void SegmentBase::SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src) {
		try {
			/* empty input point cloud */
			if (!_src || _src->size() == 0) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			this->source_cloud_ = _src;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<common::Patch> SegmentBase::GetResultPatches() {
		try {
			/* no result */
			if (this->results_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			std::vector<common::Patch> result;
			int cnt = 0;
			for (auto i : this->results_) {
				if (!i) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				if (!i->empty()) {
					result.emplace_back();
					result.back().timestamp = this->timestamp_;
					result.back().cloud = i;
					result.back().index = cnt;
					result.back().type = common::PATCH_TYPE::FORCE_KEY_PATCH;
					cnt++;
				}
			}
			/* release source cloud */
			this->source_cloud_.reset();
			return result;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void SegmentBase::SetParams(common::PVVCParam_t::Ptr _ptr) {
		try {
			if (!_ptr) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			else {
				this->params_ = _ptr;
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void SegmentBase::SetTimeStamp(int _time) {
		try {
			if (_time < 0) {
				throw __EXCEPT__(BAD_TIME_STAMP);
			}
			this->timestamp_ = _time;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void SegmentBase::KMeans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _centroids) {
		auto center = _centroids;
		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
		std::vector<int> idx(1);
		std::vector<float> dis(1);
		for (int iter = 0; iter < this->params_->patch.max_iter; ++iter) {
			/* New centers */
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr n_center(new pcl::PointCloud<pcl::PointXYZRGB>());
			n_center->resize(center->size());
			for (auto& c : *n_center) {
				c.x = c.y = c.z = 0.0f;
			}
			/* Cluster point num */
			std::vector<int> center_cnt(center->size(), 0);
			kdtree.setInputCloud(center);

			/* Add to cluster */
			for (auto& p : *this->source_cloud_) {
				kdtree.nearestKSearch(p, 1, idx, dis);
				n_center->at(idx[0]).x += p.x;
				n_center->at(idx[0]).y += p.y;
				n_center->at(idx[0]).z += p.z;
				center_cnt[idx[0]]++;
			}

			/* Compute new centroids */
			for (int i = 0; i < n_center->size(); ++i) {
				if (center_cnt[i] == 0) {
					center_cnt[i] = 1;
				}
				n_center->at(i).x /= center_cnt[i];
				n_center->at(i).y /= center_cnt[i];
				n_center->at(i).z /= center_cnt[i];
			}
			/* Compute error */
			float error{};

			for (int i = 0; i < n_center->size(); ++i) {
				error += std::pow(n_center->at(i).x - center->at(i).x, 2) + std::pow(n_center->at(i).y - center->at(i).y, 2) + std::pow(n_center->at(i).z - center->at(i).z, 2);
			}
			error /= n_center->size();
			/* New centroids */
			center.swap(n_center);
			if (error < this->params_->patch.clustering_err_ths) {
				break;
			}
		}
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> result;
		result.resize(center->size());
		for (auto& i : result) {
			i.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
		}

		kdtree.setInputCloud(center);
		for (auto& p : *this->source_cloud_) {
			kdtree.nearestKSearch(p, 1, idx, dis);
			result[idx[0]]->emplace_back(p);
		}
		this->results_.swap(result);
	}
}  // namespace segment
}  // namespace vvc
