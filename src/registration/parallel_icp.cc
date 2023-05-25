/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/29 11:03
 * Last Modified : 2022/12/29 11:03
 *
 */

#include "registration/registration.h"

using namespace vvc;
namespace vvc {
namespace registration {
	void ParallelICP::SetSourcePatches(std::vector<common::Patch>& _patches) {
		try {
			/* check point cloud is empty */
			if (_patches.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			for (auto& i : _patches) {
				if (!i || i.empty()) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
			}

			/* copy to source_clouds_ */
			this->reference_patches_.clear();
			this->reference_patches_.assign(_patches.begin(), _patches.end());

			/* init result_clouds_ */
			this->result_patches_.clear();
			this->result_patches_.resize(this->reference_patches_.size(), common::Patch());
			for (int i = 0; i < this->reference_patches_.size(); ++i) {
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGB>());
				*p += *(this->reference_patches_[i].cloud);
				this->result_patches_[i].cloud = p;
				/* Copy index, increment timestamp */
				this->result_patches_[i].index = this->reference_patches_[i].index;
				this->result_patches_[i].timestamp = this->reference_patches_[i].timestamp + this->params_->time_interval;
				/* Initialize type, simple patch */
				this->result_patches_[i].type = common::PATCH_TYPE::SIMPLE_PATCH;
			}

			/* init */
			this->converged_.resize(this->reference_patches_.size(), false);
			this->mse_.resize(this->reference_patches_.size(), -1.0f);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<common::Patch> ParallelICP::GetResultPatches() {
		try {
			/* check point cloud is empty */
			if (this->result_patches_.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
			std::vector<common::Patch> result;
			for (auto& i : this->result_patches_) {
				if (!i.cloud) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				if (!i.empty()) {
					result.emplace_back();
					result.back().timestamp = i.timestamp;
					result.back().index = i.index;
					result.back().cloud = i.cloud;
					result.back().type = i.type;
				}
			}
			return result;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void ParallelICP::CentroidAlignment() {
		/* global centroid alignment*/
		/* calculate centroids */
		pcl::PointXYZ source_global_centroid(0.0f, 0.0f, 0.0f), target_global_centroid(0.0f, 0.0f, 0.0f);
		size_t source_size = 0;
		for (auto& i : this->result_patches_) {
			for (auto j : *i) {
				source_global_centroid.x += j.x;
				source_global_centroid.y += j.y;
				source_global_centroid.z += j.z;
				source_size += 1;
			}
		}

		for (auto i : *(this->target_cloud_)) {
			target_global_centroid.x += i.x;
			target_global_centroid.y += i.y;
			target_global_centroid.z += i.z;
		}

		source_global_centroid.x /= source_size;
		source_global_centroid.y /= source_size;
		source_global_centroid.z /= source_size;

		target_global_centroid.x /= this->target_cloud_->size();
		target_global_centroid.y /= this->target_cloud_->size();
		target_global_centroid.z /= this->target_cloud_->size();

		/* move point and fill the result_clouds_ */
		for (size_t i = 0; i < this->result_patches_.size(); i++) {
			for (auto& j : *(this->result_patches_.at(i))) {
				j.x += target_global_centroid.x - source_global_centroid.x;
				j.y += target_global_centroid.y - source_global_centroid.y;
				j.z += target_global_centroid.z - source_global_centroid.z;
			}
		}
	}

	void ParallelICP::Task() {
		while (true) {
			/* loop until task_queue_ is empty */
			size_t task_idx = 0;
			bool is_end = true;
			/* get task index from task_queue_ */
			this->task_mutex_.lock();
			if (!this->task_queue_.empty()) {
				task_idx = this->task_queue_.front();
				is_end = false;
				this->task_queue_.pop();
			}
			this->task_mutex_.unlock();

			if (is_end) {
				break;
			}
			/* for each patch do nearest neighbor search and movement */
			pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(this->target_cloud_);

			pcl::PointXYZ local(0.0f, 0.0f, 0.0f);
			for (auto i : *(this->result_patches_[task_idx])) {
				std::vector<int> idx(1);
				std::vector<float> dis(1);
				kdtree.nearestKSearch(i, 1, idx, dis);
				local.x += this->target_cloud_->at(idx[0]).x - i.x;
				local.y += this->target_cloud_->at(idx[0]).y - i.y;
				local.z += this->target_cloud_->at(idx[0]).z - i.z;
			}
			local.x /= this->result_patches_[task_idx].size();
			local.y /= this->result_patches_[task_idx].size();
			local.z /= this->result_patches_[task_idx].size();

			for (auto& i : *(this->result_patches_[task_idx])) {
				i.x += local.x;
				i.y += local.y;
				i.z += local.z;
			}

			vvc::registration::ICPBase::Ptr icp;
			icp.reset(new vvc::registration::ICP());
			icp->SetParams(this->params_);
			icp->SetTargetCloud(this->target_cloud_);
			icp->SetSourceCloud(this->result_patches_[task_idx].cloud);
			icp->Align();

			/* converge or not */
			if (icp->Converged()) {
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = icp->GetResultCloud();
				this->result_patches_[task_idx].cloud->swap(*temp);
				this->converged_[task_idx] = 1;
				this->mse_[task_idx] = icp->GetMSE();
			}
			else {
				this->converged_[task_idx] = 0;
			}
		}
	}

	void ParallelICP::Align() {
		try {
			/* check point cloud is empty */
			if (!this->target_cloud_ || this->reference_patches_.empty() || this->result_patches_.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			for (auto i : this->reference_patches_) {
				if (!i || i.empty()) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
			}

			for (auto i : this->result_patches_) {
				if (!i) {
					throw __EXCEPT__(INITIALIZER_ERROR);
				}
			}

			/* check params is empty */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			/* Check params is illegal */
			if (this->params_->icp.correspondence_ths <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (this->params_->icp.iteration_ths <= 1) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (this->params_->icp.mse_ths <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (this->params_->icp.transformation_ths <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (this->params_->thread_num < 1) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			/* need to centroid alignment */
			if (this->params_->icp.centroid_alignment) {
				this->CentroidAlignment();
			}

			// this->params_ = common::CopyParams(this->params_);

			/* fill the task_queue_ */
			for (size_t i = 0; i < this->result_patches_.size(); ++i) {
				this->task_queue_.push(i);
			}

			/* create threads and arrange task */
			std::vector<std::thread> thread_pool(this->params_->thread_num);
			for (auto& i : thread_pool) {
				i = std::thread(&registration::ParallelICP::Task, this);
			}

			/* block until all threads are ended */
			for (auto& i : thread_pool) {
				i.join();
			}

			/* Segment target_cloud_ according to the result_clouds_ */
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
			std::vector<std::pair<int, float>> source_index;

			for (int i = 0; i < this->result_patches_.size(); ++i) {
				for (auto j : *(this->result_patches_[i])) {
					search_cloud->emplace_back(j);
					source_index.emplace_back(std::make_pair(i, this->mse_[i]));
				}
			}

			pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
			kdtree.setInputCloud(search_cloud);

			std::vector<pcl::PointCloud<pcl::PointXYZRGB>> temp_clouds(this->result_patches_.size(), pcl::PointCloud<pcl::PointXYZRGB>());
			for (int i = 0; i < this->target_cloud_->size(); ++i) {
				std::vector<int> idx(5);
				std::vector<float> dis(5);
				kdtree.nearestKSearch(this->target_cloud_->at(i), 5, idx, dis);
				int t_idx{};
				for (int t = 0; t < 5; ++t) {
					if (dis[t] < source_index[idx[t]].second * 2) {
						t_idx = t;
						break;
					}
				}
				temp_clouds[source_index[idx[t_idx]].first].emplace_back(this->target_cloud_->at(i));
			}

			for (int i = 0; i < this->result_patches_.size(); ++i) {
				this->result_patches_[i].cloud->swap(temp_clouds[i]);
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<float> registration::ParallelICP::GetStat() const {
		std::vector<float> res(this->mse_.size(), -1.0f);
		for (int i = 0; i < this->mse_.size(); ++i) {
			if (this->converged_[i]) {
				res[i] = this->mse_[i];
			}
		}
		return res;
	}
}  // namespace registration
}  // namespace vvc
