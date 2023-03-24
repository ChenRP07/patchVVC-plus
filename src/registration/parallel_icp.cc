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

void registration::ParallelICP::SetSourceClouds(std::vector<common::Patch>& _clouds) {
	try {
		/* check point cloud is empty */
		if (_clouds.empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		for (auto i : _clouds) {
			if (!i || i.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
		}

		/* copy to source_clouds_ */
		this->source_clouds_.clear();
		this->source_clouds_.assign(_clouds.begin(), _clouds.end());

		/* init result_clouds_ */
		this->result_clouds_.clear();
		this->result_clouds_.resize(this->source_clouds_.size(), common::Patch());
		for (int i = 0; i < this->source_clouds_.size(); ++i) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr p(new pcl::PointCloud<pcl::PointXYZRGB>());
			this->result_clouds_[i].cloud     = p;
			this->result_clouds_[i].index     = this->source_clouds_[i].index;
			this->result_clouds_[i].timestamp = this->source_clouds_[i].timestamp + 1;
		}

		/* init */
		this->mses_.resize(this->source_clouds_.size(), 0.0f);
		this->stat_.converged_.resize(this->source_clouds_.size(), 1);
		this->stat_.score_.resize(this->source_clouds_.size(), 0.0f);
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void registration::ParallelICP::GetResultClouds(std::vector<common::Patch>& _clouds) {
	try {
		/* check point cloud is empty */
		if (this->result_clouds_.empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		for (auto i : this->result_clouds_) {
			if (!i || i.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
		}

		_clouds.clear();
		_clouds.assign(this->result_clouds_.begin(), this->result_clouds_.end());
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

std::vector<float> registration::ParallelICP::GetMSEs() const {
	return this->mses_;
}

void registration::ParallelICP::CentroidAlignment() {
	/* global centroid alignment*/
	/* calculate centroids */
	pcl::PointXYZ source_global_centroid(0.0f, 0.0f, 0.0f), target_global_centroid(0.0f, 0.0f, 0.0f);
	size_t        source_size = 0;
	for (auto& i : this->source_clouds_) {
		for (auto& j : *i.cloud) {
			source_global_centroid.x += j.x;
			source_global_centroid.y += j.y;
			source_global_centroid.z += j.z;
			source_size += 1;
		}
	}

	for (auto& i : *(this->target_cloud_)) {
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
	for (size_t i = 0; i < this->result_clouds_.size(); i++) {
		this->result_clouds_[i].cloud->clear();
		for (auto& j : *(this->source_clouds_.at(i).cloud)) {
			pcl::PointXYZRGB temp = j;
			temp.x += target_global_centroid.x - source_global_centroid.x;
			temp.y += target_global_centroid.y - source_global_centroid.y;
			temp.z += target_global_centroid.z - source_global_centroid.z;
			this->result_clouds_[i].cloud->emplace_back(temp);
		}
	}
}

void registration::ParallelICP::CloudMSE() {
	/* calculate mse for each patch */
	pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(this->target_cloud_);
	this->mses_.clear();
	for (auto& i : this->result_clouds_) {
		float temp = 0.0f;
		for (auto& j : *i.cloud) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			kdtree.nearestKSearch(j, 1, idx, dis);
			temp += dis[0];
		}
		temp /= i.size();
		this->mses_.emplace_back(temp);
	}
}

void registration::ParallelICP::Task() {
	while (true) {
		/* loop until task_queue_ is empty */
		size_t task_idx = 0;
		bool   is_end   = true;
		/* get task index from task_queue_ */
		this->task_mutex_.lock();
		if (!this->task_queue_.empty()) {
			task_idx = this->task_queue_.front();
			is_end   = false;
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
		for (auto& i : *(this->result_clouds_[task_idx].cloud)) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			kdtree.nearestKSearch(i, 1, idx, dis);
			local.x += this->target_cloud_->at(idx[0]).x - i.x;
			local.y += this->target_cloud_->at(idx[0]).y - i.y;
			local.z += this->target_cloud_->at(idx[0]).z - i.z;
		}
		local.x /= this->result_clouds_[task_idx].size();
		local.y /= this->result_clouds_[task_idx].size();
		local.z /= this->result_clouds_[task_idx].size();

		for (auto& i : *(this->result_clouds_[task_idx].cloud)) {
			i.x += local.x;
			i.y += local.y;
			i.z += local.z;
		}
		/* icp */
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setMaximumIterations(this->params_->icp.iteration_ths);
		icp.setMaxCorrespondenceDistance(this->params_->icp.correspondence_ths);
		icp.setEuclideanFitnessEpsilon(this->params_->icp.mse_ths);
		icp.setTransformationEpsilon(this->params_->icp.transformation_ths);

		icp.setInputSource(this->result_clouds_[task_idx].cloud);
		icp.setInputTarget(this->target_cloud_);

		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
		icp.align(temp_cloud);

		/* converge or not */
		if (icp.hasConverged()) {
			this->mses_[task_idx] = this->stat_.score_[task_idx] = icp.getFitnessScore();
			this->result_clouds_[task_idx].cloud->swap(temp_cloud);
			this->stat_.converged_[task_idx] = 1;
		}
		else {
			this->stat_.score_[task_idx]     = this->mses_[task_idx];
			this->stat_.converged_[task_idx] = 0;
		}
	}
}

void registration::ParallelICP::Align() {
	try {
		std::cout << __GREENT__(Start ICP registration.) << std::endl;
		std::cout << __GREENT__(Check point clouds.) << std::endl;
		/* check point cloud is empty */
		if (!this->target_cloud_ || this->source_clouds_.empty() || this->result_clouds_.empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		for (auto i : this->source_clouds_) {
			if (!i || i.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
		}

		for (auto i : this->result_clouds_) {
			if (!i) {
				throw __EXCEPT__(INITIALIZER_ERROR);
			}
		}

		std::cout << __GREENT__(Check parameters.) << std::endl;
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
			std::cout << __GREENT__(Process centroid alignment.) << std::endl;
			this->CentroidAlignment();
		}

		/* if no centroid alignemnt, fill the result_clouds_ */
		if (!this->params_->icp.centroid_alignment) {
			for (size_t i = 0; i < this->result_clouds_.size(); i++) {
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>());
				*temp += *(this->source_clouds_[i].cloud);
			}
		}

		/* fill the task_queue_ */
		for (size_t i = 0; i < this->result_clouds_.size(); i++) {
			this->task_queue_.push(i);
		}

		// clang-format off
		std::cout << __GREENT__(Launch multi-threads.) << std::endl;
        //clang-format on
		/* create threads and arrange task */
		std::vector<std::thread> thread_pool(this->params_->thread_num);
		for (auto& i : thread_pool) {
			i = std::thread(&registration::ParallelICP::Task, this);
		}

		/* block until all threads are ended */
		for (auto& i : thread_pool) {
			i.join();
		}

		std::cout << __GREENT__(Registration completed.) << std::endl;

		/* Segment target_cloud_ according to the result_clouds_ */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		std::vector<int>                       source_index;

		for (int i = 0; i < this->result_clouds_.size(); ++i) {
			for (auto j : *(this->result_clouds_[i].cloud)) {
				search_cloud->emplace_back(j);
				source_index.emplace_back(i);
			}
		}

		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(search_cloud);

		std::vector<pcl::PointCloud<pcl::PointXYZRGB>> temp_clouds(this->result_clouds_.size(), pcl::PointCloud<pcl::PointXYZRGB>());
		for (auto i : *(this->target_cloud_)) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			kdtree.nearestKSearch(i, 1, idx, dis);
			temp_clouds[source_index[idx[0]]].emplace_back(i);
		}

		for (int i = 0; i < this->result_clouds_.size(); ++i) {
			this->result_clouds_[i].cloud->swap(temp_clouds[i]);
		}

		/* log statistic */
		this->Log();
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
// clang-format off
void registration::ParallelICP::Log() {
	size_t converged = 0;
	for (auto i : this->stat_.converged_) {
		if (i != 0) {
			converged += 1;
		}
	}
	if (this->params_->log_level & 0x01) {
		std::cout << __BLUET__(Launch threads : ) << this->params_->thread_num << std::endl;
		std::cout << __BLUET__(Converged / Not patches : ) << converged << " / " << this->stat_.converged_.size() - converged << std::endl;
		std::cout << __AZURET__(===================================================) << std::endl;
	}

	if (this->params_->log_level & 0x02) {
		std::cout << __BLUET__(Average MSE : ) << std::accumulate(this->stat_.score_.begin(), this->stat_.score_.end(), 0) / this->stat_.score_.size() << std::endl;
		std::cout << __BLUET__(Min / Max MSE : ) << *std::min_element(this->stat_.score_.begin(), this->stat_.score_.end()) << " / "
		          << *std::max_element(this->stat_.score_.begin(), this->stat_.score_.end()) << std::endl;
		std::cout << __BLUET__(Standard deviation : ) << common::Deviation(this->stat_.score_) << std::endl;
		std::cout << __AZURET__(===================================================) << std::endl;
	}

	if (this->params_->log_level & 0x04) {
		for (size_t i = 0; i < this->stat_.converged_.size(); i++) {
			std::cout << __BLUET__(Patch #) << i << " : ";
			if (this->stat_.converged_[i] ==0) {
				std::cout << __YELLOWT__(is not converged);
			}
			else {
				std::cout << __GREENT__(is converged);
			}
			std::cout << __BLUET__(~~MSE is) << " " << this->stat_.score_[i] << std::endl;
		}
		std::cout << __AZURET__(===================================================) << std::endl;
	}
}
