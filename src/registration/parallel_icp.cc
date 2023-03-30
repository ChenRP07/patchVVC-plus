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
			*p += *(this->source_clouds_[i].cloud);
			this->result_clouds_[i].cloud     = p;
			this->result_clouds_[i].index     = this->source_clouds_[i].index;
			this->result_clouds_[i].timestamp = this->source_clouds_[i].timestamp + 1;
		}

		/* init */
		this->stat_.converged.resize(this->source_clouds_.size(), false);
		this->mse_.resize(this->source_clouds_.size(), -1.0f);
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

		_clouds.clear();
		for (auto i : this->result_clouds_) {
			if (!i.cloud) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
			if (!i.empty()) {
				_clouds.emplace_back();
				_clouds.back().timestamp = i.timestamp;
				_clouds.back().index     = i.index;
				_clouds.back().cloud     = i.cloud;
			}
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

common::ParallelICPStat_t registration::ParallelICP::GetScore() const {
	return std::move(this->stat_);
}

void registration::ParallelICP::CentroidAlignment() {
	/* global centroid alignment*/
	/* calculate centroids */
	pcl::PointXYZ source_global_centroid(0.0f, 0.0f, 0.0f), target_global_centroid(0.0f, 0.0f, 0.0f);
	size_t        source_size = 0;
	for (auto& i : this->result_clouds_) {
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
		for (auto& j : *(this->result_clouds_.at(i).cloud)) {
			j.x += target_global_centroid.x - source_global_centroid.x;
			j.y += target_global_centroid.y - source_global_centroid.y;
			j.z += target_global_centroid.z - source_global_centroid.z;
		}
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

		vvc::registration::ICPBase::Ptr icp;
		if (this->params_->icp.type == common::NORMAL_ICP) {
			icp.reset(new vvc::registration::NICP());
			icp->SetSourceNormal(this->source_normals_[task_idx]);
			icp->SetTargetNormal(this->target_normal_);
		}
		else {
			icp.reset(new vvc::registration::ICP());
		}

		icp->SetParams(this->params_);
		icp->SetTargetCloud(this->target_cloud_);
		icp->SetSourceCloud(this->result_clouds_[task_idx].cloud);
		icp->Align();

		/* converge or not */
		if (icp->Converged()) {
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp = icp->GetResultCloud();
			this->result_clouds_[task_idx].cloud->swap(*temp);
			this->stat_.converged[task_idx] = 1;
			this->mse_[task_idx]            = icp->GetMSE();
		}
		else {
			this->stat_.converged[task_idx] = 0;
		}
	}
}

void registration::ParallelICP::Align() {
	try {
		this->clock_.SetTimeBegin();
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

		if (this->params_->icp.type == common::NORMAL_ICP) {
			if (this->params_->icp.radius_search_ths <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> est;
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr           kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>());
			kdtree->setInputCloud(this->target_cloud_);
			est.setInputCloud(this->target_cloud_);
			est.setSearchMethod(kdtree);
			est.setRadiusSearch(this->params_->icp.radius_search_ths);
			est.compute(*(this->target_normal_));

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_source(new pcl::PointCloud<pcl::PointXYZRGB>());
			pcl::PointCloud<pcl::Normal>           temp_normal;
			for (auto i : this->result_clouds_) {
				for (auto j : *(i.cloud)) {
					temp_source->emplace_back(j);
				}
			}
			this->source_normals_.clear();
			this->source_normals_.resize(this->source_clouds_.size());
			kdtree->setInputCloud(temp_source);
			est.setInputCloud(temp_source);
			est.setSearchMethod(kdtree);
			est.setRadiusSearch(this->params_->icp.radius_search_ths);
			est.compute(temp_normal);

			int idx = 0;
			for (int i = 0; i < this->result_clouds_.size(); ++i) {
				for (int j = 0; j < this->result_clouds_[i].size(); ++j) {
					this->source_normals_[i]->emplace_back(temp_normal[idx]);
					idx++;
				}
			}
		}
		/* fill the task_queue_ */
		for (size_t i = 0; i < this->result_clouds_.size(); ++i) {
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

        this->clock_.SetTimeEnd();
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
	for (auto i : this->stat_.converged) {
		if (i != 0) {
			converged += 1;
		}
	}
	if (this->params_->log_level & 0x01) {
		std::cout << __BLUET__(Launch threads : ) << " " << this->params_->thread_num << std::endl;
		std::cout << __BLUET__(Converged / Not patches : ) << " " << converged << " / " << this->stat_.converged.size() - converged << std::endl;
        std::cout << __BLUET__(Time consuming : );
        printf(" %.3fs / %.3fms\n", this->clock_.GetTimeS(), this->clock_.GetTimeMs());
		std::cout << __AZURET__(===================================================) << std::endl;
	}
	if (this->params_->log_level & 0x02) {
        float mean = 0.0f, Max = 0.0f, Min = FLT_MAX;
        int cnt = 0;
        std::for_each(this->mse_.begin(), this->mse_.end(), [&mean, &Max, &Min, &cnt] (float x) {
                if (x >= 0.0f) {
                    mean += x, Max = std::max(x, Max), Min = std::min(x, Min);
                    cnt++;
                }
            });
        if (cnt != 0) {
            mean /= static_cast<float>(cnt);
        }
        else {
            mean = -1.0f;
        }
		std::cout << __BLUET__(Average  MSE : );
        printf(" %.3f\n", mean);
		std::cout << __BLUET__(Mininum  MSE : );
        printf(" %.3f\n", Min);
        std::cout << __BLUET__(Maximum  MSE : );
        printf(" %.3f\n", Max);
		std::cout << __BLUET__(Standard Err : ); 
        printf(" %.3f\n", common::Deviation(this->mse_));
		std::cout << __AZURET__(===================================================) << std::endl;
	}

	if (this->params_->log_level & 0x04) {
		for (size_t i = 0; i < this->stat_.converged.size(); i++) {
			std::cout << __BLUET__(Patch #) << i << " : ";
			if (this->stat_.converged[i] ==0) {
				std::cout << __YELLOWT__(is not converged);
			}
			else {
				std::cout << __GREENT__(is converged);
			}
			std::cout << __BLUET__(~~ MSE : ); 
            printf(" %.3f\n", this->mse_[i]);
		}
		std::cout << __AZURET__(===================================================) << std::endl;
	}
}
