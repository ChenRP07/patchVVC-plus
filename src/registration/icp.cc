/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/26 10:54
 * Last Modified : 2022/12/27 16:53
 *
 */

#include "registration/registration.h"

using namespace vvc;

registration::ICP::ICP() : result_cloud_{new pcl::PointCloud<pcl::PointXYZRGB>}, motion_vector_{Eigen::Matrix4f::Identity()}, mse_{0.0f} {}

void registration::ICP::SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
		if (!_cloud || _cloud->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		else {
			this->source_cloud_ = _cloud;
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void registration::ICP::GetResultCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
		if (!this->result_cloud_ || this->result_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		else {
			_cloud = this->result_cloud_;
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

Eigen::Matrix4f registration::ICP::GetMotionVector() const {
	return this->motion_vector_;
}

float registration::ICP::GetMSE() const {
	return this->mse_;
}

void registration::ICP::CentroidAlignment() {
	try {
		if (!this->source_cloud_ || !this->target_cloud_ || this->source_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		/* calculate centroids */
		pcl::PointXYZ source_centroid(0.0f, 0.0f, 0.0f);
		pcl::PointXYZ target_centroid(0.0f, 0.0f, 0.0f);

		for (auto& i : *(this->source_cloud_)) {
			source_centroid.x += i.x, source_centroid.y += i.y, source_centroid.z += i.z;
		}

		for (auto& i : *(this->target_cloud_)) {
			target_centroid.x += i.x, target_centroid.y += i.y, target_centroid.z += i.z;
		}

		source_centroid.x /= this->source_cloud_->size();
		source_centroid.y /= this->source_cloud_->size();
		source_centroid.z /= this->source_cloud_->size();

		target_centroid.x /= this->target_cloud_->size();
		target_centroid.y /= this->target_cloud_->size();
		target_centroid.z /= this->target_cloud_->size();

		/* translate source point to result point */
		this->result_cloud_->clear();

		for (auto& i : *(this->source_cloud_)) {
			pcl::PointXYZRGB temp = i;
			temp.x += target_centroid.x - source_centroid.x;
			temp.y += target_centroid.y - source_centroid.y;
			temp.z += target_centroid.z - source_centroid.z;
			this->result_cloud_->emplace_back(temp);
		}

		/* record translation vector */
		this->motion_vector_(0, 3) += target_centroid.x - source_centroid.x;
		this->motion_vector_(1, 3) += target_centroid.y - source_centroid.y;
		this->motion_vector_(2, 3) += target_centroid.z - source_centroid.z;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

float registration::ICP::CloudMSE() {
	try {
		if (!this->result_cloud_ || !this->target_cloud_ || this->result_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		float MSE;

		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(this->target_cloud_);

		for (auto& i : *(this->result_cloud_)) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			kdtree.nearestKSearch(i, 1, idx, dis);
			MSE += dis.at(0);
		}

		MSE /= this->result_cloud_->size();
		return MSE;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void registration::ICP::Align() {
	try {
		if (!this->source_cloud_ || !this->target_cloud_ || this->source_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		if (!this->params_) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}

		if (!this->result_cloud_) {
			throw __EXCEPT__(INITIALIZER_ERROR);
		}

		this->CentroidAlignment();

		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

		if (this->params_->icp_.correspondence_ths_ > 0) {
			icp.setMaxCorrespondenceDistance(this->params_->icp_.correspondence_ths_);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp_.iteration_ths_ > 1) {
			icp.setMaximumIterations(this->params_->icp_.iteration_ths_);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp_.mse_ths_ > 0) {
			icp.setEuclideanFitnessEpsilon(this->params_->icp_.mse_ths_);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp_.transformation_ths_ > 0) {
			icp.setTransformationEpsilon(this->params_->icp_.transformation_ths_);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		icp.setInputSource(this->result_cloud_);
		icp.setInputTarget(this->target_cloud_);

		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_;

		icp.align(temp_cloud_);

		if (icp.hasConverged()) {
			this->mse_           = icp.getFitnessScore();
			this->motion_vector_ = icp.getFinalTransformation() * this->motion_vector_;
			this->result_cloud_->swap(temp_cloud_);
			std::cout << __GREENT__(ICP alignment completed.);
		}
		else {
			this->mse_ = this->CloudMSE();
			std::cout << __YELLOWT__(Warning : ICP alignment failed !) << std::endl;
		}
		std::cout << "Source/Target size : " << this->source_cloud_->size() << " / " << this->target_cloud_->size() << std::endl;
		std::cout << "Mse : " << this->mse_ << std::endl;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
