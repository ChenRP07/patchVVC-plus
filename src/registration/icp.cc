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

registration::ICP::ICP() : result_cloud_{new pcl::PointCloud<pcl::PointXYZRGB>}, motion_vector_{Eigen::Matrix4f::Identity()}, mse_{0.0f}, icp_{nullptr} {}

void registration::ICP::SetParams(common::PVVCParam_t::Ptr _param) {
	try {
		/* check param is empty */
		if (!_param) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}
		else {
			this->params_ = _param;
		}

        if (this->params_->icp.type == common::SIMPLE_ICP){
            this->icp_ = pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
        }
        else if (this->params_->icp.type == common::LM_ICP) {
            this->icp_ = pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>());
        }
        else if (this->params_->icp.type == common::NORMAL_ICP) {
            this->icp_ = pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGB, pcl::PointXYZRGB>());
        }
        else if (this->params_->icp.type == common::GENERAL_ICP) {
            this->icp_ = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
        }
        else {
            throw __EXCEPT__(BAD_PARAMETERS);
        }
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void registration::ICP::SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
		/* check point cloud is empty */
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

void registration::ICP::SetSourceNormals(pcl::PointCloud<pcl::Normal>::Ptr _normals) {
    try{
        if (!_normals || _normals->empty()) {
            throw __EXCEPT__(EMPTY_POINT_CLOUD);
        }
        else {
            this->source_normal_ = _normals;
        }
    }
    catch (const common::Exception&e) {
        e.Log();
        throw  __EXCEPT__(ERROR_OCCURED);
    }
}

void registration::ICP::SetTargetNormals(pcl::PointCloud<pcl::Normal>::Ptr _normals) {
    try{
        if (!_normals || _normals->empty()) {
            throw __EXCEPT__(EMPTY_POINT_CLOUD);
        }
        else {
            this->target_normal_ = _normals;
        }
    }
    catch (const common::Exception&e) {
        e.Log();
        throw  __EXCEPT__(ERROR_OCCURED);
    }
}

void registration::ICP::GetResultCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
		/* check point cloud is empty */
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

float registration::ICP::CloudMSE() {
	try {
		/* check point cloud is empty */
		if (!this->result_cloud_ || !this->target_cloud_ || this->result_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		float MSE;

		/* nearest neighbor search */
		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(this->target_cloud_);

		/* calculate mse */
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
		/* check point cloud is empty */
		if (!this->source_cloud_ || !this->target_cloud_ || this->source_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		/* check param is empty */
		if (!this->params_) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}

		if (!this->result_cloud_) {
			throw __EXCEPT__(INITIALIZER_ERROR);
		}

        this->result_cloud_->clear();
        *(this->result_cloud_) += *(this->source_cloud_);


		/* check params illegal */
		if (this->params_->icp.correspondence_ths > 0) {
			this->icp_->setMaxCorrespondenceDistance(this->params_->icp.correspondence_ths);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp.iteration_ths > 1) {
			this->icp_->setMaximumIterations(this->params_->icp.iteration_ths);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp.mse_ths > 0) {
			this->icp_->setEuclideanFitnessEpsilon(this->params_->icp.mse_ths);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		if (this->params_->icp.transformation_ths > 0) {
			this->icp_->setTransformationEpsilon(this->params_->icp.transformation_ths);
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}

		/* set point cloud */
		this->icp_->setInputSource(this->result_cloud_);
		this->icp_->setInputTarget(this->target_cloud_);

		pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_;

		/* do alignment */
		this->icp_->align(temp_cloud_);

		/* converge or not */
		if (this->icp_->hasConverged()) {
			this->mse_           = this->icp_->getFitnessScore();
			this->motion_vector_ = this->icp_->getFinalTransformation() * this->motion_vector_;
			this->result_cloud_->swap(temp_cloud_);
		}
		else {
			this->mse_ = this->CloudMSE();
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
