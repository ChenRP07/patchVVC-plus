/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/26 10:47
 * Last Modified : 2022/12/26 10:47
 *
 */

#include "registration/registration.h"

using namespace vvc;

vvc::registration::RegistrationBase::RegistrationBase() : time_{}, target_cloud_{nullptr}, params_{nullptr} {}

void vvc::registration::RegistrationBase::SetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
        /* Check point cloud is empty */
		if (!_cloud || _cloud->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		else {
			this->target_cloud_ = _cloud;
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr vvc::registration::RegistrationBase::GetTargetCloud() const {
	try {
        /* check point cloud is empty */
		if (!this->target_cloud_ || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		
		return this->target_cloud_;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void vvc::registration::RegistrationBase::SetParams(common::PVVCParam_t::Ptr _param) {
	try {
        /* check param is empty */
		if (!_param) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}
		else {
			this->params_ = _param;
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

vvc::registration::ICPBase::ICPBase() : vvc::registration::RegistrationBase{}, source_cloud_{nullptr}, result_cloud_{nullptr}, motion_vector_{Eigen::Matrix4f::Identity()}, mse_{0.0f}, converged_{false} {}

void vvc::registration::ICPBase::SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
    try {
        /* Check point cloud is empty */
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr vvc::registration::ICPBase::GetResultCloud() const {
    try {
        /* check point cloud is empty */
		if (!this->result_cloud_ || this->result_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		
		return this->result_cloud_;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

Eigen::Matrix4f vvc::registration::ICPBase::GetMotionVector() const {
    return this->motion_vector_;
}

float vvc::registration::ICPBase::GetMSE() const {
    return this->mse_;
}

float vvc::registration::ICPBase::CloudMSE() const {
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
		for (auto i : *(this->result_cloud_)) {
			std::vector<int>   idx(1);
			std::vector<float> dis(1);
			kdtree.nearestKSearch(i, 1, idx, dis);
			MSE += dis.front();
		}

		MSE /= this->result_cloud_->size();
		return MSE;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
