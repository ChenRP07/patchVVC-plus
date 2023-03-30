/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of class RegistrationBase, ICPBase, MSE
 * Create Time   : 2022/12/26 10:47
 * Last Modified : 2023/03/29 16:49
 *
 */

#include "registration/registration.h"

using namespace vvc;

vvc::registration::RegistrationBase::RegistrationBase() : time_{}, target_cloud_{nullptr}, params_{nullptr} {}

inline void vvc::registration::RegistrationBase::SetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
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

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr vvc::registration::RegistrationBase::GetTargetCloud() const {
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

inline void vvc::registration::RegistrationBase::SetParams(common::PVVCParam_t::Ptr _param) {
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

vvc::registration::ICPBase::ICPBase()
    : vvc::registration::RegistrationBase{}, source_cloud_{nullptr}, result_cloud_{nullptr}, motion_vector_{Eigen::Matrix4f::Identity()}, mse_{0.0f}, converged_{false} {}

inline void vvc::registration::ICPBase::SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
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

inline pcl::PointCloud<pcl::PointXYZRGB>::Ptr vvc::registration::ICPBase::GetResultCloud() const {
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

inline Eigen::Matrix4f vvc::registration::ICPBase::GetMotionVector() const {
	return this->motion_vector_;
}

inline float vvc::registration::ICPBase::GetMSE() const {
	return this->mse_;
}

inline bool vvc::registration::ICPBase::Converged() const {
	return this->converged_;
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

vvc::common::MSE::MSE()
    : p_{nullptr}, q_{nullptr}, geo_mses_{std::make_pair(0.0f, 0.0f)}, y_mses_{std::make_pair(0.0f, 0.0f)}, u_mses_{std::make_pair(0.0f, 0.0f)}, v_mses_{std::make_pair(0.0f, 0.0f)} {}

inline void vvc::common::MSE::SetClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _x, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _y) {
	this->p_ = _x;
	this->q_ = _y;
}

void vvc::common::MSE::Compute() {
	try {
		if (!(this->p_) || this->p_->empty()) {
			this->geo_mses_.first = this->y_mses_.first = this->u_mses_.first = this->v_mses_.first = -1.0f;
		}
		if (!(this->q_) || this->q_->empty()) {
			this->geo_mses_.second = this->y_mses_.second = this->u_mses_.second = this->v_mses_.second = -1.0f;
		}

		if (this->geo_mses_.first < 0 || this->geo_mses_.second < 0) {
			return;
		}

		pcl::search::KdTree<pcl::PointXYZRGB> tree_p, tree_q;
		tree_p.setInputCloud(this->p_);
		tree_q.setInputCloud(this->q_);

		std::tuple<float, float, float> color_x(0.0f, 0.0f, 0.0f), color_y(0.0f, 0.0f, 0.0f);

		std::vector<int>   idx(1);
		std::vector<float> dis(1);
		for (auto i : *(this->p_)) {
			tree_q.nearestKSearch(i, 1, idx, dis);
			this->geo_mses_.first += dis.front();
			vvc::common::ColorYUV pp(i);
			vvc::common::ColorYUV qq(this->q_->at(idx.front()));
			this->y_mses_.first += std::pow(pp.y - qq.y, 2);
			this->u_mses_.first += std::pow(pp.u - qq.u, 2);
			this->v_mses_.first += std::pow(pp.v - qq.v, 2);
		}

		for (auto i : *(this->q_)) {
			tree_p.nearestKSearch(i, 1, idx, dis);
			this->geo_mses_.second += dis.front();
			vvc::common::ColorYUV qq(i);
			vvc::common::ColorYUV pp(this->p_->at(idx.front()));
			this->y_mses_.second += std::pow(pp.y - qq.y, 2);
			this->u_mses_.second += std::pow(pp.u - qq.u, 2);
			this->v_mses_.second += std::pow(pp.v - qq.v, 2);
		}

		this->geo_mses_.first /= this->p_->size();
		this->y_mses_.first /= this->p_->size();
		this->u_mses_.first /= this->p_->size();
		this->v_mses_.first /= this->p_->size();

		this->geo_mses_.second /= this->q_->size();
		this->y_mses_.second /= this->q_->size();
		this->u_mses_.second /= this->q_->size();
		this->v_mses_.second /= this->q_->size();
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

inline std::pair<float, float> vvc::common::MSE::GetGeoMSEs() const {
    return this->geo_mses_;
}

inline std::pair<float, float> vvc::common::MSE::GetYMSEs() const {
    return this->y_mses_;
}

inline std::pair<float, float> vvc::common::MSE::GetUMSEs() const {
    return this->u_mses_;
}

inline std::pair<float, float> vvc::common::MSE::GetVMSEs() const {
    return this->v_mses_;
}
