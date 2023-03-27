/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of Class ICP and NICP in module vvc::registration
 * Create Time   : 2022/12/26 10:54
 * Last Modified : 2023/03/27 11:27
 *
 */

#include "registration/registration.h"

using namespace vvc;

vvc::registration::ICP::ICP() : vvc::registration::ICPBase::ICPBase{}, icp_{nullptr} {}

void vvc::registration::ICP::Align() {
	try {
		/* Check point cloud is empty */
		if (!this->source_cloud_ || !this->target_cloud_ || this->source_cloud_->empty() || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}

		/* Check param is empty */
		if (!this->params_) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}

		if (this->result_cloud_) {
			throw __EXCEPT__(INITIALIZER_ERROR);
		}

        /* Create icp instance */
		if (this->params_->icp.type == common::SIMPLE_ICP) {
			this->icp_ = pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
		}
		else if (this->params_->icp.type == common::LM_ICP) {
			this->icp_ = pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB>());
		}
		else if (this->params_->icp.type == common::GENERAL_ICP) {
			this->icp_ = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());
		}
		else {
			throw __EXCEPT__(BAD_PARAMETERS);
		}
		
        /* Initlize result cloud by source cloud */
        this->result_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		*(this->result_cloud_) += *(this->source_cloud_);

        /* Check params illegal */
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

        this->icp_->setInputSource(this->result_cloud_);
        this->icp_->setInputTarget(this->target_cloud_);

        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;

        this->icp_->align(temp_cloud);

        if (this->icp_->hasConverged()) {
            this->motion_vector_ = this->icp_->getFinalTransformation() * this->motion_vector_;
            this->result_cloud_->swap(temp_cloud);
            this->converged_ = true;
        }
        this->mse_ = this->CloudMSE();
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
