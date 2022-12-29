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

void registration::RegistrationBase::SetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
	try {
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

void registration::RegistrationBase::GetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) const {
	try {
		if (!this->target_cloud_ || this->target_cloud_->empty()) {
			throw __EXCEPT__(EMPTY_POINT_CLOUD);
		}
		else {
			_cloud = this->target_cloud_;
		}
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void registration::RegistrationBase::SetParams(std::shared_ptr<common::VVCParam_t> _param) {
	try {
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
