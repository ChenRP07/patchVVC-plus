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

void segment::SegmentBase::SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src) {
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

void segment::SegmentBase::GetResultPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _result) {
	try {
		/* no result */
		if (this->results_.empty()) {
			throw __EXCEPT__(EMPTY_RESULT);
		}

		_result.clear();
		_result.assign(this->results_.begin(), this->results_.end());
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}

void segment::SegmentBase::SetParams(common::PVVCParam_t::Ptr _ptr) {
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

void segment::SegmentBase::Log() const {
    std::cout << __AZURET__(===================================================) << std::endl;
	if (this->params_->log_level & 0x01) {
		std::cout << __BLUET__(Excepted patch number : ) << " " << this->stat_.expect_.size() << std::endl;
	}
	if (this->params_->log_level & 0x02) {
		std::cout << __BLUET__(Average patch size : ) << " " << this->source_cloud_->size() / this->stat_.expect_.size() << std::endl;
        std::cout << __BLUET__(Min/Max patch size : ) << " " << *std::min_element(this->stat_.expect_.begin(), this->stat_.expect_.end()) << " / " << *std::max_element(this->stat_.expect_.begin(), this->stat_.expect_.end()) << std::endl;
		std::cout << __BLUET__(Standard deviation : ) << " " << common::Deviation(this->stat_.expect_) << std::endl;
	}
	if (this->params_->log_level & 0x04) {
		/* TODO : complete log */
	}

    std::cout << __AZURET__(===================================================) << std::endl;
	if (this->params_->log_level & 0x01) {
		std::cout << __BLUET__(Actual patch number : ) << " " << this->stat_.fact_.size() << std::endl;
	}
	if (this->params_->log_level & 0x02) {
		std::cout << __BLUET__(Average patch size : ) << " " << std::accumulate(this->stat_.fact_.begin(), this->stat_.fact_.end(), 0) / this->stat_.fact_.size() << std::endl;
        std::cout << __BLUET__(Min/Max patch size : ) << " " << *std::min_element(this->stat_.fact_.begin(), this->stat_.fact_.end()) << " / " << *std::max_element(this->stat_.fact_.begin(), this->stat_.fact_.end()) << std::endl;
		std::cout << __BLUET__(Standard deviation : )  << " " << common::Deviation(this->stat_.fact_) << std::endl;
	}
	if (this->params_->log_level & 0x04) {
		/* TODO : complete log */
	}
    std::cout << __AZURET__(===================================================) << std::endl;
}
