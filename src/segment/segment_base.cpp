/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/09 15:12
 * Last Modified : 2022/12/09 15:12
 *
 */

#include "segment/segment.h"

using namespace vvc;

void segment::segment_base::SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src) {
	try {
        /* empty input point cloud */
		if (!_src || _src->size() == 0) {
			throw common::__EXCEPT__(common::EMPTY_POINT_CLOUD);
		}

		this->source_cloud_ = _src;
	}
	catch (const common::exception& e) {
		e.log();
		throw common::__EXCEPT__(common::ERROR_OCCURED);
	}
}

void segment::segment_base::GetResultPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _result) {
	try {
        /* no result */
		if (this->results_.empty()) {
			throw common::__EXCEPT__(common::EMPTY_RESULT);
		}

        _result.clear();
        _result.assign(this->results_.begin(), this->results_.end());
	}
	catch (const common::exception& e) {
		e.log();
		throw common::__EXCEPT__(common::ERROR_OCCURED);
	}
}
