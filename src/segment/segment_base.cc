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
namespace vvc {
namespace segment {
	void SegmentBase::SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src) {
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

	std::vector<common::Patch> SegmentBase::GetResultPatches() {
		try {
			/* no result */
			if (this->results_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			std::vector<common::Patch> result;
			int                        cnt = 0;
			for (auto i : this->results_) {
				if (!i) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				if (!i->empty()) {
					result.emplace_back();
					result.back().timestamp = this->timestamp_;
					result.back().cloud     = i;
					result.back().index     = cnt;
					cnt++;
				}
			}
			/* release source cloud */
			this->source_cloud_.reset();
			return result;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void SegmentBase::SetParams(common::PVVCParam_t::Ptr _ptr) {
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

	void SegmentBase::SetTimeStamp(int _time) {
		try {
			if (_time < 0) {
				throw __EXCEPT__(BAD_TIME_STAMP);
			}
			this->timestamp_ = _time;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

}  // namespace segment
}  // namespace vvc
