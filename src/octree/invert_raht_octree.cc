/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/03 12:10
 * Last Modified : 2023/05/03 12:10
 *
 */

#include "octree/octree.h"

namespace vvc {
namespace octree {

	InvertRAHTOctree::InvertRAHTOctree() : tree_{}, source_cloud_{nullptr}, source_colors_{nullptr}, slice_{} {}

	void InvertRAHTOctree::SetSlice(const common::Slice& _slice) {
		try {
			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_VALID)) {
				throw __EXCEPT__(BAD_SLICE);
			}

			if (!_slice.geometry || !_slice.color) {
				throw __EXCEPT__(BAD_SLICE);
			}

			this->slice_ = _slice;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	vvc::common::Patch InvertRAHTOctree::GetPatch() const {
		try {
			if (!this->source_cloud_ || this->source_cloud_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			if (!this->source_colors_ || this->source_colors_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			vvc::common::Patch result;
			result.timestamp = this->slice_.timestamp;
			result.index     = this->slice_.index;
			result.mv        = this->slice_.mv;

			result.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			return result;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace octree
}  // namespace vvc
