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

	std::vector<common::Patch> SegmentBase::GetResultPointClouds() {
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

	// clang-format off
    void SegmentBase::Log() const {
        std::cout << __AZURET__(===================================================) << std::endl;
        if (this->params_->log_level & 0x01) {
            std::cout << __BLUET__(Excepted patch number : ) << " " << this->stat_.expect_.size() << std::endl;
        }
        if (this->params_->log_level & 0x02) {
            std::cout << __BLUET__(Average patch size : ) << " " << this->source_cloud_->size() / this->stat_.expect_.size() << std::endl;
            std::cout << __BLUET__(Min/Max patch size : ) << " " << *std::min_element(this->stat_.expect_.begin(), this->stat_.expect_.end()) << " / " << *std::max_element(this->stat_.expect_.begin(), this->stat_.expect_.end()) << std::endl;
        }
        if (this->params_->log_level & 0x04) {
            /* TODO : complete log */
            std::cout << __BLUET__(Standard deviation : ) << " " << common::Deviation(this->stat_.expect_) << std::endl;
        }

        std::cout << __AZURET__(===================================================) << std::endl;
        if (this->params_->log_level & 0x01) {
            std::cout << __BLUET__(Actual patch number : ) << " " << this->stat_.fact_.size() << std::endl;
        }
        if (this->params_->log_level & 0x02) {
            std::cout << __BLUET__(Average patch size : ) << " " << std::accumulate(this->stat_.fact_.begin(), this->stat_.fact_.end(), 0) / this->stat_.fact_.size() << std::endl;
            std::cout << __BLUET__(Min/Max patch size : ) << " " << *std::min_element(this->stat_.fact_.begin(), this->stat_.fact_.end()) << " / " << *std::max_element(this->stat_.fact_.begin(), this->stat_.fact_.end()) << std::endl;
        }
        if (this->params_->log_level & 0x04) {
            /* TODO : complete log */
            std::cout << __BLUET__(Standard deviation : )  << " " << common::Deviation(this->stat_.fact_) << std::endl;
        }
        std::cout << __AZURET__(===================================================) << std::endl;

        std::cout << __BLUET__(Time consuming : );
        printf("%.3fs / %.3fms\n", this->clock_.GetTimeS(), this->clock_.GetTimeMs());
        std::cout << __AZURET__(===================================================) << std::endl;
    }

	// clang-format on
}  // namespace segment
}  // namespace vvc
