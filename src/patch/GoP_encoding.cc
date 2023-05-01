/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/25 10:20
 * Last Modified : 2023/04/25 10:20
 *
 */

#include "patch/patch.h"

namespace vvc {
namespace patch {

	GoPEncoding::GoPEncoding() : params_{nullptr}, clock_{}, tree_{}, results_{}, fitting_cloud_{nullptr}, source_patches_{}, patch_colors_{} {}

	void GoPEncoding::SetParams(common::PVVCParam_t::Ptr _param) {
		try {
			if (!_param) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			this->params_ = _param;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GoPEncoding::SetSourcePatches(std::vector<common::Patch> _patches) {
		try {
			if (!this->source_patches_.empty()) {
				common::PVVCLog_Mutex.lock();
				std::cout << __YELLOWT__([Warning]) << " GoP is not empty when setting patches for it, the original data will be overwritten.\n";
				common::PVVCLog_Mutex.unlock();
			}

			if (_patches.empty()) {
				throw __EXCEPT__(EMPTY_GOP);
			}

			for (auto& i : _patches) {
				if (!i) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
			}
			this->source_patches_.swap(_patches);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GoPEncoding::SetFittingCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
		try {
			if (!_cloud || _cloud->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
			this->fitting_cloud_ = _cloud;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	common::ColorYUV GoPEncoding::Interpolation(int _i, std::vector<int>& _idx, std::vector<float>& _dis) {
		/* Use reciprocal of distance as the weight */
		for (auto& i : _dis) {
			if (i <= 1e-6) {
				i = 1e-6;
			}
			i = 1.0f / i;
		}
		/* Normalize weight */
		float sum = std::accumulate(_dis.begin(), _dis.end(), 0.0f);
		for (auto& i : _dis) {
			i /= sum;
		}
		/* Color interpolation */
		float color_r = 0.0f, color_g = 0.0f, color_b = 0.0f;
		for (int i = 0; i < _idx.size(); ++i) {
			color_r += _dis[i] * this->source_patches_[_i].cloud->at(_idx[i]).r;
			color_g += _dis[i] * this->source_patches_[_i].cloud->at(_idx[i]).g;
			color_b += _dis[i] * this->source_patches_[_i].cloud->at(_idx[i]).b;
		}

		return common::ColorYUV(color_r, color_g, color_b, false);
	}

	void GoPEncoding::Encode() {
		try {
			/* Check parameters and status here */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->params_->patch.interpolation_num <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (this->params_->slice.qp_i == 0 || this->params_->slice.qp_p == 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			if (!this->fitting_cloud_ || this->fitting_cloud_->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
			if (this->source_patches_.empty()) {
				throw __EXCEPT__(EMPTY_GOP);
			}

			/* Init RAHTOctree */
			this->tree_.SetParams(this->params_);
			/* Init slices */
			this->results_.resize(this->source_patches_.size());
			for (int i = 0; i < this->source_patches_.size(); ++i) {
				this->results_[i].type      = common::PVVC_SLICE_TYPE_INTER;
				this->results_[i].mv        = this->source_patches_[i].mv.inverse();
				this->results_[i].timestamp = this->source_patches_[i].timestamp;
				this->results_[i].index     = this->source_patches_[i].index;
				this->results_[i].geometry  = nullptr;
				this->results_[i].color     = nullptr;
			}
			/* First slice is an intra slice */
			this->results_.front().type = common::PVVC_SLICE_TYPE_INTRA;

			/* Color interpolation */
			this->patch_colors_.resize(this->source_patches_.size());
			for (int i = 0; i < this->source_patches_.size(); ++i) {
				this->patch_colors_[i] = std::make_shared<std::vector<common::ColorYUV>>();
				/* Kdtree for nearest neighbors searching */
				pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
				kdtree.setInputCloud(this->source_patches_[i].cloud);
				/* k-NN search */
				for (auto p : *(this->fitting_cloud_)) {
					std::vector<int>   idx(this->params_->patch.interpolation_num);
					std::vector<float> dis(this->params_->patch.interpolation_num);
					kdtree.nearestKSearch(p, this->params_->patch.interpolation_num, idx, dis);
					this->patch_colors_[i]->emplace_back(this->Interpolation(i, idx, dis));
				}
			}

			/* Set geometry for RAHTOctree */
			this->tree_.SetSourceCloud(this->fitting_cloud_);
			this->tree_.MakeTree();

			/* Get geometry */
			this->results_.front().geometry = this->tree_.GetOctree();

			for (int i = 0; i < this->patch_colors_.size(); ++i) {
				/* QP */
				size_t QP = i == 0 ? this->params_->slice.qp_i : this->params_->slice.qp_p;

				/* Do RAHT */
				this->tree_.SetSourceColors(this->patch_colors_[i]);
				this->tree_.RAHT();
				auto RAHT_data         = this->tree_.GetRAHTResult();
				auto RAHT_quant_result = std::make_shared<std::vector<common::FIX_DATA_INT>>(RAHT_data->size() * 3);

				/* Quantization */
				for (int j = 0; j < RAHT_data->size(); ++j) {
					RAHT_quant_result->at(j)                         = static_cast<common::FIX_DATA_INT>(std::ceil(RAHT_data->at(j).y / QP));
					RAHT_quant_result->at(j + RAHT_data->size())     = static_cast<common::FIX_DATA_INT>(std::ceil(RAHT_data->at(j).u / QP));
					RAHT_quant_result->at(j + 2 * RAHT_data->size()) = static_cast<common::FIX_DATA_INT>(std::ceil(RAHT_data->at(j).v / QP));
				}

				/* RLGR encoding */
				common::RLGREncoder enc;
				enc.Encode(RAHT_quant_result);
				this->results_[i].color = enc.GetResult();
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<common::Slice> GoPEncoding::GetResults() const {
		return this->results_;
	}
}  // namespace patch
}  // namespace vvc
