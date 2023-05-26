/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/25 14:58
 * Last Modified : 2023/05/25 14:58
 *
 */

#include "registration/registration.h"

namespace vvc {
namespace registration {

	void PatchesRegistration::SetParams(common::PVVCParam_t::Ptr _params) {
		try {
			if (!_params) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			this->params_ = _params;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PatchesRegistration::SetSourcePatches(std::vector<common::Patch> _patches) {
		this->source_patches_ = _patches;
	}

	void PatchesRegistration::SetTargetPatches(std::vector<common::Patch> _patches) {
		this->target_patches_ = _patches;
	}

	void PatchesRegistration::Task() {
		while (true) {
			int task_idx = -1;
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				task_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			if (task_idx == -1) {
				break;
			}

			int source_idx{-1}, target_idx{-1};
			for (int i = 0; i < this->source_patches_.size(); ++i) {
				if (this->source_patches_[i].index == task_idx) {
					source_idx = i;
				}
			}

			for (int i = 0; i < this->target_patches_.size(); ++i) {
				if (this->target_patches_[i].index == task_idx) {
					target_idx = i;
				}
			}
			if (source_idx == -1 || target_idx == -1) {
				continue;
			}

			if (this->target_patches_[target_idx].size() <= 10) {
                this->mses_[target_idx] = -1.0f;
				continue;
			}
			
            if (this->source_patches_[source_idx].size() <= 10) {
                this->mses_[target_idx] = -1.0f;
				continue;
			}
			ICPBase::Ptr icp(new ICP());
			icp->SetParams(this->params_);
			icp->SetSourceCloud(this->source_patches_[source_idx].cloud);
			icp->SetTargetCloud(this->target_patches_[target_idx].cloud);
			icp->Align();
			if (icp->Converged()) {
				this->mses_[target_idx] = icp->CloudMSE();
			}
			else {
				this->mses_[target_idx] = -1.0f;
			}
		}
	}

	void PatchesRegistration::Align() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (this->source_patches_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			if (this->target_patches_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			// if (this->target_patches_.size() != this->source_patches_.size()) {
			// 	throw __EXCEPT__(UNMATCHED_PATCH_SIZE);
			// }
			//
			this->mses_.resize(this->source_patches_.size(), -1.0f);

			for (int i = 0; i < this->source_patches_.size(); ++i) {
				this->task_queue_.push(this->source_patches_[i].index);
			}

			this->threads_.resize(this->params_->thread_num);

			for (auto& i : this->threads_) {
				i = std::thread(&PatchesRegistration::Task, this);
			}

			for (auto& i : this->threads_) {
				i.join();
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<float> PatchesRegistration::GetMSEs() {
		return this->mses_;
	}
}  // namespace registration
}  // namespace vvc
