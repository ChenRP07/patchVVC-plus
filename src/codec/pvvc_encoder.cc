/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/27 16:30
 * Last Modified : 2023/04/27 16:30
 *
 */

#include "codec/pvvc_encoder.h"

namespace vvc {
namespace codec {
	void PVVCCompensation::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCCompensation::LoadFrames() {
		try {
			/* Check valid parameters */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			/* Total frames */
			if (this->params_->frames <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			/* Force key frame interval */
			if (this->params_->max_keyframe <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			/* Load frames */
			this->frames_.resize(this->params_->frames);
			std::cout << __B_PURPLET__(Start loading frames......) << '\n';
			for (int i = 0; i < this->params_->frames; ++i) {
				int         timestamp      = this->params_->start_timestamp + i * this->params_->time_interval;
				auto        fmt            = boost::format{this->params_->io.source_file} % index;
				std::string file_name      = fmt.str();
				this->frames_[i].cloud     = io::LoadColorPlyFile(file_name);
				this->frames_[i].timestamp = timestamp;
				this->frames_[i].type      = i % this->params_->max_keyframe == 0 ? RAWFRAMETYPE::FORCE_KEY_FRAME : RAWFRAMETYPE::PREDICTIVE_FRAME;
				char type_name{'N'};
				if (this->frames_[i].type == RAWFRAMETYPE::FORCE_KEY_FRAME) {
					type_name = 'K';
				}
				if (this->frames_[i].type == RAWFRAMETYPE::PREDICTIVE_FRAME) {
					type_name = 'P';
				}
				// clang-format off
				std::cout << __B_GREENT__(Load frame) << ' ' << file_name << ", " 
                          << __B_GREENT__(type) << ' ' << type_name << ", " 
                          << __B_GREENT__(size) << ' ' << this->frames_[i].cloud->size()
				          << '\n';
				// clang-format on
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCEncoder::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCEncoder::LoadFrame() {
		try {
			if (this->params_->frames <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			if (this->params_->max_keyframe <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			this->frames_.resize(this->params_->frames);
			this->patches_.resize(this->params_->frames);

			for (int i = 0; i < this->frames_.size(); ++i) {
				int index = this->params_->start_timestamp + i * this->params_->time_interval;
				if (!(i % this->params_->max_keyframe)) {
					if (this->params_->check_point & 0x01) {
						boost::format fmt(this->params_->io.source_file);
						fmt % index;
						std::string file_name = fmt.str();
						std::cout << __B_AZURET__(Loading key frame #) << index << ", from ";
					}
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCEncoder::Encode() {
		try {
			/* Check parameters and data */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->frames_.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			/* Initilize, segmentation */
			this->patches_.resize(this->frames_.size());
			for (int i = 0; i < this->frames_.size(); ++i) {
				/* Force Intra-Frame */
				if (i % this->params_->max_keyframe == 0) {
					/* Load check points */
					if (this->params_->check_point & 0x02) {
						// io::PatchesLoader loader;
						continue;
					}
					/* Dense based segmentation */
					segment::DenseSegment seg;
					seg.SetParams(this->params_);
					seg.SetTimeStamp(this->params_->start_timestamp + this->params_->time_interval * i);
					// seg.SetSourcePointCloud(this->frames_[i].cloud);
					this->patches_[i] = seg.GetResultPatches();
				}
				else {
					/* Load check points */
					if (this->params_->check_point & 0x04) {
						// io::PatchesLoader loader;
						continue;
					}
					/* Parallel icp, segment predictive frames */
					registration::ParallelICP p_icp;
					p_icp.SetParams(this->params_);
					p_icp.SetSourceClouds(this->patches_[i / this->params_->max_keyframe]);
					// p_icp.SetSourceClouds(this->patches_[i / this->params_->max_keyframe]);
					// p_icp.SetTargetCloud(this->frames_[i].cloud);
					this->patches_[i] = p_icp.GetResultClouds();
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace codec
}  // namespace vvc
