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
	PVVCSegmentation::PVVCSegmentation() : clock_{}, frames_{}, current_frame_idx_{}, params_{} {
		this->patches_ = std::make_shared<std::vector<std::vector<common::Patch>>>();
	}

	void PVVCSegmentation::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCSegmentation::LoadFrames() {
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
			this->clock_.SetTimeBegin();
			this->frames_.resize(this->params_->frames);
			std::cout << __AZURET__(Start loading frames......) << '\n';
			for (int i = 0; i < this->params_->frames; ++i) {
				int timestamp = this->params_->start_timestamp + i * this->params_->time_interval;
				auto fmt = boost::format(this->params_->io.source_file) % timestamp;
				std::string file_name = fmt.str();
				this->frames_[i].cloud = io::LoadColorPlyFile(file_name);
				this->frames_[i].timestamp = timestamp;
				this->frames_[i].type = i % this->params_->max_keyframe == 0 ? RAWFRAMETYPE::FORCE_KEY_FRAME : RAWFRAMETYPE::PREDICTIVE_FRAME;
				char type_name{'N'};
				if (this->frames_[i].type == RAWFRAMETYPE::FORCE_KEY_FRAME) {
					type_name = 'K';
				}
				if (this->frames_[i].type == RAWFRAMETYPE::PREDICTIVE_FRAME) {
					type_name = 'P';
				}
				boost::format log_fmt{"\033[%1%mLoad frame \033[0m%2%, \033[%1%mtype \033[0m%3%, \033[%1%msize \033[0m%4%\n"};
				log_fmt % common::GREEN % file_name % type_name % this->frames_[i].cloud->size();
				std::cout << log_fmt;
			}
			this->clock_.SetTimeEnd();
			boost::format end_fmt{"\033[%1%mLoad \033[0m%2% \033[%1%mframes, cost \033[m%3$.2fs\n"};
			end_fmt % common::AZURE % this->params_->frames % this->clock_.GetTimeS();
			std::cout << end_fmt;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

#ifdef _PVVC_SEGMENT_230524_
	void PVVCSegmentation::Segmentation() {
		try {
			/* Check parameters valid */
			if (!this->params_) {
				throw __EXCEPT__(ERROR_OCCURED);
			}
			/* Check frames are loaded */
			if (frames_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			/* Segment each frame */
			this->patches_->resize(this->frames_.size());
			while (current_frame_idx_ < this->frames_.size()) {
				/* Start segmentation on K-Frame with Dense Segmentation */
				if (this->frames_[this->current_frame_idx_].type != RAWFRAMETYPE::FORCE_KEY_FRAME) {
					std::cout << __YELLOWT__([Warning]) << " dense segmentation for a non-K-Frame.\n";
				}

				boost::format fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
				                    "Start dense segmentation for frame \033[0m#%2% ......\n"};
				fmt_0 % common::AZURE % current_frame_idx_;
				std::cout << fmt_0;
				this->clock_.SetTimeBegin();

				/* Start segmentation */
				segment::DenseSegment frame_segment;
				frame_segment.SetParams(this->params_);
				frame_segment.SetSourcePointCloud(this->frames_[this->current_frame_idx_].cloud);
				frame_segment.SetTimeStamp(this->frames_[this->current_frame_idx_].timestamp);
				frame_segment.Segment();
				this->patches_->at(current_frame_idx_) = frame_segment.GetResultPatches();
				std::vector<int> patch_size;
				for (auto& i : this->patches_->at(current_frame_idx_)) {
					patch_size.emplace_back(i.size());
				}
				this->clock_.SetTimeEnd();

				/* Log */
				boost::format fmt_1{"\033[%1%mDense segmentation finished\n"
				                    "\t\033[%2%mTotal patches count : \033[0m%3%\n"
				                    "\t\033[%2%mAverage patch size  : \033[0m%4$.2f\n"
				                    "\t\033[%2%mMax/Min patch size  : \033[0m%5% / %6%\n"
				                    "\t\033[%2%mSegmentation  cost  : \033[0m%7$.2fs / %8$.2fms\n"
				                    "\033[%1%m-------------------------------------------------------------------\n\033[0m"};

				float avg = static_cast<float>(std::accumulate(patch_size.begin(), patch_size.end(), 0));
				avg /= static_cast<float>(patch_size.size());
				fmt_1 % common::AZURE % common::BLUE % patch_size.size() % avg % (*std::max_element(patch_size.begin(), patch_size.end())) % (*std::min_element(patch_size.begin(), patch_size.end())) %
				    this->clock_.GetTimeS() % this->clock_.GetTimeMs();
				std::cout << fmt_1;
				int reference_frame_idx = current_frame_idx_;
				/* Next frame */
				current_frame_idx_++;

				/* ICP segmentation until reach next K-Frame */
				for (; current_frame_idx_ < this->frames_.size(); current_frame_idx_++) {
					if (this->frames_[current_frame_idx_].type == RAWFRAMETYPE::FORCE_KEY_FRAME) {
						break;
					}

					boost::format icp_fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
					                        "Start ICP segmentation for frame \033[0m#%2% \033[%1%mon \033[0m%3% \033[%1%mthreads \033[0m......\n"};
					this->clock_.SetTimeBegin();
					icp_fmt_0 % common::AZURE % current_frame_idx_ % this->params_->thread_num;
					std::cout << icp_fmt_0;
					registration::ParallelICP picp_segment;
					picp_segment.SetParams(this->params_);
					picp_segment.SetSourcePatches(this->patches_->at(current_frame_idx_ - 1));
					picp_segment.SetTargetCloud(this->frames_[current_frame_idx_].cloud);
					picp_segment.Align();
					this->patches_->at(current_frame_idx_) = picp_segment.GetResultPatches();
					this->clock_.SetTimeEnd();
					auto icp_stat = picp_segment.GetStat();
					float max_stat{}, min_stat{FLT_MAX}, avg_stat{};
					int cnt_stat{};
					for (auto i : icp_stat) {
						if (i >= 0.0f) {
							cnt_stat++;
							avg_stat += i;
							max_stat = std::max(i, max_stat);
							min_stat = std::min(i, min_stat);
						}
					}
					avg_stat /= cnt_stat;
					float avg_psize{};
					int max_psize{}, min_psize{INT_MAX};
					for (auto& i : this->patches_->at(current_frame_idx_)) {
						avg_psize += static_cast<float>(i.size());
						max_psize = std::max(max_psize, static_cast<int>(i.size()));
						min_psize = std::min(min_psize, static_cast<int>(i.size()));
					}
					avg_psize /= this->patches_->at(current_frame_idx_).size();
					boost::format icp_fmt_1{"\033[%1%mICP segmentation finished\n"
					                        "\t\033[%2%mConverged patches  : \033[0m%3% / %4%\n"
					                        "\t\033[%2%mAverage patch mse  : \033[0m%5$.2f\n"
					                        "\t\033[%2%mMax/Min patch mse  : \033[0m%6$.2f / %7$.2f\n"
					                        "\t\033[%2%mAverage patch size : \033[0m%10$.2f\n"
					                        "\t\033[%2%mMax/Min patch size : \033[0m%11% / %12%\n"
					                        "\t\033[%2%mSegmentation cost : \033[0m%8$.2fs / %9$.2fms\n"
					                        "\033[%1%m-------------------------------------------------------------------\n\033[0m"};
					icp_fmt_1 % common::AZURE % common::BLUE % cnt_stat % icp_stat.size() % avg_stat % max_stat % min_stat % this->clock_.GetTimeS() % this->clock_.GetTimeMs() % avg_psize %
					    max_psize % min_psize;
					std::cout << icp_fmt_1;
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
#endif

#ifdef _PVVC_SEGMENT_230525_

	void PVVCSegmentation::Segmentation() {
		try {
			/* Check parameters valid */
			if (!this->params_) {
				throw __EXCEPT__(ERROR_OCCURED);
			}
			/* Check frames are loaded */
			if (frames_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			/* Segment each frame */
			this->patches_->resize(this->frames_.size());
			while (current_frame_idx_ < this->frames_.size()) {
				if (this->frames_[current_frame_idx_].type == RAWFRAMETYPE::FORCE_KEY_FRAME) {
					boost::format fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
					                    "Start dense segmentation for frame \033[0m#%2% ......\n"};
					fmt_0 % common::AZURE % current_frame_idx_;
					std::cout << fmt_0;
					this->clock_.SetTimeBegin();

					/* Start segmentation */
					segment::DenseSegment frame_segment;
					frame_segment.SetParams(this->params_);
					frame_segment.SetSourcePointCloud(this->frames_[this->current_frame_idx_].cloud);
					frame_segment.SetTimeStamp(this->frames_[this->current_frame_idx_].timestamp);
					frame_segment.Segment();
					this->patches_->at(current_frame_idx_) = frame_segment.GetResultPatches();
					std::vector<int> patch_size;
					for (auto& i : this->patches_->at(current_frame_idx_)) {
						patch_size.emplace_back(i.size());
					}
					this->clock_.SetTimeEnd();

					/* Log */
					boost::format fmt_1{"\033[%1%mDense segmentation finished\n"
					                    "\t\033[%2%mTotal patches count : \033[0m%3%\n"
					                    "\t\033[%2%mAverage patch size  : \033[0m%4$.2f\n"
					                    "\t\033[%2%mMax/Min patch size  : \033[0m%5% / %6%\n"
					                    "\t\033[%2%mSegmentation  cost  : \033[0m%7$.2fs\n"
					                    "\033[%1%m-------------------------------------------------------------------\n\033[0m"};

					float avg = static_cast<float>(std::accumulate(patch_size.begin(), patch_size.end(), 0));
					avg /= static_cast<float>(patch_size.size());
					fmt_1 % common::AZURE % common::BLUE % patch_size.size() % avg % (*std::max_element(patch_size.begin(), patch_size.end())) %
					    (*std::min_element(patch_size.begin(), patch_size.end())) % this->clock_.GetTimeS();
					std::cout << fmt_1;
					current_frame_idx_++;
				}
				else {
					/* TODO: avg mse change computing method, check avg mse or something else to decide this frame can be ref segment */
					boost::format fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
					                    "Start reference segmentation for frame \033[0m#%2% ......\n"};
					fmt_0 % common::AZURE % current_frame_idx_;
					std::cout << fmt_0;
					this->clock_.SetTimeBegin();
					segment::RefSegment frame_segment;
					frame_segment.SetParams(this->params_);
					frame_segment.SetSourcePointCloud(this->frames_[this->current_frame_idx_].cloud);
					frame_segment.SetTimeStamp(this->frames_[this->current_frame_idx_].timestamp);
					frame_segment.SetRefPatches(this->patches_->at(this->current_frame_idx_ - 1));
					frame_segment.Segment();
					this->patches_->at(current_frame_idx_) = frame_segment.GetResultPatches();

					std::vector<int> patch_size;
					for (auto& i : this->patches_->at(current_frame_idx_)) {
						if (i.size() > 1) {
							patch_size.emplace_back(i.size());
						}
					}
					this->clock_.SetTimeEnd();

					/* Log */
					boost::format fmt_1{"\033[%1%mReference segmentation finished\n"
					                    "\t\033[%2%mTotal patches count : \033[0m%3%\n"
					                    "\t\033[%2%mAverage patch size  : \033[0m%4$.2f\n"
					                    "\t\033[%2%mMax/Min patch size  : \033[0m%5% / %6%\n"
					                    "\t\033[%2%mSegmentation  cost  : \033[0m%7$.2fs\n"};

					float avg = static_cast<float>(std::accumulate(patch_size.begin(), patch_size.end(), 0));
					avg /= static_cast<float>(patch_size.size());
					fmt_1 % common::AZURE % common::BLUE % patch_size.size() % avg % (*std::max_element(patch_size.begin(), patch_size.end())) %
					    (*std::min_element(patch_size.begin(), patch_size.end())) % this->clock_.GetTimeS();
					std::cout << fmt_1;

					if (*std::max_element(patch_size.begin(), patch_size.end()) > this->params_->segment.num * 2) {
						printf("\t\033[%dmPatch is too large, resegment this frame as key frame!\033[0m\n", common::B_PURPLE);
						this->patches_->at(current_frame_idx_).clear();
						this->frames_[current_frame_idx_].type = RAWFRAMETYPE::FORCE_KEY_FRAME;
						continue;
					}
					this->clock_.SetTimeBegin();
					registration::PatchesRegistration check;
					check.SetParams(this->params_);
					check.SetSourcePatches(this->patches_->at(current_frame_idx_));
					check.SetTargetPatches(this->patches_->at(current_frame_idx_ - 1));
					check.Align();
					auto result = check.GetMSEs();

					int conv_cnt{};
					int total_point{};
					float avg_mse{}, max_mse{}, min_mse{FLT_MAX};
					for (int i = 0; i < result.size(); ++i) {
						if (result[i] >= 0.0f) {
							conv_cnt++;
							avg_mse += result[i] * this->patches_->at(current_frame_idx_)[i].size();
							total_point += this->patches_->at(current_frame_idx_)[i].size();
							max_mse = std::max(max_mse, result[i]);
							min_mse = std::min(min_mse, result[i]);
							this->patches_->at(current_frame_idx_)[i].type = common::PATCH_TYPE::SIMPLE_PATCH;
						}
						else {
							this->patches_->at(current_frame_idx_)[i].type = common::PATCH_TYPE::FORCE_KEY_PATCH;
						}
					}
					avg_mse /= total_point;

					this->clock_.SetTimeEnd();
					boost::format fmt_2{"\033[%1%mCheck with reference patches\n"
					                    "\t\033[%2%mConverged patches : \033[0m%3% / %4%\n"
					                    "\t\033[%2%mAverage patch mse : \033[0m%5$.2f\n"
					                    "\t\033[%2%mMax/Min patch mse : \033[0m%6$.2f / %7$.2f\n"
					                    "\t\033[%2%mCheck time cost   : \033[0m%8$.2fs\n"};
					fmt_2 % common::AZURE % common::BLUE % conv_cnt % result.size() % avg_mse % max_mse % min_mse % this->clock_.GetTimeS();
					std::cout << fmt_2;

					if (avg_mse > 10.0f) {
						printf("\t\033[%dmMse is too large, resegment this frame as key frame!\033[0m\n", common::B_PURPLE);
						this->patches_->at(current_frame_idx_).clear();
						this->frames_[current_frame_idx_].type = RAWFRAMETYPE::FORCE_KEY_FRAME;
						continue;
					}
					else {
						current_frame_idx_++;
					}
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
#endif

	std::shared_ptr<std::vector<std::vector<common::Patch>>> PVVCSegmentation::GetPatches() {
		return this->patches_;
	}

	void PVVCSegmentation::SavePatches() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (!this->patches_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			boost::format fmt_t{"\033[%1%mSave patches ......\033[0m\n"};
			fmt_t % common::AZURE;
			std::cout << fmt_t;
			std::string dirs = this->params_->io.segment_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}
			dirs += "segment";

			std::ofstream outfile;
			struct stat info;
			int a;

			if (stat(dirs.c_str(), &info) == 0) {
				std::string s0 = "rm -rf " + dirs;
				a = system(s0.c_str());
				boost::format fmt_1{"\t%1% \033[%2%mexists, delete it.\n\033[0m"};
				fmt_1 % dirs % common::YELLOW;
				std::cout << fmt_1;
			}
			std::string s1 = "mkdir " + dirs;
			a = system(s1.c_str());
			boost::format fmt_2{"\t\033[%1%mCreate directory \033[0m%2%\n"};
			fmt_2 % common::BLUE % dirs;
			std::cout << fmt_2;
			outfile.open(dirs + "/.config");
			outfile << this->params_->io.sequence_name << '\n' << this->patches_->size() << '\n';
			outfile.close();
			for (int i = 0; i < this->patches_->size(); ++i) {
				boost::format fmt_3{"%s/%s_%04d"};
				fmt_3 % dirs % this->params_->io.sequence_name % i;
				std::string sub_dirs = fmt_3.str();
				std::string s2 = "mkdir " + sub_dirs;
				a = system(s2.c_str());
				outfile.open(sub_dirs + "/.config");
				outfile << this->patches_->at(i).size() << '\n';
				outfile.close();
				for (int j = 0; j < this->patches_->at(i).size(); j++) {
					boost::format fmt_4{"%s/%s_time_%04d_patch_%04d.patch"};
					fmt_4 % sub_dirs % this->params_->io.sequence_name % i % j;
					std::string name = fmt_4.str();
					io::SavePatch(this->patches_->at(i)[j], name);
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
