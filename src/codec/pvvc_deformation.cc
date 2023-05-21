/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/21 16:01
 * Last Modified : 2023/05/21 16:01
 *
 */

#include "codec/pvvc_encoder.h"

namespace vvc {
namespace codec {
	PVVCDeformation::PVVCDeformation() : params_{}, clock_{}, patches_{}, gops_{}, handler_{}, handler_data_{}, task_queue_{}, current_frame_idx_{} {}

	void PVVCDeformation::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCDeformation::SetPatches(std::shared_ptr<std::vector<std::vector<common::Patch>>> _patches) {
		try {
			/* Check parameters */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->params_->frames <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			/* Check input patches */
			if (!_patches || _patches->size() != this->params_->frames) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			this->patches_ = _patches;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>> PVVCDeformation::GetResults() {
		return this->results_;
	}

	void PVVCDeformation::Task() {
		while (true) {
			int data_idx{-1};
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				data_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			if (data_idx == -1) {
				break;
			}
			auto& patch     = this->patches_->at(this->current_frame_idx_)[data_idx];
			int   patch_idx = patch.index;
			bool  flag      = this->handler_[patch_idx].AddPatch(patch);
			/* Cannot expand GoP */
			if (!flag) {
				int start_now = this->gops_->at(patch_idx).empty() ? 0 : this->gops_->at(patch_idx).back().end + 1;
				/* Add a GoP */
				this->gops_->at(patch_idx).emplace_back();
				/* Set data */
				this->gops_->at(patch_idx).back().cloud   = this->handler_[patch_idx].GetFittingCloud();
				this->gops_->at(patch_idx).back().patches = this->handler_[patch_idx].GetSourcePatches();

				auto stat = this->handler_[patch_idx].GetStat();

				int end_now                             = start_now + this->gops_->at(patch_idx).back().patches.size() - 1;
				this->gops_->at(patch_idx).back().start = start_now;
				this->gops_->at(patch_idx).back().end   = end_now;

				boost::format fmt{"\033[%1%m-------------------------------------------------------------------\n"
				                  "Generate GoP \033[0m#%3% \033[%1%mfrom \033[0m%4% \033[%1%mto \033[0m%5%\n"
				                  "\t\033[%2%mAverage mse  : \033[0m%6$.2f\n"
				                  "\t\033[%2%mAverage iter : \033[0m%7$.2f\n"
				                  "\033[%1%m-------------------------------------------------------------------\033[0m\n"};
				fmt % common::AZURE % common::BLUE % patch_idx % start_now % end_now % stat.first % stat.second;
				this->log_mutex_.lock();
				std::cout << fmt;
				this->log_mutex_.unlock();
				/* Clear old data */
				this->handler_[patch_idx].Clear();
				this->handler_data_[patch_idx] = false;
				this->handler_[patch_idx].AddPatch(patch);
				this->handler_data_[patch_idx] = true;
			}
			else {
				this->handler_data_[patch_idx] = true;
			}
		}
	}
	void PVVCDeformation::Deformation() {
		try {
			/* Check preset */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (!this->patches_) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			for (; this->current_frame_idx_ < this->patches_->size();) {
				this->clock_.SetTimeBegin();
				int max_idx{};
				for (auto& i : this->patches_->at(this->current_frame_idx_)) {
					max_idx = std::max(i.index, max_idx);
				}
				this->handler_.clear();
				this->handler_data_.clear();
				/* Create and initialize handlers */
				this->handler_.resize(max_idx + 1);
				this->handler_data_.resize(max_idx + 1, false);
				for (auto& i : this->handler_) {
					i.SetParams(this->params_);
				}

				this->threads_.resize(this->params_->thread_num);

				/* Malloc space for result */
				this->gops_ = std::make_shared<std::vector<std::vector<GoP>>>(max_idx + 1);

				while (current_frame_idx_ < this->patches_->size()) {
					for (int i = 0; i < this->patches_->at(current_frame_idx_).size(); ++i) {
						this->task_queue_.push(i);
					}
					for (auto& t : this->threads_) {
						t = std::thread(&PVVCDeformation::Task, this);
					}
					for (auto& i : this->threads_) {
						i.join();
					}
					current_frame_idx_++;
					if (current_frame_idx_ % this->params_->max_keyframe == 0) {
						break;
					}
				}

				for (int i = 0; i < this->handler_.size(); ++i) {
					if (this->handler_data_[i] == false) {
						continue;
					}
					int start_now = this->gops_->at(i).empty() ? 0 : this->gops_->at(i).back().end + 1;
					/* Add a GoP */
					this->gops_->at(i).emplace_back();
					/* Set data */
					this->gops_->at(i).back().cloud   = this->handler_[i].GetFittingCloud();
					this->gops_->at(i).back().patches = this->handler_[i].GetSourcePatches();

					auto stat = this->handler_[i].GetStat();

					int end_now = start_now + this->gops_->at(i).back().patches.size() - 1;

					this->gops_->at(i).back().start = start_now;
					this->gops_->at(i).back().end   = end_now;

					boost::format fmt{"\033[%1%m-------------------------------------------------------------------\n"
					                  "Generate GoP \033[0m#%3% \033[%1%mfrom \033[0m%4% \033[%1%mto \033[0m%5%\n"
					                  "\t\033[%2%mAverage mse  : \033[0m%6$.2f\n"
					                  "\t\033[%2%mAverage iter : \033[0m%7$.2f\n"
					                  "\033[%1%m-------------------------------------------------------------------\033[0m\n"};
					fmt % common::AZURE % common::BLUE % i % start_now % end_now % stat.first % stat.second;
					std::cout << fmt;
					this->handler_[i].Clear();
					this->handler_data_[i] = false;
				}
				this->clock_.SetTimeEnd();
				boost::format fmt_end{"\033[%1%m-------------------------------------------------------------------\n"
				                      "\033[%2%mPatch deformation finished."
				                      "\t\033[%2%mCost \033[0m%3$.2fs\n"
				                      "\033[%1%m-------------------------------------------------------------------\n\033[0m"};
				fmt_end % common::AZURE % common::BLUE % this->clock_.GetTimeS();
				std::cout << fmt_end;
				this->results_.emplace_back(this->gops_);
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCDeformation::SaveDeformPatches() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (!this->gops_) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			boost::format fmt_0{"\033[%1%mSave GoPs ......\033[0m\n"};
			fmt_0 % common::AZURE;
			std::cout << fmt_0;
			std::string dirs = this->params_->io.deform_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}
			dirs += "gops";
			std::ofstream outfile;
			struct stat   info;
			int           a;
			if (stat(dirs.c_str(), &info) == 0) {
				std::string s0 = "rm -rf " + dirs;
				a              = system(s0.c_str());
				boost::format fmt_1{"\t%1% \033[%2%mexists, delete it.\n\033[0m"};
				fmt_1 % dirs % common::YELLOW;
				std::cout << fmt_1;
			}
			std::string s1 = "mkdir " + dirs;
			a              = system(s1.c_str());
			boost::format fmt_2{"\t\033[%1%mCreate directory \033[0m%2%\n"};
			fmt_2 % common::BLUE % dirs;
			std::cout << fmt_2;
			outfile.open(dirs + "/.config");
			outfile << this->params_->io.sequence_name << '\n' << this->results_.size();
			outfile.close();
			for (int k = 0; k < this->results_.size(); ++k) {
				boost::format key_dirs_fmt{"%s/%s_%04d"};
				key_dirs_fmt % dirs % this->params_->io.sequence_name % k;
				std::string key_dirs = key_dirs_fmt.str();
				std::string ss       = "mkdir " + key_dirs;
				a                    = system(ss.c_str());
				outfile.open(key_dirs + "/.config");
				outfile << this->results_[k]->size();
				outfile.close();
				for (int i = 0; i < this->results_[k]->size(); ++i) {
					boost::format sub_dirs_fmt{"%s/%s_patch_%04d"};
					sub_dirs_fmt % key_dirs % this->params_->io.sequence_name % i;
					std::string sub_dirs = sub_dirs_fmt.str();
					std::string s2       = "mkdir " + sub_dirs;
					a                    = system(s2.c_str());
					outfile.open(sub_dirs + "/.config");
					outfile << this->results_[k]->at(i).size();
					outfile.close();
					for (int j = 0; j < this->results_[k]->at(i).size(); ++j) {
						boost::format gop_dirs_fmt{"%s/%s_patch_%04d_gop_%04d"};
						gop_dirs_fmt % sub_dirs % this->params_->io.sequence_name % i % j;
						std::string gop_dirs = gop_dirs_fmt.str();
						std::string s3       = "mkdir " + gop_dirs;
						a                    = system(s3.c_str());
						io::SaveColorPlyFile(gop_dirs + "/fitting_cloud.ply", this->results_[k]->at(i)[j].cloud);
						outfile.open(gop_dirs + "/.config");
						outfile << this->results_[k]->at(i)[j].start << '\n' << this->results_[k]->at(i)[j].end;
						outfile.close();
						for (int idx = 0; idx < this->results_[k]->at(i)[j].patches.size(); idx++) {
							boost::format patch_fmt{"%s/%s_patch_%04d_gop_%04d_time_%04d.patch"};
							patch_fmt % gop_dirs % this->params_->io.sequence_name % i % j % (this->results_[k]->at(i)[j].start + idx);
							std::string patch_name = patch_fmt.str();
							io::SavePatch(this->results_[k]->at(i)[j].patches[idx], patch_name);
						}
					}
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCDeformation::LoadPatches() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			boost::format fmt_begin{"\033[%1%mLoad patches ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			std::string dirs = this->params_->io.segment_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}

            dirs += "segment";
			std::ifstream infile;
			struct stat   info;

			if (stat(dirs.c_str(), &info) != 0) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			infile.open(dirs + "/.config");
			std::string seq_name;
			int         frame_cnt;
			infile >> seq_name >> frame_cnt;
			infile.close();
			this->patches_ = std::make_shared<std::vector<std::vector<common::Patch>>>(frame_cnt);

			for (int i = 0; i < frame_cnt; ++i) {
				boost::format sub_dirs_fmt{"%s/%s_%04d"};
				sub_dirs_fmt % dirs % seq_name % i;
				std::string sub_dirs = sub_dirs_fmt.str();
				int         patch_cnt;
				infile.open(sub_dirs + "/.confg");
				infile >> patch_cnt;
				infile.close();
				this->patches_->at(i).resize(patch_cnt);
				for (int j = 0; j < patch_cnt; j++) {
					boost::format name_fmt{"%s/%s_time_%04d_patch_%04d.patch"};
					name_fmt % sub_dirs % seq_name % i % j;
					std::string name = name_fmt.str();
					io::LoadPatch(this->patches_->at(i)[j], name);
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
