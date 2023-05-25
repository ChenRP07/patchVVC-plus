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

	std::vector<std::vector<GoP>> PVVCDeformation::GetResults() {
		return this->gops_;
	}

	void PVVCDeformation::Task() {
		while (true) {
			/* Fetch data index from task queue */
			int data_idx{-1};
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				data_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			/* Invalid index, all tasks have been done, exit */
			if (data_idx == -1) {
				break;
			}

			int patch_loc{-1};

			/* Find which patch is the target, i.e., index is equal to data_idx */
			for (int i = 0; i < this->patches_->at(this->current_frame_idx_).size(); ++i) {
				if (this->patches_->at(this->current_frame_idx_)[i].index == data_idx) {
					patch_loc = i;
				}
			}

			/* Cannot find this patch, something should be wrong */
			if (patch_loc == -1) {
				printf("\033[%dmUnknown error in PVVCDeformation::Task::83.\033[0m\n", common::RED);
				break;
			}

			/* Find this patch */
			auto& patch = this->patches_->at(this->current_frame_idx_)[patch_loc];
			int patch_idx = patch.index;

			/* Can this patch be able to be added into GoP */
			bool expand_gop_or_not = false;
			/* This patch should be simple_patch and AddPatch should return true */
			if (patch.type == common::PATCH_TYPE::SIMPLE_PATCH) {
				expand_gop_or_not = this->handler_[patch_idx].AddPatch(patch);
			}

			/* Can not expand GoP, should get fitted GoP */
			if (!expand_gop_or_not) {
				/* This handler has data */
				if (this->handler_data_[patch_idx]) {
					/* Get data from handler and add GoP into gops_ */
					GoP gop;
					gop.cloud = this->handler_[patch_idx].GetFittingCloud();
					gop.patches = this->handler_[patch_idx].GetSourcePatches();
					auto stat = this->handler_[patch_idx].GetStat();
					gop.start = gop.patches.front().timestamp;
					gop.end = gop.patches.back().timestamp;
					this->gops_[patch_idx].emplace_back(gop);

					/* Output information */
					size_t max_size{}, min_size{INT_MAX};
					for (auto& i : gop.patches) {
						max_size = std::max(max_size, i.size());
						min_size = std::min(min_size, i.size());
					}

					boost::format fmt{"\033[%1%m-------------------------------------------------------------------\n"
					                  "Generate GoP \033[0m#%3% \033[%1%mfrom \033[0m%4% \033[%1%mto \033[0m%5%\n"
					                  "\t\033[%2%mAverage mse    : \033[0m%6$.2f\n"
					                  "\t\033[%2%mAverage iter   : \033[0m%7$.2f\n"
					                  "\t\033[%2%mGoP cloud size : \033[0m%8% / %9% / %10%\n"
					                  "\033[%1%m-------------------------------------------------------------------\033[0m\n"};
					fmt % common::AZURE % common::BLUE % patch_idx % gop.start % gop.end % stat.first % stat.second % gop.cloud->size() % max_size % min_size;
					this->log_mutex_.lock();
					std::cout << fmt;
					this->log_mutex_.unlock();
				}
				/* Clear old data */
				this->handler_[patch_idx].Clear();
				this->handler_data_[patch_idx] = false;
				/* Set a new GoP start */
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
			this->clock_.SetTimeBegin();
			/* For each frame */
			for (; this->current_frame_idx_ < this->patches_->size();) {
				/* Max patch index of current frame */
				int max_idx{};
				for (auto& i : this->patches_->at(this->current_frame_idx_)) {
					max_idx = std::max(i.index, max_idx);
				}
				max_idx++;
				/* Now patch index is lower, add new patch */
				while (this->gops_.size() < max_idx) {
					this->gops_.emplace_back();
				}
				while (this->handler_.size() < max_idx) {
					this->handler_.emplace_back();
					this->handler_.back().SetParams(this->params_);
					this->handler_data_.emplace_back(false);
				}

				/* Threads pool */
				this->threads_.resize(this->params_->thread_num);

				/* Initialize task queue */
				while (!this->task_queue_.empty()) {
					this->task_queue_.pop();
				}
				for (auto& i : this->patches_->at(this->current_frame_idx_)) {
					this->task_queue_.push(i.index);
				}

				/* Launch threads to deform */
				for (auto& t : this->threads_) {
					t = std::thread(&PVVCDeformation::Task, this);
				}
				for (auto& i : this->threads_) {
					i.join();
				}
				current_frame_idx_++;
			}

			for (int i = 0; i < this->handler_.size(); ++i) {
				if (this->handler_data_[i]) {
					GoP gop;
					gop.cloud = this->handler_[i].GetFittingCloud();
					gop.patches = this->handler_[i].GetSourcePatches();
					auto stat = this->handler_[i].GetStat();
					gop.start = gop.patches.front().timestamp;
					gop.end = gop.patches.back().timestamp;
					this->gops_[i].emplace_back(gop);

					/* Output information */
					size_t max_size{}, min_size{INT_MAX};
					for (auto& j : gop.patches) {
						max_size = std::max(max_size, j.size());
						min_size = std::min(min_size, j.size());
					}

					boost::format fmt{"\033[%1%m-------------------------------------------------------------------\n"
					                  "Generate GoP \033[0m#%3% \033[%1%mfrom \033[0m%4% \033[%1%mto \033[0m%5%\n"
					                  "\t\033[%2%mAverage mse    : \033[0m%6$.2f\n"
					                  "\t\033[%2%mAverage iter   : \033[0m%7$.2f\n"
					                  "\t\033[%2%mGoP cloud size : \033[0m%8% / %9% / %10%\n"
					                  "\033[%1%m-------------------------------------------------------------------\033[0m\n"};
					fmt % common::AZURE % common::BLUE % i % gop.start % gop.end % stat.first % stat.second % gop.cloud->size() % max_size % min_size;
					std::cout << fmt;
				}
				/* Clear old data */
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
			/* Log start */
			boost::format fmt_0{"\033[%1%mSave GoPs ......\033[0m\n"};
			fmt_0 % common::AZURE;
			std::cout << fmt_0;

			/* Root dir */
			std::string dirs = this->params_->io.deform_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}
			dirs += "gops";

			/* Check status */
			std::ofstream outfile;
			struct stat info;
			int a;
			/* Root dir already exists, delete it */
			if (stat(dirs.c_str(), &info) == 0) {
				std::string s0 = "rm -rf " + dirs;
				a = system(s0.c_str());
				boost::format fmt_1{"\t%1% \033[%2%mexists, delete it.\n\033[0m"};
				fmt_1 % dirs % common::YELLOW;
				std::cout << fmt_1;
			}

			/* Create root dir */
			std::string s1 = "mkdir " + dirs;
			a = system(s1.c_str());
			boost::format fmt_2{"\t\033[%1%mCreate directory \033[0m%2%\n"};
			fmt_2 % common::BLUE % dirs;
			std::cout << fmt_2;

			/* Write sequence name and patch num to file */
			outfile.open(dirs + "/.config");
			outfile << this->params_->io.sequence_name << '\n' << this->gops_.size() << '\n';
			outfile.close();

			/* For each patch */
			for (int k = 0; k < this->gops_.size(); ++k) {
				/* Patch dir Root/seq_%04d */
				boost::format patch_dirs_fmt{"%s/%s_patch_%04d"};
				patch_dirs_fmt % dirs % this->params_->io.sequence_name % k;
				std::string patch_dirs = patch_dirs_fmt.str();

				/* Create this dir */
				std::string ss = "mkdir " + patch_dirs;
				a = system(ss.c_str());

				/* Write how many GoP this patch have */
				outfile.open(patch_dirs + "/.config");
				outfile << this->gops_[k].size() << '\n';
				outfile.close();

				for (int i = 0; i < this->gops_[k].size(); ++i) {
					/* GoP dir Root/seq_%04d/seq_patch_%04d_gop_%04d */
					boost::format gop_dirs_fmt{"%s/%s_patch_%04d_gop_%04d"};
					gop_dirs_fmt % patch_dirs % this->params_->io.sequence_name % k % i;
					std::string gop_dirs = gop_dirs_fmt.str();

					/* Create this dir */
					std::string s2 = "mkdir " + gop_dirs;
					a = system(s2.c_str());

					/* Write start timestamp and end timestamp to file */
					outfile.open(gop_dirs + "/.config");
					outfile << this->gops_[k][i].start << '\n' << this->gops_[k][i].end << '\n';
					outfile.close();

					/* Save fitting cloud */
					io::SaveColorPlyFile(gop_dirs + "/fitting_cloud.ply", this->gops_[k][i].cloud);

					for (int j = 0; j < this->gops_[k][i].patches.size(); ++j) {
						boost::format patch_fmt{"%s/%s_patch_%04d_gop_%04d_time_%04d.patch"};
						patch_fmt % gop_dirs % this->params_->io.sequence_name % k % i % this->gops_[k][i].patches[j].timestamp;
						std::string patch_name = patch_fmt.str();
						io::SavePatch(this->gops_[k][i].patches[j], patch_name);
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

			/* Log start */
			boost::format fmt_begin{"\033[%1%mLoad patches ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			/* Root dir */
			std::string dirs = this->params_->io.segment_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}

			dirs += "segment";

			/* Check status */
			std::ifstream infile;
			struct stat info;

			/* Root dir not exist */
			if (stat(dirs.c_str(), &info) != 0) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			/* Read sequence name and total frames number from file */
			infile.open(dirs + "/.config");
			std::string seq_name;
			int frame_cnt;
			infile >> seq_name >> frame_cnt;
			infile.close();

			/* Init container */
			this->patches_ = std::make_shared<std::vector<std::vector<common::Patch>>>(frame_cnt);

			/* Load frames */
			for (int i = 0; i < frame_cnt; ++i) {
				/* Frame dir */
				boost::format sub_dirs_fmt{"%s/%s_%04d"};
				sub_dirs_fmt % dirs % seq_name % i;
				std::string sub_dirs = sub_dirs_fmt.str();
				int patch_cnt;

				/* Read patch number from file */
				infile.open(sub_dirs + "/.config");
				infile >> patch_cnt;
				infile.close();
				/* Init container */
				this->patches_->at(i).resize(patch_cnt);
				/* Load each patches */
				for (int j = 0; j < patch_cnt; j++) {
					boost::format name_fmt{"%s/%s_time_%04d_patch_%04d.patch"};
					name_fmt % sub_dirs % seq_name % i % j;
					std::string name = name_fmt.str();
					io::LoadPatch(this->patches_->at(i)[j], name);
				}
			}
			printf("\033[%dmLoad patches finished.\033[0m\n", common::AZURE);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace codec
}  // namespace vvc
