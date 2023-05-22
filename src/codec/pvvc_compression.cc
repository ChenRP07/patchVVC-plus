/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/22 09:45
 * Last Modified : 2023/05/22 09:45
 *
 */

#include "codec/pvvc_encoder.h"

namespace vvc {
namespace codec {
	PVVCCompression::PVVCCompression() : params_{}, clock_{}, patches_{}, results_{}, handler_{}, task_queue_{} {}

	void PVVCCompression::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCCompression::SetGoPs(std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>> _patches) {
		this->patches_ = _patches;
	}

	std::vector<std::vector<common::Slice>> PVVCCompression::GetResults() {
		return this->results_;
	}

	void PVVCCompression::Task(int k) {
		while (true) {
			int gop_idx = -1;
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				gop_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			if (gop_idx == -1) {
				break;
			}
			auto& gops = this->patches_[k]->at(gop_idx);
			for (int i = 0; i < gops.size(); ++i) {
				int start_time          = gops[i].start;
				this->handler_[gop_idx] = patch::GoPEncoding();
				this->handler_[gop_idx].SetParams(this->params_);
				this->handler_[gop_idx].SetFittingCloud(gops[i].cloud);
				this->handler_[gop_idx].SetSourcePatches(gops[i].patches);
				this->handler_[gop_idx].Encode();
				auto res = this->handler_[gop_idx].GetResults();
				for (int t = 0; t < res.size(); t++) {
					this->results_[t + start_time][gop_idx] = res[t];
				}
			}
		}
	}

	void PVVCCompression::Compression() {
		try {
			/* Check parameters and data */
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->patches_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			boost::format fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
			                    "Start compression ......\033[0m\n"};
			fmt_0 % common::AZURE;
			std::cout << fmt_0;
			this->clock_.SetTimeBegin();
			/* Initialize results */
			this->results_.resize(this->params_->frames);
			for (int i = 0; i < this->results_.size(); ++i) {
				this->results_[i].resize(this->patches_[i / this->params_->max_keyframe]->size());
			}

			for (int i = 0; i < this->patches_.size(); ++i) {
				/* Reset handler */
				this->handler_.clear();
				this->handler_.resize(this->patches_[i]->size());
				/* Reset threads */
				this->threads_.resize(this->params_->thread_num);
				for (int j = 0; j < this->patches_[i]->size(); ++j) {
					this->task_queue_.push(j);
				}

				for (auto& t : this->threads_) {
					t = std::thread(&PVVCCompression::Task, this, i);
				}

				for (auto& t : this->threads_) {
					t.join();
				}
			}
			this->clock_.SetTimeEnd();
			boost::format fmt_1{"\033[%1%m-------------------------------------------------------------------\n"
			                    "Compression finished\n"
			                    "\t\033[%2%m Cost \033[0m%3$.2fs / %4$.2fms\n"};
			fmt_1 % common::AZURE % common::BLUE % this->clock_.GetTimeS() % this->clock_.GetTimeMs();
			std::cout << fmt_1;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCCompression::SaveSlices() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->results_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			boost::format fmt_begin{"\033[%1%mSave slices ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			std::string dirs = this->params_->io.result_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}

			dirs += "slice";
			struct stat info;
			int         a;

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

			std::ofstream outfile;
			outfile.open(dirs + "/.config");
			outfile << this->params_->io.sequence_name << '\n' << this->results_.size();
			outfile.close();

			int total_cnt{};

			for (int i = 0; i < this->results_.size(); ++i) {
				boost::format fmt_3{"%s/%s_%04d"};
				fmt_3 % dirs % this->params_->io.sequence_name % i;
				std::string sub_dirs = fmt_3.str();

				std::string s2 = "mkdir " + sub_dirs;

				a = system(s2.c_str());

				outfile.open(sub_dirs + "/.config");
				outfile << this->results_[i].size();
				outfile.close();

				int frame_cnt{};
				for (int j = 0; j < this->results_[i].size(); ++j) {
					boost::format fmt_4{"%s/%s_time_%04d_slice_%04d.slice"};
					fmt_4 % sub_dirs % this->params_->io.sequence_name % i % j;

					std::string name = fmt_4.str();
					frame_cnt += io::SaveSlice(this->results_[i][j], name);
				}

				boost::format fmt_frame{"\t\033[%1%mSave slices of frame #\033[0m%2$04d, \033[%1%msize \033[0m%3$.2fKB\n"};
				fmt_frame % common::BLUE % i % (frame_cnt / 1024.0f);
				std::cout << fmt_frame;

				total_cnt += frame_cnt;
			}

			boost::format fmt_total{"\033[%1%mSave all slices of all frames, total size \033[0m%2$.2fMB %3$.2fMbps\n"};

			float bit_rate = total_cnt * 8.0f * 30.0f / this->results_.size() / 1000000.0f;
			fmt_total % common::AZURE % (total_cnt / 1024.0f / 1024.0f) % bit_rate;

			std::cout << fmt_total;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCCompression::LoadGoPs() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			boost::format fmt_begin{"\033[%1%mLoad gops ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			std::string dirs = this->params_->io.deform_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}
			dirs += "gops";

			std::ifstream infile;

			std::string seq_name;
			int         key_num;

			infile.open(dirs + "/.config");
			infile >> seq_name >> key_num;
			infile.close();

			this->patches_.resize(key_num, nullptr);

			for (int i = 0; i < key_num; ++i) {
				boost::format key_dir_fmt{"%s/%s_%04d"};
				key_dir_fmt % dirs % seq_name % i;
				std::string key_dir = key_dir_fmt.str();

				int patch_cnt{};
				infile.open(key_dir + "/.config");
				infile >> patch_cnt;
				infile.close();

				this->patches_[i] = std::make_shared<std::vector<std::vector<GoP>>>(patch_cnt);
				for (int p = 0; p < patch_cnt; ++p) {
					boost::format patch_dir_fmt{"%s/%s_patch_%04d"};
					patch_dir_fmt % key_dir % seq_name % p;
					std::string patch_dir = patch_dir_fmt.str();

					int gop_cnt{};
					infile.open(patch_dir + "/.config");
					infile >> gop_cnt;
					infile.close();
					auto& gops = this->patches_[i]->at(p);
					gops.resize(gop_cnt);

					for (int g = 0; g < gop_cnt; ++g) {
						boost::format gop_dir_fmt{"%s/%s_patch_%04d_gop_%04d"};
						gop_dir_fmt % patch_dir % seq_name % p % g;
						std::string gop_dir = gop_dir_fmt.str();

						int start{}, end{};
						infile.open(gop_dir + "/.config");
						infile >> start >> end;
						infile.close();

						gops[g].start = start;
						gops[g].end   = end;

						gops[g].cloud = io::LoadColorPlyFile(gop_dir + "/fitting_cloud.ply");

						gops[g].patches.resize(end - start + 1);

						for (int t = start; t <= end; ++t) {
							boost::format file_fmt{"%s/%s_patch_%04d_gop_%04d_time_%04d.patch"};
							file_fmt % gop_dir % seq_name % p % g % t;
							std::string file_name = file_fmt.str();
							io::LoadPatch(gops[g].patches[t - start], file_name);
						}
					}
				}
			}
			boost::format fmt_end{"\033[%1%mLoad gops finished.\033[0m\n"};
			fmt_end % common::AZURE;
			std::cout << fmt_end;
		}
		catch (const common::Exception&) {
		}
	}

	void BuildFrames(common::PVVCParam_t::Ptr _param) {
		std::string dirs = _param->io.result_file;
		if (dirs.back() != '/') {
			dirs += '/';
		}
		std::string frame_dir = dirs + "frame";
		dirs += "slice";

		std::ifstream infile;

		std::string seq_name;
		int         frame_cnt{};

		infile.open(dirs + "/.config");
		infile >> seq_name >> frame_cnt;
		infile.close();

		struct stat info;
		if (stat(frame_dir.c_str(), &info) == 0) {
			std::string s = "rm -rf " + frame_dir;
			int         a = system(s.c_str());
		}

		std::string ss = "mkdir " + frame_dir;
		int         aa = system(ss.c_str());

		for (int f = 0; f < frame_cnt; ++f) {
			boost::format sub_dir_fmt{"%s/%s_%04d"};
			sub_dir_fmt % dirs % seq_name % f;
			boost::format f_fmt{"%s/%s_%d.frame"};
			f_fmt % frame_dir % seq_name % f;
			std::string           frame   = f_fmt.str();
			std::string           sub_dir = sub_dir_fmt.str();
			std::filesystem::path dir_path{sub_dir};
			io::ChangeSliceToFrame(dir_path, frame);
            std::cout << "Save frame " << frame << '\n';
		}
	}
}  // namespace codec
}  // namespace vvc
