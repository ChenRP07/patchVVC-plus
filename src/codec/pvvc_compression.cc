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
	PVVCCompression::PVVCCompression() : params_{}, clock_{}, gops_{}, results_{}, task_queue_{} {}

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

	void PVVCCompression::SetGoPs(std::vector<std::vector<GoP>> _gops) {
		this->gops_ = _gops;
	}

	std::vector<std::vector<common::Slice>> PVVCCompression::GetResults() {
		return this->results_;
	}

	void PVVCCompression::Task() {
		while (true) {
			int patch_idx = -1;
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				patch_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			if (patch_idx == -1) {
				break;
			}

			for (int i = 0; i < this->gops_[patch_idx].size(); ++i) {
				patch::GoPEncoding enc;
				enc.SetParams(this->params_);
				enc.SetFittingCloud(this->gops_[patch_idx][i].cloud);
				enc.SetSourcePatches(this->gops_[patch_idx][i].patches);
				enc.Encode();
				auto res = enc.GetResults();
				for (auto& p : res) {
					int frame_idx = (p.timestamp - this->params_->start_timestamp) / this->params_->time_interval;
					this->results_[frame_idx][p.index] = p;
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
			if (this->gops_.empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			boost::format fmt_0{"\033[%1%m-------------------------------------------------------------------\n"
			                    "Start compression ......\033[0m\n"};
			fmt_0 % common::AZURE;
			std::cout << fmt_0;
			this->clock_.SetTimeBegin();

			/* Split large size GoP */
			int start_back = this->gops_.size() - 1;

			for (int i = 0; i <= start_back; ++i) {
				for (int j = 0; j < this->gops_[i].size(); ++j) {
					bool add = false;
					if (this->gops_[i][j].cloud->size() > this->params_->segment.num * (1.0f + segment::NUM_THS)) {
						std::vector<GoP> temp;
						this->SplitGoP(this->gops_[i][j], temp);
						this->gops_[i][j] = temp[0];
						for (int t = 1; t < temp.size(); t++) {
							if (!add) {
								this->gops_.emplace_back();
								add = true;
							}
							for (auto& p : temp[t].patches) {
								p.index = this->gops_.size() - 1;
							}
							this->gops_.back().emplace_back(temp[t]);
						}
						boost::format split_fmt{"\t\033[%1%mSplit GoP \033[0m#%2% \033[%1%mfrom \033[0m%3% \033[%1%mto \033[0m%4%\033[%1%m, generate GoP \033[0m#%5%\n"};
						split_fmt % common::BLUE % i % this->gops_[i][j].start % this->gops_[i][j].end % (this->gops_.size() - 1);
						std::cout << split_fmt;
					}
				}
			}

			this->threads_.resize(this->params_->thread_num);
			this->results_.resize(this->params_->frames, std::vector<common::Slice>(this->gops_.size()));
			for (int i = 0; i < this->gops_.size(); ++i) {
				this->task_queue_.push(i);
			}

			for (auto& i : this->threads_) {
				i = std::thread(&PVVCCompression::Task, this);
			}

			for (auto& i : this->threads_) {
				i.join();
			}

			this->clock_.SetTimeEnd();
			boost::format fmt_1{"\033[%1%m-------------------------------------------------------------------\n"
			                    "Compression finished\n"
			                    "\t\033[%2%m Cost \033[0m%3$.2fs\n"};
			fmt_1 % common::AZURE % common::BLUE % this->clock_.GetTimeS();
			std::cout << fmt_1;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PVVCCompression::SplitCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _old, decltype(_old) _new_0, decltype(_old) _new_1) {
		float min_x{FLT_MAX}, min_y{FLT_MAX}, min_z{FLT_MAX};
		float max_x{FLT_TRUE_MIN}, max_y{FLT_TRUE_MIN}, max_z{FLT_TRUE_MIN};

		for (auto& p : *_old) {
			min_x = std::min(min_x, p.x);
			min_y = std::min(min_y, p.y);
			min_z = std::min(min_z, p.z);

			max_x = std::max(max_x, p.x);
			max_y = std::max(max_y, p.y);
			max_z = std::max(max_z, p.z);
		}

		char type{};
		float temp{};
		if (max_x - min_x >= max_y - min_y) {
			type = 'x';
			temp = max_x - min_x;
		}
		else {
			type = 'y';
			temp = max_y - min_y;
		}

		if (temp < max_z - min_z) {
			type = 'z';
		}

		float mid{};
		if (type == 'x')
			mid = (max_x + min_x) / 2.0f;
		else if (type == 'y')
			mid = (max_y + min_y) / 2.0f;
		else if (type == 'z')
			mid = (max_z + min_z) / 2.0f;

		for (auto& p : *_old) {
			if (type == 'x') {
				if (p.x < mid)
					_new_0->emplace_back(p);
				else
					_new_1->emplace_back(p);
			}
			else if (type == 'y') {
				if (p.y < mid)
					_new_0->emplace_back(p);
				else
					_new_1->emplace_back(p);
			}
			else if (type == 'z') {
				if (p.z < mid)
					_new_0->emplace_back(p);
				else
					_new_1->emplace_back(p);
			}
		}
	}

	void PVVCCompression::SplitGoP(GoP& _g, std::vector<GoP>& _res) {
		auto cmp = [](const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _x, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _y) -> bool { return _x->size() < _y->size(); };

		std::priority_queue<decltype(_g.cloud), std::vector<decltype(_g.cloud)>, decltype(cmp)> heap(cmp);

		heap.push(_g.cloud);

		while (heap.top()->size() < this->params_->segment.num * (1.0f + segment::NUM_THS)) {
			auto ptr = heap.top();
			heap.pop();
			decltype(_g.cloud) ptr_a, ptr_b;
			ptr_a.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			ptr_b.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			this->SplitCloud(ptr, ptr_a, ptr_b);
			heap.push(ptr_a);
			heap.push(ptr_b);
		}
		GoP t = _g;
		while (!heap.empty()) {
			auto ptr = heap.top();
			heap.pop();
			t.cloud = ptr;
			_res.emplace_back(t);
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

			/* log start */
			boost::format fmt_begin{"\033[%1%mSave slices ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			/* Root */
			std::string dirs = this->params_->io.result_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}

			dirs += "slice";

			/* Check status */
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

			/* Write seq_name and frames number to file */
			std::ofstream outfile;
			outfile.open(dirs + "/.config");
			outfile << this->params_->io.sequence_name << '\n' << this->results_.size() << '\n';
			outfile.close();

			int total_cnt{};

			/* For each frame */
			for (int i = 0; i < this->results_.size(); ++i) {
				/* Frame dir */
				boost::format fmt_3{"%s/%s_%04d"};
				fmt_3 % dirs % this->params_->io.sequence_name % i;
				std::string sub_dirs = fmt_3.str();

				/* Create frame dir */
				std::string s2 = "mkdir " + sub_dirs;

				a = system(s2.c_str());

				/* Write patch indexes to file */
				outfile.open(sub_dirs + "/.config");
				outfile << '\n';
				outfile.close();

				int frame_cnt{};
				for (int j = 0; j < this->results_[i].size(); ++j) {
					if (!common::CheckSliceType(this->results_[i][j].type, common::PVVC_SLICE_TYPE_VALID)) {
						std::cout << "frame : " << i << " patch : " << j << '\n';
					}
					else {
						/* Each patch */
						boost::format fmt_4{"%s/%s_time_%04d_slice_%04d.slice"};
						fmt_4 % sub_dirs % this->params_->io.sequence_name % this->results_[i][j].timestamp % this->results_[i][j].index;

						std::string name = fmt_4.str();
						frame_cnt += io::SaveSlice(this->results_[i][j], name);
						outfile << this->results_[i][j].index << ' ';
					}
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

			/* Log start */
			boost::format fmt_begin{"\033[%1%mLoad gops ......\033[0m\n"};
			fmt_begin % common::AZURE;
			std::cout << fmt_begin;

			/* Root dirs */
			std::string dirs = this->params_->io.deform_file;
			if (dirs.back() != '/') {
				dirs += '/';
			}
			dirs += "gops";

			/* Check status */
			std::ifstream infile;

			std::string seq_name;
			int patch_num;

			/* Read sequence name and patch number from file */
			infile.open(dirs + "/.config");
			infile >> seq_name >> patch_num;
			infile.close();

			/* Init container */
			this->gops_.resize(patch_num);

			/* For each patch */
			for (int i = 0; i < patch_num; ++i) {
				/* Patch dir Root/seq_patch_%04d */
				boost::format patch_dir_fmt{"%s/%s_patch_%04d"};
				patch_dir_fmt % dirs % seq_name % i;
				std::string patch_dir = patch_dir_fmt.str();

				/* Read GoP number of this patch */
				int gop_cnt{};
				infile.open(patch_dir + "/.config");
				infile >> gop_cnt;
				infile.close();

				this->gops_[i].resize(gop_cnt);
				for (int j = 0; j < gop_cnt; ++j) {
					/* GoP dir Root/seq_patch_%04d/seq_patch_%04d_gop_%04d */
					boost::format gop_dir_fmt{"%s/%s_patch_%04d_gop_%04d"};
					gop_dir_fmt % patch_dir % seq_name % i % j;
					std::string gop_dir = gop_dir_fmt.str();

					/* Read start and end timestamp from file */
					int start_time{}, end_time{};
					infile.open(gop_dir + "/.config");
					infile >> start_time >> end_time;
					infile.close();

					/* Load fitting cloud */
					this->gops_[i][j].cloud = io::LoadColorPlyFile(gop_dir + "/fitting_cloud.ply");
					this->gops_[i][j].start = start_time;
					this->gops_[i][j].end = end_time;

					for (int t = start_time; t <= end_time; t += this->params_->time_interval) {
						boost::format patch_fmt{"%s/%s_patch_%04d_gop_%04d_time_%04d.patch"};
						patch_fmt % gop_dir % seq_name % i % j % t;
						std::string patch_name = patch_fmt.str();
						common::Patch p_tmp;
						io::LoadPatch(p_tmp, patch_name);
						this->gops_[i][j].patches.emplace_back(p_tmp);
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
		int frame_cnt{};

		infile.open(dirs + "/.config");
		infile >> seq_name >> frame_cnt;
		infile.close();

		struct stat info;
		if (stat(frame_dir.c_str(), &info) == 0) {
			std::string s = "rm -rf " + frame_dir;
			int a = system(s.c_str());
		}

		std::string ss = "mkdir " + frame_dir;
		int aa = system(ss.c_str());

		for (int f = 0; f < frame_cnt; ++f) {
			boost::format sub_dir_fmt{"%s/%s_%04d"};
			sub_dir_fmt % dirs % seq_name % f;
			boost::format f_fmt{"%s/%s_%d.frame"};
			f_fmt % frame_dir % seq_name % f;
			std::string frame = f_fmt.str();
			std::string sub_dir = sub_dir_fmt.str();
			std::filesystem::path dir_path{sub_dir};
			io::ChangeSliceToFrame(dir_path, frame);
			std::cout << "Save frame " << frame << '\n';
		}
	}
}  // namespace codec
}  // namespace vvc
