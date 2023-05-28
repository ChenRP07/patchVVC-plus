/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/24 10:36
 * Last Modified : 2023/05/24 10:36
 *
 */

#include "codec/pvvc_encoder.h"

namespace vvc {
namespace codec {

	PVVCDecompression::PVVCDecompression() : params_{}, clock_{}, slices_{}, patches_{}, results_{}, handler_{} {}
	void PVVCDecompression::SetParams(common::PVVCParam_t::Ptr _param) {
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

	void PVVCDecompression::LoadSlices() {
		/* Log start */
		boost::format fmt_begin{"\033[%1%mLoad slices ......\033[0m\n"};
		fmt_begin % common::AZURE;
		std::cout << fmt_begin;

		/* Rootdir */
		std::string dirs = this->params_->io.result_file;
		if (dirs.back() != '/') {
			dirs += '/';
		}

		dirs += "slice";

		/* Check status */
		std::ifstream infile;

		/* Read sequence name and frame number from file */
		infile.open(dirs + "/.config");
		std::string seq_name;
		int frame_cnt;
		infile >> seq_name >> frame_cnt;
		infile.close();
		this->slices_.resize(frame_cnt);

		/* For each frame */
		for (int i = 0; i < frame_cnt; ++i) {
			int time = this->params_->start_timestamp + i * this->params_->time_interval;
			/* Frame dir */
			boost::format sub_dirs_fmt{"%s/%s_%04d"};
			sub_dirs_fmt % dirs % seq_name % i;
			std::string sub_dirs = sub_dirs_fmt.str();

			/* Read all patch index from file */
			std::vector<int> patch_cnt;
			int index_num{};
			infile.open(sub_dirs + "/.config");
			while (infile >> index_num) {
				patch_cnt.emplace_back(index_num);
			}
			infile.close();
			this->slices_[i].resize(patch_cnt.size());
			for (int j = 0; j < patch_cnt.size(); j++) {
				boost::format name_fmt{"%s/%s_time_%04d_slice_%04d.slice"};
				name_fmt % sub_dirs % seq_name % time % patch_cnt[j];
				std::string name = name_fmt.str();
				io::LoadSlice(this->slices_[i][j], name);
			}
		}
		printf("\033[%dmLoad patches finished.\033[0m\n", common::AZURE);
	}

	void PVVCDecompression::Task(int k) {
		while (true) {
			int data_idx = -1;
			this->task_queue_mutex_.lock();
			if (!this->task_queue_.empty()) {
				data_idx = this->task_queue_.front();
				this->task_queue_.pop();
			}
			this->task_queue_mutex_.unlock();
			if (data_idx == -1) {
				break;
			}

			int patch_idx = this->slices_[k][data_idx].index;
			this->handler_[patch_idx].SetSlice(this->slices_[k][data_idx]);
			this->patches_[k][data_idx] = this->handler_[patch_idx].GetPatch();
		}
	}

	void PVVCDecompression::Decompression() {
		this->avg_geo = this->avg_y = 0.0f;
		this->psnr_cnt = 0;
		this->patches_.resize(this->slices_.size());
		this->results_.resize(this->slices_.size(), nullptr);
		this->threads_.resize(this->params_->thread_num);
		for (int frame = 0; frame < this->slices_.size(); frame++) {
			this->patches_[frame].resize(this->slices_[frame].size());

			int max_idx{};
			for (auto& i : this->slices_[frame]) {
				max_idx = std::max(i.index, max_idx);
			}

			while (this->handler_.size() < max_idx + 1) {
				this->handler_.emplace_back();
				this->handler_.back().SetParams(this->params_);
			}

			common::PVVCTime_t tim;
			tim.SetTimeBegin();
			for (int patch = 0; patch < this->slices_[frame].size(); patch++) {
				this->task_queue_.push(patch);
			}

			for (auto& t : this->threads_) {
				t = std::thread(&PVVCDecompression::Task, this, frame);
			}

			for (auto& t : this->threads_) {
				t.join();
			}
			this->results_[frame].reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			tim.SetTimeEnd();
			printf("decode frame %d, time %.2fms\n", frame, tim.GetTimeMs());

			for (auto& p : this->patches_[frame]) {
				for (auto& pp : *p.cloud) {
					auto tp = pp;
					tp.x = pp.x * p.mv(0, 0) + pp.y * p.mv(0, 1) + pp.z * p.mv(0, 2) + p.mv(0, 3);
					tp.y = pp.x * p.mv(1, 0) + pp.y * p.mv(1, 1) + pp.z * p.mv(1, 2) + p.mv(1, 3);
					tp.z = pp.x * p.mv(2, 0) + pp.y * p.mv(2, 1) + pp.z * p.mv(2, 2) + p.mv(2, 3);

					this->results_[frame]->emplace_back(tp);
				}
			}
			boost::format fmt{this->params_->io.source_file};
			fmt % (this->params_->start_timestamp + frame * this->params_->time_interval);
			auto cloud_source = io::LoadColorPlyFile(fmt.str());
			common::MSE m;
			m.SetClouds(cloud_source, this->results_[frame]);
			m.Compute();
			float g_mse = std::max(m.GetGeoMSEs().first, m.GetGeoMSEs().second);
			float y_mse = std::max(m.GetYMSEs().first, m.GetYMSEs().second);
			g_mse = 10.0f * std::log10(1024.0f * 1024.0f / g_mse);
			y_mse = 10.0f * std::log10(256.0f * 256.0f / y_mse);

			this->avg_geo += g_mse;
			this->avg_y += y_mse;
			this->psnr_cnt++;
			printf("\tGEO : %.2f   Y : %.2f\n", g_mse, y_mse);
		}

		this->avg_geo /= this->psnr_cnt;
		this->avg_y /= this->psnr_cnt;

		printf("Avg GEO: %.2f\nAvg Y: %.2f\n", this->avg_geo, this->avg_y);
	}
}  // namespace codec
}  // namespace vvc
