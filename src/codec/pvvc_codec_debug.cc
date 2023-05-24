#include "codec/pvvc_encoder.h"

namespace vvc {
namespace codec {

	void PVVCDeformation::Test() {
		for (int i = this->params_->start_timestamp; i < this->params_->start_timestamp + this->params_->frames * this->params_->time_interval; i += this->params_->time_interval) {
			int idx = (i - this->params_->start_timestamp) / this->params_->time_interval;

			boost::format fmt{this->params_->io.source_file};
			fmt % i;
			auto                                   cloud_source = io::LoadColorPlyFile(fmt.str());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGB>());

			for (auto& p : this->patches_->at(idx)) {
				*cloud_target += *(p.cloud);
			}

			common::MSE m;
			m.SetClouds(cloud_source, cloud_target);
			m.Compute();
			std::cout << "Index : " << idx << "\n";
			printf("\tGeo: %.2f / %.2f  ||  Y: %.2f / %.2f\n\n", m.GetGeoMSEs().first, m.GetGeoMSEs().second, m.GetYMSEs().first, m.GetYMSEs().second);
		}
	}
	void PVVCCompression::Test() {
        int cnt{}, num{};
		for (int i = this->params_->start_timestamp; i < this->params_->start_timestamp + this->params_->frames * this->params_->time_interval; i += this->params_->time_interval) {
			int cnt = 0;
			// for (auto& q : this->patches_[0]->at(230)) {
			// 	for (auto& qq : q.patches) {
			// 		io::SaveColorPlyFile("data/ply/test_" + std::to_string(cnt++) + ".ply",qq.cloud);
			// 	}
   //          io::SaveColorPlyFile("data/ply/fit.ply", q.cloud);
			// }
   //      return;
			int idx = (i - this->params_->start_timestamp) / this->params_->time_interval;

			boost::format fmt{this->params_->io.source_file};
			fmt % i;
			auto                                   cloud_source = io::LoadColorPlyFile(fmt.str());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGB>());

			std::vector<float> mses;
			int                iii = -1;
			float              mmm = 0.0f;
            int total_cnt{};
			for (auto& t : *this->patches_.front()) {
				for (auto& gop : t) {
                    cnt += gop.end - gop.start + 1;
                    num++;
					for (auto& patch : gop.patches) {
						if (patch.timestamp == i) {
							auto        v = patch.mv.inverse();
							common::MSE mm;
							mm.SetClouds(gop.cloud, patch.cloud);
							mm.Compute();
							mses.push_back(std::max(mm.GetGeoMSEs().second, mm.GetGeoMSEs().first) * patch.size());
                            total_cnt += patch.size();
							if (mses.back() > mmm) {
								iii = mses.size() - 1;
								mmm = mses.back();
							}
							for (auto& p : *patch.cloud) {
								auto tmp = p;
								tmp.x    = p.x * v(0, 0) + p.y * v(0, 1) + p.z * v(0, 2) + v(0, 3);
								tmp.y    = p.x * v(1, 0) + p.y * v(1, 1) + p.z * v(1, 2) + v(1, 3);
								tmp.z    = p.x * v(2, 0) + p.y * v(2, 1) + p.z * v(2, 2) + v(2, 3);
								cloud_target->emplace_back(tmp);
							}
						}
					}
				}
			}
			float       avg  = std::accumulate(mses.begin(), mses.end(), 0.0f) / total_cnt;
			float       mmax = *std::max_element(mses.begin(), mses.end());
			float       mmin = *std::min_element(mses.begin(), mses.end());
			common::MSE m;
			m.SetClouds(cloud_source, cloud_target);
			m.Compute();
			std::cout << "Index : " << idx << "\n";
			printf("\tGeo: %.2f / %.2f  ||  Y: %.2f / %.2f\n", m.GetGeoMSEs().first, m.GetGeoMSEs().second, m.GetYMSEs().first, m.GetYMSEs().second);
			printf("\tAVG: %.2f  ||  MAX : %.2f  ||  MIN : %.2f\n", avg, mmax, mmin);
			printf("\t\t%d\n\n", iii);
		}

        std::cout << "Average GOP size : " << float(cnt) / float(num) << "\n";
	}
}  // namespace codec
}  // namespace vvc
