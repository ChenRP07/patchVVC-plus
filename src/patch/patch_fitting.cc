/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/03 11:18
 * Last Modified : 2023/04/03 11:18
 *
 */

#include "patch/patch.h"

namespace vvc {
namespace patch {
	PatchFitting::PatchFitting() : fitting_cloud_{nullptr}, source_patches_{}, params_{nullptr}, stat_{}, clock_{}, max_height_{0} {}

	void PatchFitting::SetParams(vvc::common::PVVCParam_t::Ptr _param) {
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

	bool PatchFitting::AddPatch(vvc::common::Patch _patch) {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (!_patch || _patch.empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			this->clock_.SetTimeBegin();
            this->stat_.iters.clear();
			/* First patch in a GOP, save as fitting_cloud_ */
			if (!fitting_cloud_ || this->source_patches_.empty()) {
				this->source_patches_.emplace_back(_patch);
				this->fitting_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
				*(this->fitting_cloud_) += *(this->source_patches_.front());
				this->clock_.SetTimeEnd();
				this->stat_.costs.emplace_back(this->clock_.GetTimeMs());
				return true;
			}

			/* Otherwise, use ICP to calcualte MSE between new patch and fitting patch, if MSE is less than a threshold, add this patch into GOP and regenerate fitting patch */
			registration::ICPBase::Ptr icp;
			if (this->params_->icp.type == common::NORMAL_ICP) {
				if (this->params_->icp.radius_search_ths <= 0) {
					throw __EXCEPT__(BAD_PARAMETERS);
				}
				pcl::PointCloud<pcl::Normal>                         normal_fit, normal_res;
				pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> est_x, est_y;
				pcl::search::KdTree<pcl::PointXYZRGB>::Ptr           kdtree_x, kdtree_y;

				kdtree_x->setInputCloud(this->fitting_cloud_);
				est_x.setInputCloud(this->fitting_cloud_);
				est_x.setSearchMethod(kdtree_x);
				est_x.setRadiusSearch(this->params_->icp.radius_search_ths);
				est_x.compute(normal_fit);

				kdtree_y->setInputCloud(_patch.cloud);
				est_y.setInputCloud(_patch.cloud);
				est_y.setSearchMethod(kdtree_y);
				est_y.setRadiusSearch(this->params_->icp.radius_search_ths);
				est_y.compute(normal_res);

				icp.reset(new registration::NICP());
				icp->SetSourceNormal(normal_fit.makeShared());
				icp->SetTargetNormal(normal_res.makeShared());
			}
			else {
				icp.reset(new registration::ICP());
			}

			icp->SetParams(this->params_);
			icp->SetSourceCloud(this->fitting_cloud_);
			icp->SetTargetCloud(_patch.cloud);
			icp->Align();

			/* This patch can be added into GOP, change point cloud to transformed cloud, record motion vector */
			if (icp->Converged()) {
				if (icp->GetMSE() <= this->params_->patch.fitting_ths) {
					_patch.cloud = icp->GetResultCloud();
					_patch.mv    = icp->GetMotionVector() * _patch.mv;
					this->source_patches_.emplace_back(_patch);
					/* Compute fitting patch */
					this->Compute();
					this->stat_.score.emplace_back(icp->GetMSE());
					this->clock_.SetTimeEnd();
					this->stat_.score.emplace_back(this->clock_.GetTimeMs());
					return true;
				}
			}

			this->clock_.SetTimeEnd();
			this->stat_.score.emplace_back(this->clock_.GetTimeMs());
			/* Can not add this patch into GOP */
			return false;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PatchFitting::Compute() {
		try {
			if (this->params_->patch.clustering_ths < 1.0f) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			/* Compute range and center */
			float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
			float max_x = FLT_TRUE_MIN, max_y = FLT_TRUE_MIN, max_z = FLT_TRUE_MIN;

			for (auto& p : this->source_patches_) {
				for (auto i : *p) {
					min_x = i.x < min_x ? i.x : min_x;
					min_y = i.y < min_y ? i.y : min_y;
					min_z = i.z < min_z ? i.z : min_z;
					max_x = i.x > max_x ? i.x : max_x;
					max_y = i.y > max_y ? i.y : max_y;
					max_z = i.z > max_z ? i.z : max_z;
				}
			}

			pcl::PointXYZ range, center;

			center.x = (max_x + min_x) / 2.0f;
			center.y = (max_y + min_y) / 2.0f;
			center.z = (max_z + min_z) / 2.0f;

			range.x = max_x - min_x;
			range.y = max_y - min_y;
			range.z = max_z - min_z;

            int range_height = static_cast<int>(std::ceil(std::log2(std::max(range.x, std::max(range.y, range.z)))));
            int ths_height = static_cast<int>(std::ceil(std::log2(this->params_->patch.clustering_ths)));
            this->max_height_ = range_height - ths_height;

			/* Init point idx */
			std::vector<std::vector<int>> points(this->source_patches_.size());
			for (int i = 0; i < points.size(); ++i) {
				points[i].resize(this->source_patches_[i].size(), 0);
				std::iota(points[i].begin(), points[i].end(), 0);
			}

			/* Iteratively split */
			this->Split(points, center, range, 0);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PatchFitting::Clustering(std::vector<std::vector<int>>& _points) {
		try {
			/* K-Means centroids */
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr centers(new pcl::PointCloud<pcl::PointXYZRGB>());
			/* All points to be clustered */
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>());

			/* Compute centroids size, i.e., K */
			int k_num = 0;
			for (auto& i : _points) {
				k_num += i.size();
			}
			k_num = std::round(static_cast<float>(k_num) / static_cast<float>(_points.size()));

			/* Initialize the clustering points */
			for (int i = 0; i < _points.size(); ++i) {
				for (auto p : _points[i]) {
					points->emplace_back(this->source_patches_[i][p]);
				}
			}

			/* Initialize the centroids, randomly select points */
			for (int i = points->size() - 1; i >= points->size() - k_num; --i) {
				int idx = rand() % (i + 1);
				std::swap(points->at(i), points->at(idx));
				centers->emplace_back(points->at(i));
			}

			/* KNN searching method */
			pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
			std::vector<int>                      idx(1);
			std::vector<float>                    dis(1);

			/* K-Means */
			int iter = 1;
			for (; iter <= this->params_->patch.max_iter; ++iter) {
				/* New centroids */
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr next_center(new pcl::PointCloud<pcl::PointXYZRGB>());
				next_center->resize(centers->size());
				for (auto& p : *next_center) {
					p.x = p.y = p.z = 0.0f;
				}

				/* Point number of last clusters */
				std::vector<int> points_count(centers->size(), 0);

				/* Add point to nearest cluster */
				kdtree.setInputCloud(centers);
				for (auto p : *points) {
					kdtree.nearestKSearch(p, 1, idx, dis);
					next_center->at(idx.front()).x += p.x;
					next_center->at(idx.front()).y += p.y;
					next_center->at(idx.front()).z += p.z;
					points_count[idx.front()]++;
				}

				/* Compute new centroids */
				float error = 0.0f;
				for (int i = 0; i < points->size(); ++i) {
					/* There is no point in this cluster, choose a random point as the new centroid */
					if (points_count[i] == 0) {
						next_center->at(i) = points->at(rand() % points->size());
					}
					/* Otherwise, use mean of cluster points as the new centroid */
					else {
						next_center->at(i).x /= static_cast<float>(points_count[i]);
						next_center->at(i).y /= static_cast<float>(points_count[i]);
						next_center->at(i).z /= static_cast<float>(points_count[i]);
					}

					/* Compute error between last centroid and next centroid */
					error +=
					    std::sqrt(std::pow(next_center->at(i).x - centers->at(i).x, 2) + std::pow(next_center->at(i).y - centers->at(i).y, 2) + std::pow(next_center->at(i).z - centers->at(i).z, 2));
				}
				error /= static_cast<float>(centers->size());
				/* Update centroids */
				centers.swap(next_center);

				/* K-Means converged */
				if (error < this->params_->patch.clustering_ths) {
					break;
				}
			}

			/* Add centroids to fitting cloud */
			*(this->fitting_cloud_) += *centers;
			/* Record iters */
			this->stat_.iters.emplace_back(iter);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void PatchFitting::Split(std::vector<std::vector<int>>& _points, pcl::PointXYZ _center, pcl::PointXYZ _range, int _height) {
		try {
			/* Reaches the givien depth threshold, do clustering */
			if (_height >= this->max_height_) {
				this->Clustering(_points);
				/* Release memory */
				std::vector<std::vector<int>>().swap(_points);
				return;
			}

			/* Try to split _points into eight sub-spaces, each sub-space must contains points from all patches */
			/* Eight subspaces indexes are below :
			 *   z+
			 *   6————4
			 *  /|   /|
			 * 2—+——0 |
			 * | 7——+-5 y+
			 * |/   |/
			 * 3————1
			 *x+
			 * */

			/*
			 * 1st dim: subspaces idx 0-7;
			 * 2nd dim: patch idx;
			 * 3rd dim: point idx;
			 * */
			std::vector<std::vector<std::vector<int>>> subspaces(8, std::vector<std::vector<int>>(_points.size(), std::vector<int>()));

			/* For each patch */
			for (int i = 0; i < _points.size(); ++i) {
				for (auto idx : _points[i]) {
					/* Compute subspace position */
					int pos = 0;
					pos |= this->source_patches_[i][idx].x > _center.x ? 0 : 1;
					pos <<= 1;
					pos |= this->source_patches_[i][idx].y > _center.y ? 0 : 1;
					pos <<= 1;
					pos |= this->source_patches_[i][idx].z > _center.z ? 0 : 1;
					/* Add to relative subspace */
					subspaces[pos][i].emplace_back(idx);
				}
			}

			/* If split _points into eight sub-spaces will make at least one sub-space lose point from some patches */
			if (!octree::CheckSubSpace(subspaces)) {
				bool do_clustering = true;

				/* There are two methods to handle this problem */
				/* One is try to split _points by x/y/z-plane */
				if (this->params_->patch.split_method == common::PLANAR_BISECTION) {
					std::vector<std::vector<std::vector<int>>> planar_bisection_result(2, std::vector<std::vector<int>>(_points.size()));

					/* Try to split, return the most proper result */
					int split_type = this->PlanarBisection(_points, _center, planar_bisection_result);
					if (split_type != -1) {
						do_clustering                    = false;
						pcl::PointXYZ subspace_range     = _range;
						pcl::PointXYZ subspace_center[2] = {_center, _center};
						float         mid;
						if (split_type == 0) {
							subspace_range.x /= 2.0f;
							subspace_center[0].x += subspace_range.x / 2.0f;
							subspace_center[1].x -= subspace_range.x / 2.0f;
							mid = _center.x;
						}
						else if (split_type == 1) {
							subspace_range.y /= 2.0f;
							subspace_center[0].y += subspace_range.y / 2.0f;
							subspace_center[1].y -= subspace_range.y / 2.0f;
							mid = _center.y;
						}
						else if (split_type == 2) {
							subspace_range.z /= 2.0f;
							subspace_center[0].z += subspace_range.z / 2.0f;
							subspace_center[1].z -= subspace_range.z / 2.0f;
							mid = _center.z;
						}
						else {
							throw __EXCEPT__(BAD_POSITION);
						}

						/* Release memory */
						std::vector<std::vector<int>>().swap(_points);
						/* Iteratively split */
						for (int i = 0; i < 2; ++i) {
							this->Split(planar_bisection_result[i], subspace_center[i], subspace_range, _height + 1);
						}
					}
				}
				else if (this->params_->patch.split_method == common::PARTIAL_CLUSTERING) {
					do_clustering = false;
					std::vector<std::vector<int>> clustering_points(_points.size());
					pcl::PointXYZ                 subspace_range(_range.x / 2.0f, _range.y / 2.0f, _range.z / 2.0f);
					for (int i = 0; i < 8; ++i) {
						if (octree::CheckSubSpace(subspaces[i])) {
							pcl::PointXYZ subspace_center = octree::SubSpaceCenter(_center, subspace_range, i);
							this->Split(subspaces[i], subspace_center, subspace_range, _height + 1);
						}
						else {
							for (int idx = 0; idx < subspaces[i].size(); ++idx) {
								for (auto p : subspaces[i][idx]) {
									clustering_points[idx].emplace_back(p);
								}
							}
						}
					}
					std::vector<std::vector<int>>().swap(_points);
					this->Clustering(clustering_points);
				}
				else {
					throw __EXCEPT__(BAD_PARAMETERS);
				}

				/* Non method can be used, do clustering */
				if (do_clustering) {
					this->Clustering(_points);
					/* Release memory */
					std::vector<std::vector<int>>().swap(_points);
				}
			}
			/* Otherwise, continue to split */
			else {
				/* Release memory */
				std::vector<std::vector<int>>().swap(_points);
				/* Subspace range is half of _range */
				pcl::PointXYZ subspace_range(_range.x / 2.0f, _range.y / 2.0f, _range.z / 2.0f);
				for (int i = 0; i < 8; ++i) {
					/* Compute subspace center and split it */
					pcl::PointXYZ subspace_center = octree::SubSpaceCenter(_center, subspace_range, i);
					this->Split(subspaces[i], subspace_center, subspace_range, _height + 1);
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	int PatchFitting::PlanarBisection(std::vector<std::vector<int>>& _points, pcl::PointXYZ _center, std::vector<std::vector<std::vector<int>>>& _result) {
		try {
			/* Split score */
			float score[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
			/* Middle coordinate */
			float                                                   Mid[3] = {_center.x, _center.y, _center.z};
			std::vector<std::vector<std::vector<std::vector<int>>>> sub(3, std::vector<std::vector<std::vector<int>>>(2, std::vector<std::vector<int>>(_points.size())));

			/* For x/y/z */
			for (int i = 0; i < 3; ++i) {
				/* Split */
				for (int idx = 0; idx < _points.size(); ++i) {
					for (auto p : _points[idx]) {
						float data[3] = {this->source_patches_[idx][p].x, this->source_patches_[idx][p].y, this->source_patches_[idx][p].z};
						if (data[i] > Mid[i]) {
							sub[i][0][idx].emplace_back(p);
						}
						else {
							sub[i][1][idx].emplace_back(p);
						}
					}
				}
				/* Check it */
				if (!octree::CheckSubSpace(sub[i])) {
					continue;
				}
				/* Return type which has min dev of patch size */
				std::vector<int> size_0, size_1;
				for (auto& s : sub[i][0]) {
					size_0.emplace_back(s.size());
				}
				for (auto& s : sub[i][1]) {
					size_1.emplace_back(s.size());
				}
				score[i] = std::max(common::Deviation(size_0), common::Deviation(size_1));
			}

			float Min  = FLT_MAX;
			int   type = -1;
			for (int i = 0; i < 3; ++i) {
				if (score[i] != FLT_MAX && score[i] < Min) {
					Min  = score[i];
					type = i;
				}
			}
			if (type != -1) {
				_result = std::move(sub[type]);
			}
			return type;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr PatchFitting::GetFittingCloud() const {
		return this->fitting_cloud_;
	}

	std::vector<common::Patch> PatchFitting::GetSourcePatches() {
		return std::move(this->source_patches_);
	}

    // clang-format off
	void PatchFitting::Log() const {
		std::cout << __AZURET__(===================================================) << std::endl;
		if (this->params_->log_level & 0x01) {
            std::cout << __BLUET__(Update fitting point cloud.) << std::endl;
            std::cout << __BLUET__(Fitting cloud size : ) << " " << this->fitting_cloud_->size() << std::endl;
            std::cout << __BLUET__(Registration error : ) << " ";
            printf("%.3f\n", this->stat_.score.back());
            std::cout << __BLUET__(Avg clustering iter : ) << " ";
            int ans = std::accumulate(this->stat_.iters.begin(), this->stat_.iters.end(), 0);
            printf("%.3f\n", static_cast<float>(ans) / static_cast<float>(this->stat_.iters.size()));
            std::cout << __BLUET__(Time consuming : );
            printf("%.3f  %.3f\n", this->stat_.costs.back() / 1000.0f, this->stat_.costs.back());
        }
		std::cout << __AZURET__(===================================================) << std::endl;
	}

    // clang-format on
    void PatchFitting::Clear() {
        this->fitting_cloud_->clear();
        this->source_patches_.clear();
        this->stat_.costs.clear(), this->stat_.iters.clear(), this->stat_.score.clear();

    }
}  // namespace patch
}  // namespace vvc

