/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/14 10:33
 * Last Modified : 2023/04/14 10:33
 *
 */

#include "octree/octree.h"

namespace vvc {
namespace octree {
	RAHTOctree::RAHTOctree() : OctreeBase{}, tree_{}, source_cloud_{nullptr}, source_colors_{nullptr}, RAHT_result_{nullptr} {}

	void RAHTOctree::SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) {
		try {
			if (!_cloud || _cloud->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}
			this->source_cloud_ = _cloud;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void RAHTOctree::SetSourceColors(std::shared_ptr<std::vector<common::ColorYUV>> _colors) {
		try {
			if (!_colors || _colors->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			this->source_colors_ = _colors;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::shared_ptr<std::vector<uint8_t>> RAHTOctree::GetOctree() const {
		auto result = std::make_shared<std::vector<uint8_t>>();
		SaveTreeCore(this->tree_center_, this->tree_range_, this->tree_height_, result);
		for (int i = 0; i < this->tree_height_ - 1; ++i) {
			for (auto& j : this->tree_.at(i)) {
				result->emplace_back(j.value);
			}
		}
		return result;
	}

	std::shared_ptr<std::vector<common::ColorYUV>> RAHTOctree::GetRAHTResult() {
		return this->RAHT_result_;
	}

	void RAHTOctree::MakeTree() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (this->params_->octree.resolution < 0.0f) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			if (!this->source_cloud_ || this->source_cloud_->empty()) {
				throw __EXCEPT__(EMPTY_POINT_CLOUD);
			}

			/* Compute cloud range and cloud center */
			float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
			float max_x = FLT_TRUE_MIN, max_y = FLT_TRUE_MIN, max_z = FLT_TRUE_MIN;
			for (auto i : *this->source_cloud_) {
				min_x = std::min(min_x, i.x), min_y = std::min(min_y, i.y), min_z = std::min(min_z, i.z);
				max_x = std::max(max_x, i.x), max_y = std::max(max_y, i.y), max_z = std::max(max_z, i.z);
			}

			this->tree_center_.x = max_x / 2.0f + min_x / 2.0f;
			this->tree_center_.y = max_y / 2.0f + min_y / 2.0f;
			this->tree_center_.z = max_z / 2.0f + min_z / 2.0f;

			float max_range  = std::max(max_x - min_x, std::max(max_y - min_y, max_z - min_z));
			int   max_height = static_cast<int>(std::ceil(std::log2(max_range)));
			int   ths_height = static_cast<int>(std::ceil(std::log2(this->params_->octree.resolution)));
			/* For example, 8->4->2 : 3 - 1 + 1 = 3 */
			this->tree_height_ = max_height - ths_height + 1;

			if (this->tree_height_ <= 0) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			this->tree_.resize(this->tree_height_);
			max_range         = std::pow(2.0f, max_height);
			this->tree_range_ = pcl::PointXYZ(max_range, max_range, max_range);

			std::vector<int> points(this->source_cloud_->size());
			std::iota(points.begin(), points.end(), 0);
			this->AddNode(points, 0, this->tree_center_, this->tree_range_);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void RAHTOctree::AddNode(std::vector<int>& _points, const int _height, const pcl::PointXYZ _center, const pcl::PointXYZ _range) {
		try {
			/* No points, return */
			if (_points.empty()) {
				return;
			}

			/* Leaf layer */
			if (_height == this->tree_height_ - 1) {
				OctreeNode_t node;
				node.value = 0xff;
				/* Record point index */
				node.index.swap(_points);
				/* Set weight */
				node.weight[1] = 1;
				/* Add node in this layer */
				this->tree_.at(_height).emplace_back(node);
			}
			/* Branch layer */
			else {
				OctreeNode_t node;
				node.index.resize(8, -1);
				/* Subrange : half of _range */
				pcl::PointXYZ subrange(_range.x / 2.0f, _range.y / 2.0f, _range.z / 2.0f);
				/* Point index in 8 subnodes */
				std::vector<int> subnodes[8];
				for (auto i : _points) {
					int pos = 0;
					pos |= this->source_cloud_->at(i).x > _center.x ? 0 : 1;
					pos <<= 1;
					pos |= this->source_cloud_->at(i).y > _center.y ? 0 : 1;
					pos <<= 1;
					pos |= this->source_cloud_->at(i).z > _center.z ? 0 : 1;
					subnodes[pos].emplace_back(i);
				}
				/* Release _points */
				std::vector<int>().swap(_points);

				/* For each subnode */
				for (int i = 0; i < 8; ++i) {
					/* If subnode is not empty */
					if (!subnodes[i].empty()) {
						/* Set i-th bit to 1 */
						node.value |= NodeValue[i];
						/* Compute subnode center */
						pcl::PointXYZ subcenter = SubSpaceCenter(_center, subrange, i);
						/* Next line will add an element in next layer */
						node.index[i] = this->tree_.at(_height + 1).size();
						/* Iteratively add subnode */
						this->AddNode(subnodes[i], _height + 1, subcenter, subrange);
					}
					/* Else, empty subnode, do nothing */
				}

				/* 8 subnode weight, i.e., weight[8] to weight[15] */
				for (int i = 0; i < 8; ++i) {
					if (node.index[i] != -1) {
						node.weight[i + 8] = this->tree_[_height + 1][node.index[i]].weight[1];
					}
				}
				/* Compute weight[1] to weight[7] */
				for (int i = 7; i > 0; --i) {
					node.weight[i] = node.weight[NodeWeight[i][0]] + node.weight[NodeWeight[i][1]];
				}
				/* Add node in this layer */
				this->tree_.at(_height).emplace_back(node);
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void RAHTOctree::RAHT() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (this->tree_.size() != this->tree_height_ || this->tree_height_ <= 0) {
				throw __EXCEPT__(EMPTY_OCTREE);
			}

			if (!this->source_colors_ || this->source_colors_->size() != this->source_cloud_->size()) {
				throw __EXCEPT__(UNMATCHED_COLOR_SIZE);
			}

			/* Do voxel downsampling for the leaf nodes */
			for (auto& node : this->tree_.back()) {
				common::ColorYUV temp;
				for (auto i : node.index) {
					temp += this->source_colors_->at(i);
				}
				temp /= node.index.size();
				node.raht[0] = temp;
			}

			for (int i = this->tree_height_ - 2; i >= 0; --i) {
				for (auto& node : this->tree_[i]) {
					/* Collect subnodes' g_DC */
					for (int idx = 0; idx < 8; ++idx) {
						if (node.index[idx] != -1) {
							node.raht[idx + 8] = this->tree_[i + 1][node.index[idx]].raht[0];
						}
					}
					/* Do region-adaptive hierarchical transform */
					node.HierarchicalTransform();
				}
			}

			/* Collect all coefficients */
			this->RAHT_result_ = std::make_shared<std::vector<common::ColorYUV>>();
			/* g_DC */
			this->RAHT_result_->emplace_back(this->tree_.front().front().raht[0]);
			/* h_AC */
			for (int i = 0; i < this->tree_height_ - 1; ++i) {
				for (auto& node : this->tree_[i]) {
					/* w1,w2 should both greater than 0 */
					for (int idx = 1; idx < 8; ++idx) {
						if (node.weight[NodeWeight[idx][0]] != 0 && node.weight[NodeWeight[idx][1]] != 0) {
							this->RAHT_result_->emplace_back(node.raht[idx]);
						}
					}
				}
			}
			std::reverse(this->RAHT_result_->begin(), this->RAHT_result_->end());
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace octree
}  // namespace vvc

