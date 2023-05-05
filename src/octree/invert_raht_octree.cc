/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/03 12:10
 * Last Modified : 2023/05/03 12:10
 *
 */

#include "octree/octree.h"

namespace vvc {
namespace octree {

	InvertRAHTOctree::InvertRAHTOctree() : tree_{}, source_cloud_{nullptr}, source_colors_{nullptr}, slice_{} {}

	void InvertRAHTOctree::SetSlice(const common::Slice& _slice) {
		try {
			/* _slice must be valid */
			if (!common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_VALID)) {
				throw __EXCEPT__(BAD_SLICE);
			}

			/* _slice should have data */
			if (!_slice.geometry || !_slice.color) {
				throw __EXCEPT__(BAD_SLICE);
			}

			/* If this->slice_ is invalid, i.e., no intra slice */
			if (!common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_VALID)) {
				/* _slice should not be a predictive slice */
				if (common::CheckSliceType(_slice.type, common::PVVC_SLICE_TYPE_PREDICT)) {
					throw __EXCEPT__(EMPTY_REFERENCE);
				}
			}
			this->slice_ = _slice;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	common::Patch InvertRAHTOctree::GetPatch() const {
		try {
			if (!this->source_cloud_ || this->source_cloud_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			if (!this->source_colors_ || this->source_colors_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}

			if (this->source_colors_->size() != this->source_cloud_->size()) {
				throw __EXCEPT__(UNMATCHED_COLOR_SIZE);
			}
			/* Generate Patch and copy data */
			vvc::common::Patch result;
			result.timestamp = this->slice_.timestamp;
			result.index     = this->slice_.index;
			result.mv        = this->slice_.mv;

			/* Malloc a point cloud */
			result.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
			/* Convert yuv to rgb and concate with xyz */
			for (int i = 0; i < this->source_cloud_->size(); ++i) {
				pcl::PointXYZRGB p;
				p.x = this->source_cloud_->at(i).x, p.y = this->source_cloud_->at(i).y, p.z = this->source_cloud_->at(i).z;
				this->source_colors_->at(i).ConvertRGB(p);
				result.cloud->emplace_back(p);
			}
			return result;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void InvertRAHTOctree::MakeTree() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			/* slice_ should have data */
			if (!this->slice_.geometry) {
				throw __EXCEPT__(BAD_SLICE);
			}

			if (!common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_VALID)) {
				throw __EXCEPT__(BAD_SLICE);
			}

			if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_PREDICT)) {
				throw __EXCEPT__(EMPTY_REFERENCE);
			}

			/* Optional Zstd decoding */
			if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_GEO_ZSTD)) {
				common::ZstdDecoder dec;
				dec.Decode(this->slice_.geometry);
				this->node_values_ = dec.GetResult();
			}
			else {
				this->node_values_ = this->slice_.geometry;
			}
			/* Load center, range and height from geometry */
			uint8_t tree_attr[25] = {};

			auto iter = this->node_values_->begin();

			for (auto& i : tree_attr) {
				if (iter != this->node_values_->end()) {
					i    = *iter;
					iter = std::next(iter, 1);
				}
				else {
					throw __EXCEPT__(BAD_SLICE);
				}
			}

			LoadTreeCore(this->tree_center_, this->tree_range_, this->tree_height_, tree_attr);

			this->tree_.resize(this->tree_height_);
			int curr_layer_node_count = 1;

			/* Assign value for each branch node */
			for (int idx = 0; idx < this->tree_height_ - 1; ++idx) {
				/* Count how many nodes next layer has */
				int next_layer_node_count = 0;
				/* Assign nodes in current layer */
				this->tree_.at(idx).resize(curr_layer_node_count);
				for (int cnt = 0; cnt < curr_layer_node_count; ++cnt) {
					if (iter != this->node_values_->end()) {
						/* Set value */
						this->tree_.at(idx).at(cnt).value = *iter;
						/* Count 1-bits of this node */
						for (auto i : NodeValue) {
							/* 0-bit, index is -1 */
							if (((*iter) & i) == 0) {
								this->tree_.at(idx).at(cnt).index.emplace_back(-1);
							}
							/* 1-bit */
							else {
								this->tree_.at(idx).at(cnt).index.emplace_back(next_layer_node_count);
								++next_layer_node_count;
							}
						}
						iter = std::next(iter, 1);
					}
				}
				curr_layer_node_count = next_layer_node_count;
			}
			if (curr_layer_node_count != this->slice_.size) {
				throw __EXCEPT__(UNMATCHED_COLOR_SIZE);
			}
			this->tree_.front().resize(curr_layer_node_count);

			this->AddPoints(0, 0, this->tree_center_, this->tree_range_);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void InvertRAHTOctree::AddPoints(const int _height, const int _index, const pcl::PointXYZ _center, const pcl::PointXYZ _range) {
		try {
			/* Check valid */
			if (_height >= this->tree_height_) {
				throw __EXCEPT__(OUT_OF_RANGE);
			}
			if (_index >= this->tree_.at(_height).size()) {
				throw __EXCEPT__(OUT_OF_RANGE);
			}

			auto& node = this->tree_[_height][_index];
			/* Leaf layer */
			if (_height == this->tree_height_ - 1) {
				node.value     = 0xff;
				node.weight[1] = 1;
				this->source_cloud_->emplace_back(_center);
				node.index.emplace_back(this->source_cloud_->size());
			}
			else {
				/* Subrange : half of _range */
				pcl::PointXYZ subrange(_range.x / 2.0f, _range.y / 2.0f, _range.z / 2.0f);

				/* For each subnode */
				for (int i = 0; i < 8; ++i) {
					/* If subnode is not empty */
					if (node.index[i] != -1) {
						/* Compute subnode center */
						pcl::PointXYZ subcenter = SubSpaceCenter(_center, subrange, i);
						/* Iteratively traversal */
						this->AddPoints(_height + 1, node.index[i], subcenter, subrange);
					}
				}

				/* Update weight
				 * Note the weights of children of node are already computed
				 * */
				for (int i = 0; i < 8; ++i) {
					if (node.index[i] != -1) {
						node.weight[i + 8] = this->tree_[_height + 1][node.index[i]].weight[0];
					}
				}

				for (int i = 7; i > 0; --i) {
					node.weight[i] = node.weight[NodeWeight[i][0]] + node.weight[NodeWeight[i][1]];
				}
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace octree
}  // namespace vvc
