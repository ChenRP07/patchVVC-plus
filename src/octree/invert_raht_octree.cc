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
			if (!_slice.geometry && !_slice.color) {
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
			/* Optional Zstd decoding */
			if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_GEO_ZSTD)) {
				common::ZstdDecoder dec;
				dec.Decode(this->slice_.geometry);
				this->node_values_ = dec.GetResult();
			}
			else {
				this->node_values_ = this->slice_.geometry;
			}

			auto temp_color = this->slice_.color;
			if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_COLOR_ZSTD)) {
				common::ZstdDecoder dec;
				dec.Decode(this->slice_.color);
				temp_color = dec.GetResult();
			}

			/* RLGR decoding */
			common::RLGRDecoder rlgr_dec;
			rlgr_dec.Decode(temp_color, 3 * this->slice_.size);
			auto rlgr_res       = rlgr_dec.GetResult();
			this->coefficients_ = std::make_shared<std::vector<common::ColorYUV>>(this->slice_.size);
			/* Reconstruct coefficients */
			for (int i = 0; i < this->slice_.size; ++i) {
				this->coefficients_->at(i).y = static_cast<float>(rlgr_res->at(i) * this->slice_.qp);
				this->coefficients_->at(i).u = static_cast<float>(rlgr_res->at(i + this->slice_.size) * this->slice_.qp);
				this->coefficients_->at(i).v = static_cast<float>(rlgr_res->at(i + 2 * this->slice_.size) * this->slice_.qp);
			}

			/* If intra slice, clear tree and related container */
			if (!common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_PREDICT)) {
				this->tree_.clear();
				this->source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
				this->reference_colors_ = std::make_shared<std::vector<common::ColorYUV>>();
				this->source_colors_    = std::make_shared<std::vector<common::ColorYUV>>();
				this->MakeTree();
			}
			this->InvertRAHT();
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

			for (auto i : tree_attr) {
				printf("%02x ", i);
			}
			printf("\n");
			LoadTreeCore(this->tree_center_, this->tree_range_, this->tree_height_, tree_attr);
			std::cout << this->tree_center_ << '\n' << this->tree_range_ << '\n' << this->tree_height_ << '\n';

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
			/* Malloc space for last layer */
			this->tree_.back().resize(curr_layer_node_count);

			/* Update weight and add point into cloud */
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
						node.weight[i + 8] = this->tree_[_height + 1][node.index[i]].weight[1];
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

	void InvertRAHTOctree::InvertRAHT() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (this->tree_.size() != this->tree_height_ || this->tree_height_ <= 0) {
				throw __EXCEPT__(EMPTY_OCTREE);
			}
			if (!this->coefficients_ || this->coefficients_->size() != this->source_cloud_->size()) {
				throw __EXCEPT__(UNMATCHED_COLOR_SIZE);
			}
			std::reverse(this->coefficients_->begin(), this->coefficients_->end());
			/* Iterator */
			auto iter = this->coefficients_->begin();

			/* Set g_DC */
			this->tree_.front().front().raht[0] = *iter;

			// iter = std::next(iter, 1);
			++iter;

			for (int i = 0; i < this->tree_height_ - 1; ++i) {
				for (auto& node : this->tree_[i]) {
					/* Set h_ACs */
					for (int idx = 1; idx < 8; ++idx) {
						if (node.weight[NodeWeight[idx][0]] != 0 && node.weight[NodeWeight[idx][1]] != 0) {
							node.raht[idx] = *iter;
							// iter           = std::next(iter, 1);
							++iter;
						}
					}
					/* Compute g_DC */
					node.InvertHierarchicalTransform();

					/* Update g_DC for each subnode */
					for (int idx = 0; idx < 8; ++idx) {
						if (node.index[idx] != -1) {
							this->tree_[i + 1][node.index[idx]].raht[0] = node.raht[idx + 8];
						}
					}
				}
			}

			/* Collect colors */
			this->source_colors_->clear();
			for (auto& node : this->tree_.back()) {
				this->source_colors_->emplace_back(node.raht[0]);
			}
			if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_PREDICT)) {
				for (int i = 0; i < this->source_colors_->size(); ++i) {
					this->source_colors_->at(i) += this->reference_colors_->at(i);
				}
			}
			else {
				this->reference_colors_->assign(this->source_colors_->begin(), this->source_colors_->end());
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace octree
}  // namespace vvc
