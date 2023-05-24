/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin
 * Description   :
 * Create Time   : 2023/05/02 16:54
 * Last Modified : 2023/05/02 16:54
 *
 */
#include "cuda/octree.cuh"
#include "cuda/entropy_codec.cuh"
#include <stdio.h>

namespace vvc {
namespace client{
namespace octree {
    __device__ void InvertHaarTransform(int _w0, int _w1, const common::ColorYUV& _g0, const common::ColorYUV& _g1, common::ColorYUV& _res0, common::ColorYUV& _res1) {
		common::ColorYUV G{}, H{};
		if (_w0 == 0 && _w1 == 0) {
            _res0 = G;
            _res1 = H;
			return ;
		}

		/*
		 *        [ √w0 -√w1]
		 *        [ √w1 √w0 ]
		 * T^-1 = -----------
		 *        √(w0 + w1)
		 *
		 * */
		float base    = sqrtf(static_cast<float>(_w0 + _w1));
		float T[2][2] = {{sqrtf(static_cast<float>(_w0)) / base, -sqrt(static_cast<float>(_w1)) / base},
		                 {sqrtf(static_cast<float>(_w1)) / base, sqrt(static_cast<float>(_w0)) / base}};

		G = _g0 * T[0][0] + _g1 * T[0][1];
		H = _g0 * T[1][0] + _g1 * T[1][1];
        _res0 = G;
        _res1 = H;
		return ;
	}

    __device__ void LoadTreeCore(common::PointXYZ& _center, common::PointXYZ& _range, int& _height, uint8_t (&_p)[25]) {
		float data[6] = {};

		for (int i = 0; i < 6; ++i) {
			int d{0};
			for (int j = 0; j < 4; ++j) {
				d <<= 8;
				d |= _p[i * 4 + j];
			}
			float* t = reinterpret_cast<float*>(&d);
			data[i]  = *t;
		}
		_center.x = data[0], _center.y = data[1], _center.z = data[2];
		_range.x = data[3], _range.y = data[4], _range.z = data[5];
        uint8_t temp = _p[24];
		_height = static_cast<int>(temp);
	}

    __device__ common::PointXYZ SubSpaceCenter(common::PointXYZ _center, common::PointXYZ _range, int _pos) {
        switch (_pos) {
            case 0: return common::PointXYZ(_center.x + _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z + _range.z / 2.0f);
            case 1: return common::PointXYZ(_center.x + _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z - _range.z / 2.0f);
            case 2: return common::PointXYZ(_center.x + _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z + _range.z / 2.0f);
            case 3: return common::PointXYZ(_center.x + _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z - _range.z / 2.0f);
            case 4: return common::PointXYZ(_center.x - _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z + _range.z / 2.0f);
            case 5: return common::PointXYZ(_center.x - _range.x / 2.0f, _center.y + _range.y / 2.0f, _center.z - _range.z / 2.0f);
            case 6: return common::PointXYZ(_center.x - _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z + _range.z / 2.0f);
            case 7: return common::PointXYZ(_center.x - _range.x / 2.0f, _center.y - _range.y / 2.0f, _center.z - _range.z / 2.0f);
            default: printf("SubSpaceCenter ERROR\n"); return common::PointXYZ();
        }
	}

    __device__ void OctreeNode_t::InvertHierarchicalTransform() {
		common::ColorYUV H[8];
		for (int i = 0; i < 8; ++i) {
			H[i] = this->raht[i];
		}
		/* g_DC */
		this->raht[1] = H[0];
		/* Invert X/Y/Z merge */
		for (int i = 1; i < 8; ++i) {
			// std::pair<common::ColorYUV, common::ColorYUV> g(this->raht[i], H[i]), res;
			// std::pair<int, int>                           w(this->weight[NodeWeight[i][0]], this->weight[NodeWeight[i][1]]);
			/* Inver haar wavelet transform */
			InvertHaarTransform(this->weight[NodeWeight[i][0]], this->weight[NodeWeight[i][1]], this->raht[i], H[i], this->raht[NodeWeight[i][0]], this->raht[NodeWeight[i][1]]);
		}
	}

    __device__ void InvertRAHTOctree::SetSlice(const common::Slice_t& _slice) {
        this->slice_ = _slice;
        /* Optional Zstd decoding */
        this->node_values_ = this->slice_.geometry;
        auto temp_color = this->slice_.color;
        /* RLGR decoding */
        common::RLGRDecoder rlgr_dec;
        rlgr_dec.Decode(temp_color, this->slice_.color_size, 3 * this->slice_.size);
        auto rlgr_res       = rlgr_dec.GetResult();
        free(this->coefficients_);
        this->coefficients_ = (common::ColorYUV*)malloc(sizeof(common::ColorYUV) * this->slice_.size);
        /* Reconstruct coefficients */
        for (int i = 0; i < this->slice_.size; ++i) {
            this->coefficients_[i].y = static_cast<float>(rlgr_res[i] * this->slice_.qp);
            this->coefficients_[i].u = static_cast<float>(rlgr_res[i + this->slice_.size] * this->slice_.qp);
            this->coefficients_[i].v = static_cast<float>(rlgr_res[i + 2 * this->slice_.size] * this->slice_.qp);
        }
        free(rlgr_res);
        /* If intra slice, clear tree and related container */
        if (!common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_PREDICT)) {
            for (int height = 0; height < this->tree_height_; ++height) {
                delete []this->tree_[height].nodes;
            }
            free(this->tree_);
            free(this->source_cloud_);
            this->source_cloud_index_ = 0;
            free(this->reference_colors_);
            free(this->source_colors_);

            this->source_cloud_ = (common::PointXYZ*)malloc(sizeof(common::PointXYZ) * this->slice_.size);
            this->reference_colors_ = (common::ColorYUV*)malloc(sizeof(common::ColorYUV) * this->slice_.size);
            this->source_colors_ = (common::ColorYUV*)malloc(sizeof(common::ColorYUV) * this->slice_.size);
            this->MakeTree();
        }
        this->InvertRAHT();
	} 

    __device__ void InvertRAHTOctree::GetPatch()  {
        /* Generate Patch and copy data */
        printf("this->slice_.timestamp = %d\n",this->slice_.timestamp);
        printf("this->slice_.index = %d\n",this->slice_.index);
        for(uint32_t i=0; i<4; i++){
            for (uint32_t j=0; j<4; j++){
                printf("%.2f ", this->slice_.mv(i,j));
            }
            printf("\n");
        }
        for(int i=0; i<source_cloud_index_; i++){
            float r = ((this->source_colors_[i].y + 1.4020f * (this->source_colors_[i].v - 128.0f)));
			float g = ((this->source_colors_[i].y - 0.3441f * (this->source_colors_[i].u - 128.0f) - 0.7141f * (this->source_colors_[i].v - 128.0f)));
			float b = ((this->source_colors_[i].y + 1.7720f * (this->source_colors_[i].u - 128.0f)));
            printf("(%.3f,%.3f,%.3f - %.0f,%.0f,%.0f)\n", this->source_cloud_[i].x, this->source_cloud_[i].y, this->source_cloud_[i].z, r, g, b);
        }
	}

    __device__ void InvertRAHTOctree::MakeTree() {
        /* Load center, range and height from geometry */
        uint8_t tree_attr[25]{};

        int node_values_index = 0;
        for (; node_values_index < 25; node_values_index++) {
            tree_attr[node_values_index] = node_values_[node_values_index];
        }

        LoadTreeCore(this->tree_center_, this->tree_range_, this->tree_height_, tree_attr);
        
        this->tree_ = (OctreeLayer_t *)malloc(sizeof(OctreeLayer_t) * this->tree_height_);
        int curr_layer_node_count = 1;
        common::ColorYUV zero{};
        /* Assign value for each branch node */
        for (int idx = 0; idx < this->tree_height_ - 1; ++idx) {
            /* Count how many nodes next layer has */
            int next_layer_node_count = 0;
            /* Assign nodes in current layer */
            this->tree_[idx].nodes = new OctreeNode_t[curr_layer_node_count];
            this->tree_[idx].length = curr_layer_node_count;
            for (int cnt = 0; cnt < curr_layer_node_count; ++cnt) {
                /* Set value */
                this->tree_[idx].nodes[cnt].value = node_values_[node_values_index];
                /* Count 1-bits of this node */
                for ( int i=0; i<8; i++){
                    if (((node_values_[node_values_index]) & NodeValue[i]) == 0) {
                        this->tree_[idx].nodes[cnt].index[i] = -1;
                    }
                    else{
                        this->tree_[idx].nodes[cnt].index[i] = next_layer_node_count;
                        ++next_layer_node_count;
                    }
                }
                node_values_index++;
            }
            curr_layer_node_count = next_layer_node_count;
        }

        /* Malloc space for last layer */
        this->tree_[this->tree_height_ - 1].nodes = (OctreeNode_t *)malloc(sizeof(OctreeNode_t) * curr_layer_node_count);
        this->tree_[this->tree_height_ - 1].length = curr_layer_node_count;
        
        /* Update center and range for each node */
        this->tree_[0].nodes[0].center = this->tree_center_;
        this->tree_[0].nodes[0].range  = this->tree_range_;
        for (int layer = 0; layer < this->tree_height_ - 1; ++layer) {
            for (int idx = 0; idx < this->tree_[layer].length; ++idx) { 
                auto& node = this->tree_[layer].nodes[idx];
                common::PointXYZ subrange(node.range.x / 2.0f, node.range.y / 2.0f, node.range.z / 2.0f);
                for (int i = 0; i < 8; ++i) {
                    if (node.index[i] != -1) {
                        this->tree_[layer + 1].nodes[node.index[i]].center = SubSpaceCenter(node.center, subrange, i);
                        this->tree_[layer + 1].nodes[node.index[i]].range = subrange;
                    }     
                }
            }
        }
        
        /* Collect last layer centers and update weight for last layer */
        for (int idx = 0; idx < this->tree_[this->tree_height_ - 1].length; ++idx) {
            auto& node = this->tree_[this->tree_height_ - 1].nodes[idx];
            node.value = 0xff;
            node.weight[1] = 1;
            this->source_cloud_[this->source_cloud_index_] = node.center;
            node.index[0] = this->source_cloud_index_;
            this->source_cloud_index_++;
        }

        /* Reversely update weight for each node */
        for (int layer = this->tree_height_ - 2; layer >= 0; --layer) {
            for (int idx = 0; idx < this->tree_[layer].length; ++idx) {
                auto& node = this->tree_[layer].nodes[idx];
                for (int i = 0; i < 8; ++i) {
                    if (node.index[i] != -1) {
                        node.weight[i + 8] = this->tree_[layer + 1].nodes[node.index[i]].weight[1];
                    }
                }
                for (int i = 7; i > 0; --i) {
                    node.weight[i] = node.weight[NodeWeight[i][0]] + node.weight[NodeWeight[i][1]];
                }
            }
        }
        /* Update weight and add point into cloud */
	}

    __device__ void InvertRAHTOctree::InvertRAHT() {
        
        /* coefficients_ and source_cloud_ have the same size */
        int coefficients_index = this->source_cloud_index_ - 1;

        /* Set g_DC */
        this->tree_[0].nodes[0].raht[0] = this->coefficients_[coefficients_index];
        coefficients_index--;

        for (int i = 0; i < this->tree_height_ - 1; ++i) {
            for(int j = 0; j < this->tree_[i].length; j++){
                auto &node = this->tree_[i].nodes[j];
                /* Set h_ACs */
                for (int idx = 1; idx < 8; ++idx) {
                    if (node.weight[NodeWeight[idx][0]] != 0 && node.weight[NodeWeight[idx][1]] != 0) {
                        node.raht[idx] = this->coefficients_[coefficients_index];
                        coefficients_index --;
                    }
                }
                /* Compute g_DC */
                node.InvertHierarchicalTransform();

                /* Update g_DC for each subnode */
                for (int idx = 0; idx < 8; ++idx) {
                    if (node.index[idx] != -1) {
                        this->tree_[i + 1].nodes[node.index[idx]].raht[0] = node.raht[idx + 8];
                    }
                }
            }
        }

        /* Collect colors */
        OctreeLayer_t &last_layer = this->tree_[tree_height_ - 1];
        for (int i = 0; i < last_layer.length; i++){
            this->source_colors_[i] = last_layer.nodes[i].raht[0];
        }

        if (common::CheckSliceType(this->slice_.type, common::PVVC_SLICE_TYPE_PREDICT)) {
            for (int i = 0; i < this->source_cloud_index_; ++i) {
                this->source_colors_[i] += this->reference_colors_[i];
            }
        }
        else {
            for(int i=0; i<source_cloud_index_; i++){
                this->reference_colors_[i] = this->source_colors_[i];
            }
        }
    }

}  // namespace octree
}
}  // namespace vvc
