/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/02 16:54
 * Last Modified : 2023/05/02 16:54
 *
 */
#include "cuda/octree.cuh"

namespace vvc {
namespace octree {

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
		_height = _p[24];
	}

    // TODO SetSlice 类型暂无
    // void InvertRAHTOctree::SetSlice(const common::Slice& _slice) 

    // TODO 返回类型 common:Patch
    __device__ void InvertRAHTOctree::GetPatch() const {
        // /* Generate Patch and copy data */
        // int timestamp = this->slice_.timestamp;
        // int index     = this->slice_.index;
        // int mv        = this->slice_.mv;

        // /* Malloc a point cloud */
        // result.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        // /* Convert yuv to rgb and concate with xyz */
        // for (int i = 0; i < this->source_cloud_->size(); ++i) {
        //     pcl::PointXYZRGB p;
        //     p.x = this->source_cloud_->at(i).x, p.y = this->source_cloud_->at(i).y, p.z = this->source_cloud_->at(i).z;
        //     this->source_colors_->at(i).ConvertRGB(p);
        //     result.cloud->emplace_back(p);
        // }
        // return result;
	}

    // TODO 未知大小
    __device__ void InvertRAHTOctree::MakeTree() {

        /* Load center, range and height from geometry */
        uint8_t tree_attr[25]{};

        int node_values_index = 0;
        for (; node_values_index < 25; node_values_index++) {
            tree_attr[node_values_index] = node_values_[node_values_index];
        }

        LoadTreeCore(this->tree_center_, this->tree_range_, this->tree_height_, tree_attr);

        this->tree_ = (OctreeNode_t **)malloc(sizeof(OctreeNode_t *) * this->tree_height_);
        int curr_layer_node_count = 1;

        /* Assign value for each branch node */
        for (int idx = 0; idx < this->tree_height_ - 1; ++idx) {
            /* Count how many nodes next layer has */
            int next_layer_node_count = 0;
            /* Assign nodes in current layer */
            this->tree_[idx] = (OctreeNode_t *)malloc(sizeof(OctreeNode_t *) * curr_layer_node_count);
            for (int cnt = 0; cnt < curr_layer_node_count; ++cnt) {
                /* Set value */
                this->tree_[idx][cnt].value = node_values_[node_values_index];
                /* Count 1-bits of this node */
                for ( int i=0; i<8; i++){
                    if (((node_values_[node_values_index]) & NodeValue[i]) == 0) {
                        this->tree_[idx][cnt].index[i] = -1;
                    }
                    else{
                        this->tree_[idx][cnt].index[i] = next_layer_node_count;
                        ++next_layer_node_count;
                    }
                }
                node_values_index++;
            }
            curr_layer_node_count = next_layer_node_count;
        }

        /* Malloc space for last layer */
        this->tree_[this->tree_height_ - 1] = (OctreeNode_t *)malloc(sizeof(OctreeNode_t *) * curr_layer_node_count);

        /* Update weight and add point into cloud */
        // this->AddPoints(0, 0, this->tree_center_, this->tree_range_);
	}

    // TODO source_cloud_的大小位置 如何进行尾插
    __device__ void InvertRAHTOctree::AddPoints(const int _height, const int _index, const common::PointXYZ _center, const common::PointXYZ _range) {
        auto& node = this->tree_[_height][_index];
        /* Leaf layer */
        if (_height == this->tree_height_ - 1) {
            node.value     = 0xff;
            node.weight[1] = 1;
            // TODO source_cloud 的大小
            // this->source_cloud_->emplace_back(_center);
            // node.index.emplace_back(this->source_cloud_->size());
        }
        else {
            /* Subrange : half of _range */
            common::PointXYZ subrange(_range.x / 2.0f, _range.y / 2.0f, _range.z / 2.0f);

        //     /* For each subnode */
        //     for (int i = 0; i < 8; ++i) {
        //         /* If subnode is not empty */
        //         if (node.index[i] != -1) {
        //             /* Compute subnode center */
        //             pcl::PointXYZ subcenter = SubSpaceCenter(_center, subrange, i);
        //             /* Iteratively traversal */
        //             this->AddPoints(_height + 1, node.index[i], subcenter, subrange);
        //         }
        //     }

        //     /* Update weight
        //         * Note the weights of children of node are already computed
        //         * */
        //     for (int i = 0; i < 8; ++i) {
        //         if (node.index[i] != -1) {
        //             node.weight[i + 8] = this->tree_[_height + 1][node.index[i]].weight[0];
        //         }
        //     }

        //     for (int i = 7; i > 0; --i) {
        //         node.weight[i] = node.weight[NodeWeight[i][0]] + node.weight[NodeWeight[i][1]];
        //     }
        }
	}


}  // namespace octree
}  // namespace vvc
