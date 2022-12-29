/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of dense segmentation.
 * Create Time   : 2022/12/09 16:53
 * Last Modified : 2022/12/14 17:25
 *
 */

#include "segment/segment.h"

using namespace vvc;

void segment::DenseSegment::BlockSegment(std::vector<size_t>& _old_block, std::vector<size_t>& _new_block_a, std::vector<size_t>& _new_block_b) {
	// calculate the bounding box of old_block_
	float max_x = FLT_MIN, max_y = FLT_MIN, max_z = FLT_MIN;
	float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

	for (auto i : _old_block) {
		max_x = std::max(max_x, this->source_cloud_->at(i).x), min_x = std::min(min_x, this->source_cloud_->at(i).x);
		max_y = std::max(max_y, this->source_cloud_->at(i).y), min_y = std::min(min_y, this->source_cloud_->at(i).y);
		max_z = std::max(max_z, this->source_cloud_->at(i).z), min_z = std::min(min_z, this->source_cloud_->at(i).z);
	}

	// segment old_block_ into two new_block along the max range dimension
	float max_range;
	char  div_dim;

	if (max_x - min_x > max_y - min_y) {
		max_range = max_x - min_x;
		div_dim   = 'x';
	}
	else {
		max_range = max_y - min_y;
		div_dim   = 'y';
	}

	if (max_z - min_z > max_range) {
		max_range = max_z - min_z;
		div_dim   = 'z';
	}

	if (div_dim == 'x') {
		float div_mid = min_x + max_range / 2.0f;
		for (auto i : _old_block) {
			if (this->source_cloud_->at(i).x < div_mid) {
				_new_block_a.emplace_back(i);
			}
			else {
				_new_block_b.emplace_back(i);
			}
		}
	}
	else if (div_dim == 'y') {
		float div_mid = min_y + max_range / 2.0f;
		for (auto i : _old_block) {
			if (this->source_cloud_->at(i).y < div_mid) {
				_new_block_a.emplace_back(i);
			}
			else {
				_new_block_b.emplace_back(i);
			}
		}
	}
	else if (div_dim == 'z') {
		float div_mid = min_z + max_range / 2.0f;
		for (auto i : _old_block) {
			if (this->source_cloud_->at(i).z < div_mid) {
				_new_block_a.emplace_back(i);
			}
			else {
				_new_block_b.emplace_back(i);
			}
		}
	}
}

void segment::DenseSegment::Segment() {
	try {
        if (!this->source_cloud_ || this->source_cloud_->empty()) {
            throw __EXCEPT__(EMPTY_POINT_CLOUD);
        }

        if (!this->params_) {
            throw __EXCEPT__(EMPTY_PARAMS);
        }

        if (this->params_->patch_num_ < 2) {
            throw __EXCEPT__(INVALID_PARAM_SEGMENT);
        }

        std::cout << __GREENT__(Start dense segmentation.) << std::endl;
		/* patch number and point per patch */
		const int kPatch         = this->params_->patch_num_;
		const int kPointPerPatch = std::floor(this->source_cloud_->size() / kPatch);

		/* heap compare lambda function */
		auto heap_cmp = [](const std::vector<size_t>& _a, const std::vector<size_t>& _b) -> bool { return _a.size() < _b.size(); };

		/* a max-heap, the vector which has the largest size will be the top element */
		std::priority_queue<std::vector<size_t>, std::vector<std::vector<size_t>>, decltype(heap_cmp)> block_queue(heap_cmp);

		/* index of each point */
		std::vector<size_t> points;
		while (points.size() < this->source_cloud_->size()) {
			points.emplace_back(points.size());
		}

		/* init, push the whole point cloud into queue */
		block_queue.push(points);

		/* do segment until the size of largest block is less than the point per patch */
		while (block_queue.top().size() > kPointPerPatch) {
			/* segment the largest block into two subblocks along the max span dimension, push the subblocks into queue */
			std::vector<size_t> temp_block = block_queue.top();
			block_queue.pop();
			std::vector<size_t> new_block_a, new_block_b;
			this->BlockSegment(temp_block, new_block_a, new_block_b);
			block_queue.push(new_block_a), block_queue.push(new_block_b);
		}

		/* for top _k largest blocks, calculate there centroids */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZRGB>());
		for (int i = 0; i < kPatch; i++) {
			pcl::PointXYZRGB    center;
			std::vector<size_t> temp_block = block_queue.top();
			block_queue.pop();
			center.x = center.y = center.z = 0.0f;
			for (auto i : temp_block) {
				center.x += this->source_cloud_->at(i).x;
				center.y += this->source_cloud_->at(i).y;
				center.z += this->source_cloud_->at(i).z;
			}
			center.x /= temp_block.size();
			center.y /= temp_block.size();
			center.z /= temp_block.size();

			centroids->emplace_back(center);
		}

		/* search nearest centroid for firstly segmentation */
		pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
		kdtree.setInputCloud(centroids);

		std::vector<std::vector<size_t>> temp_result(kPatch, std::vector<size_t>());

		for (size_t i = 0; i < this->source_cloud_->size(); i++) {
			std::vector<int>   index(1);
			std::vector<float> distance(1);
			kdtree.nearestKSearch(this->source_cloud_->at(i), 1, index, distance);
			temp_result[index[0]].emplace_back(i);
		}

        this->stat_.expect_.clear();
        for (auto& i : temp_result) {
            this->stat_.expect_.emplace_back(i.size());
        }
        
        std::cout << __GREENT__(Check the patches size.) << std::endl;
		/* TODO: delete patch whose size less than 1/10 average size */
		/* NOTE: TODO above has been solved */
		std::vector<size_t>                    changed_point;
		std::vector<std::vector<size_t>>       result_index;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_centroids(new pcl::PointCloud<pcl::PointXYZRGB>());

		for (size_t i = 0; i < temp_result.size(); i++) {
			if (temp_result[i].size() < kPointPerPatch / 10) {
				changed_point.insert(changed_point.end(), temp_result[i].begin(), temp_result[i].end());
			}
			else {
				result_index.emplace_back(temp_result[i]);
				final_centroids->emplace_back(centroids->at(i));
			}
		}

		kdtree.setInputCloud(final_centroids);
		for (auto i : changed_point) {
			std::vector<int>   index(1);
			std::vector<float> distance(1);
			kdtree.nearestKSearch(this->source_cloud_->at(i), 1, index, distance);
			result_index[index[0]].emplace_back(i);
		}

		/* segmentation */
		for (size_t i = 0; i < result_index.size(); i++) {
			this->results_.emplace_back(new pcl::PointCloud<pcl::PointXYZRGB>());
			for (auto p : result_index[i]) {
				this->results_[i]->emplace_back(this->source_cloud_->at(p));
			}
		}

        this->stat_.fact_.clear();
        for (auto& i : this->results_) {
            this->stat_.fact_.emplace_back(i->size());
        }

        std::cout << __GREENT__(Dense segmentation done.) << std::endl;
        this->Log();
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
