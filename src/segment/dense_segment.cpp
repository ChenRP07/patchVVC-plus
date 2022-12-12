/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/09 16:53
 * Last Modified : 2022/12/09 18:56
 *
 */

#include "segment/segment.h"

using namespace vvc;

void segment::dense_segment::block_segment(std::vector<size_t>& old_block_, std::vector<size_t>& new_block_a, std::vector<size_t>& new_block_b) {
    float max_x = FLT_MIN, max_y = FLT_MIN, max_z = FLT_MIN;
    float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;

    for (auto i : old_block_) {
        max_x = std::max(max_x, this->source_cloud_->at(i).x), min_x = std::min(min_x, this->source_cloud_->at(i).x);
        max_y = std::max(max_y, this->source_cloud_->at(i).y), min_y = std::min(min_y, this->source_cloud_->at(i).y);
        max_z = std::max(max_z, this->source_cloud_->at(i).z), min_z = std::min(min_z, this->source_cloud_->at(i).z);
    }


}

void segment::dense_segment::Segment(int _k) {
	const int kPatch         = _k;
	const int kPointPerPatch = std::floor(this->source_cloud_->size() / kPatch);

	auto heap_cmp = [](const std::vector<size_t>& _a, const std::vector<size_t>& _b) -> bool { return _a.size() < _b.size(); };

	std::priority_queue<std::vector<size_t>, std::vector<std::vector<size_t>>, decltype(heap_cmp)> block_queue(heap_cmp);

	std::vector<size_t> points;
	while (points.size() < this->source_cloud_->size()) {
		points.emplace_back(points.size());
	}

    block_queue.push(points);

    while (block_queue.top().size() > kPointPerPatch) {
        std::vector<size_t> temp_block = block_queue.top();
        block_queue.pop();
        std::vector<size_t> new_block_a, new_block_b;
    }
}
