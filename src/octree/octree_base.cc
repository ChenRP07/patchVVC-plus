/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2023/03/17 14:51
 * Last Modified : 2023/03/17 14:51
 * 
 */ 

#include "octree/octree.h"

using namespace vvc;

octree::OctreeBase::OctreeBase() : tree_nodes_{}, tree_range_{0.0f}, tree_height_{0}, params_{nullptr} {}


void octree::OctreeBase::SetParams(std::shared_ptr<common::PVVCParam_t> _param) {
    try {
        if (!_param) {
            throw __EXCEPT__(EMPTY_PARAMS);
        }
        this->params_ = _param;
    }
    catch(const common::Exception& e) {
        throw __EXCEPT__(ERROR_OCCURED);
    }
}
