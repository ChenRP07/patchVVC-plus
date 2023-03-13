/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : Implementation of parameters. 
 * Create Time   : 2022/12/12 16:38
 * Last Modified : 2022/12/27 14:06
 * 
 */ 

#include "common/parameter.h"

using namespace vvc;

void common::SetDefaultICPParams(ICPParam_t& _param) {
    try {
        // if (!_ptr) {
        //     throw __EXCEPT__(EMPTY_PARAMS);
        // }
        
        _param.centroid_alignment_ = true;
        _param.correspondence_ths_ = 100.0f;
        _param.iteration_ths_ = 100;
        _param.mse_ths_ = 0.01f;
        _param.transformation_ths_ = 1e-6;
    }
    catch (const common::Exception&e) {
        e.Log();
        throw __EXCEPT__(ERROR_OCCURED);
    }
}

void common::SetDefaultParams(std::shared_ptr<VVCParam_t> _ptr) {
    try {
    if (!_ptr) {
        throw __EXCEPT__(EMPTY_PARAMS);
    }
    
    /* 0Bxxxxxxx1 brief 0Bxxxxxx1x normal 0Bxxxxx1xx complete */
    _ptr->log_level_ = 0x03;

    _ptr->patch_num_ = 100;

    _ptr->thread_num_ = 30;

    common::SetDefaultICPParams(_ptr->icp_);
    
    } catch (const common::Exception& e) {
        e.Log();
        throw __EXCEPT__(ERROR_OCCURED);
    }
}
