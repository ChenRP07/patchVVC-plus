/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2022/12/12 16:38
 * Last Modified : 2022/12/12 16:38
 * 
 */ 

#include "common/param.h"

using namespace vvc;

void common::SetDefaultParams(std::shared_ptr<VVCParam_t> _ptr) {
    try {
    if (!_ptr) {
        throw __EXCEPT__(EMPTY_PARAMS);
    }
    
    _ptr->log_level_ = 0x03;

    _ptr->patch_num_ = 100;
    
    } catch (const common::Exception& e) {
        e.Log();
        throw __EXCEPT__(ERROR_OCCURED);
    }
}
