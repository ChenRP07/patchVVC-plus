/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2022/12/12 16:11
 * Last Modified : 2022/12/12 16:11
 * 
 */ 

#ifndef _PARAM_H_
#define _PARAM_H_

#include "common/exception.h"
#include <stdint.h>
#include <memory>

namespace vvc {
namespace common {

    struct VVCParam_t {
        uint8_t log_level_; /* quiet brief normal complete */
        int patch_num_;
    };

    extern void SetDefaultParams(std::shared_ptr<VVCParam_t> _ptr);
}
}

#endif 
