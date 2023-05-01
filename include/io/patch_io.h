/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07. All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2023/05/01 13:54
 * Last Modified : 2023/05/01 13:54
 * 
 */

#ifndef _PVVC_PATCH_IO_H_
#define _PVVC_PATCH_IO_H_

#include "common/common.h"

#include <cstdio>

namespace vvc {
namespace io
{
    extern void SavePatch(const common::Patch _patch, const std::string _name);   

    extern common::Patch LoadPatch(const std::string _name);
}
}

#endif 
