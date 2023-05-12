/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/12 19:59
 * Last Modified : 2023/05/12 19:59
 *
 */
#ifndef _PVVC_CUDA_SLICE_IO_H_
#define _PVVC_CUDA_SLICE_IO_H_

#include "cuda/base.cuh"

#include <string>
#include <zstd.h>

namespace vvc {
namespace io {
	extern void LoadSliceT(common::Slice_t& _slice, const std::string& _name);
}
}  // namespace vvc

#endif
