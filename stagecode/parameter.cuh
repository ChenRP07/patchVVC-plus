/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin, ChenRP07
 * Description   :
 * Create Time   : 2023/05/08 15:09
 * Last Modified : 2023/05/08 15:09
 *
 */
#ifndef _PVVC_CUDA_PARAMETER_H_
#define _PVVC_CUDA_PARAMETER_H_

#include "cuda/common/exception.cuh"

namespace vvc {
namespace common {
	struct PVVCParamCuda_t {
		struct {
			float scr_width;
			float scr_height;
			float FoV;
			float camera_pos[3][2];
			char* scr_name;
			int   gl_major_version; /* OpenGL version major */
			int   gl_minor_version; /* OpenGL version minor */
		} Render;

		struct Decoder_t {
            unsigned int buffer_size;

		} Decoder;

		using DecPtr = Decoder_t*;
		using Ptr    = PVVCParamCuda_t*;
		DecPtr GetDecoderParam() {
			return &(this->Decoder);
		}
	};

	extern void SetDefaultParamsCuda(PVVCParamCuda_t::Ptr _param);
}  // namespace common
}  // namespace vvc
#endif
