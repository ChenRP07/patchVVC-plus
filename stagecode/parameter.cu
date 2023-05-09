/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin, ChenRP07
 * Description   :
 * Create Time   : 2023/05/08 15:12
 * Last Modified : 2023/05/08 15:12
 *
 */

#include "cuda/common/parameter.cuh"

namespace vvc {
namespace common {

	void SetDefaultParamsCuda(PVVCParamCuda_t::Ptr _param) {
		try {
			if (!_param) {
				throw __CUDA_EXCEPT__(EMPTY_PARAMS);
			}
			_param->Render.FoV        = 45.0f;
			_param->Render.scr_width  = 800.0f;
			_param->Render.scr_height = 600.0f;
			const char* name          = "patchVVC\0";
			strcpy(_param->Render.scr_name, name);
			_param->Render.gl_major_version = 3;
			_param->Render.gl_minor_version = 3;
		}
		catch (const common::CudaException& e) {
			e.Log();
			throw __CUDA_EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace common
}  // namespace vvc

