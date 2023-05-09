/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDU_IIC. All Right Reserved.
 *
 * Author        : Lixin
 * Description   :
 * Create Time   : 2023/05/08 10:12
 * Last Modified : 2023/05/08 10:12
 *
 */

#ifndef _PVVC_CUDA_BASE_CUH_
#define _PVVC_CUDA_BASE_CUH_

namespace vvc {
namespace common {

	/* Point type in cloud */
	struct Points {
		float x, y, z; /* Geometry */
		float r, g, b; /* Color */

		/* Constructor and assignment */
		Points() : x{}, y{}, z{}, r{}, g{}, b{} {};

		Points(const Points& _p) : x{_p.x}, y{_p.y}, z{_p.z}, r{_p.r}, g{_p.g}, b{_p.b} {}

		Points& operator=(const Points& _p) {
			this->x = _p.x, this->y = _p.y, this->z = _p.z;
			this->r = _p.r, this->g = _p.g, this->b = _p.b;
			return *this;
		}
	};

	struct CudaSlice {};

	struct CudaFrame {
		CudaSlice** slices;
		int         size;
	};

	struct CudaFrameBuffer {
		CudaFrame* buffer;
		int        size;
		int        capacity;
		CudaFrameBuffer() : buffer{}, size{}, capacity{} {}
	};
}  // namespace common
}  // namespace vvc

#endif
