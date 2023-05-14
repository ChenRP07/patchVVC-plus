/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/03 12:55
 * Last Modified : 2023/05/03 12:55
 *
 */

#ifndef _PVVC_CUDA_BASE_CUH_
#define _PVVC_CUDA_BASE_CUH_

#include <cuda_runtime.h>
#include <stdlib.h>

typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

// typedef char int8_t;
// typedef short int16_t;
// typedef int int32_t;
// typedef long int64_t;

namespace vvc {
namespace common {
        __device__ static uint8_t PVVC_SLICE_TYPE_MASK[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

        /*
        * From low to high : valid 1 | I 0 P 1 | none 0 skip 1 | none 0 zstd 1 | none 0 zstd 1
        * */
        enum PVVC_SLICE_TYPE { PVVC_SLICE_TYPE_VALID, PVVC_SLICE_TYPE_PREDICT, PVVC_SLICE_TYPE_SKIP, PVVC_SLICE_TYPE_GEO_ZSTD, PVVC_SLICE_TYPE_COLOR_ZSTD };

        __device__ inline bool CheckSliceType(uint8_t _type, PVVC_SLICE_TYPE _MASK) {
		    return _type & PVVC_SLICE_TYPE_MASK[_MASK];
	    }

	struct PointXYZ {
		float x, y, z;

		__device__ PointXYZ() {
			this->x = 0.0f;
			this->y = 0.0f;
			this->z = 0.0f;
		}
		__device__ PointXYZ(float _x, float _y, float _z) {
			this->x = _x;
			this->y = _y;
			this->z = _z;
		}
	};

	/* Three channels color implementation, Y/Cb/Cr or Y/U/V */
	struct ColorYUV {
		float y, u, v; /* Channels */

		/* Default constructor */
		__device__ ColorYUV() : y{}, u{}, v{} {}

		/* Copy constructor */
		__device__ ColorYUV(const ColorYUV& _x) : y{_x.y}, u{_x.u}, v{_x.v} {}

            /* Assign constructor */
            __device__ ColorYUV& operator=(const ColorYUV& _x) {
                this->y = _x.y, this->u = _x.u, this->v = _x.v;
                return *this;
            }

		/* Construct by data of three channels */
		__device__ ColorYUV(float _r, float _g, float _b, bool _type = true) {
			if (_type) {
				this->y = _r, this->u = _g, this->v = _b;
			}
			else {
				this->y = _r * 0.299f + _g * 0.587f + _b * 0.114f;
				this->u = _r * -0.168736f - _g * 0.331264f + _b * 0.5f + 128;
				this->v = _r * 0.5f - _g * 0.418688f - _b * 0.081312f + 128;
			}
		}

		/* Add data with _x relative channel */
		__device__ ColorYUV& operator+=(const ColorYUV& _x) {
			this->y += _x.y, this->u += _x.u, this->v += _x.v;
			return *this;
		}

		/* Divide data by _x */
		__device__ ColorYUV& operator/=(const int _x) {
			this->y /= _x, this->u /= _x, this->v /= _x;
			return *this;
		}

		/* Multiple data by _x */
		__device__ ColorYUV& operator*=(const float _x) {
			this->y *= _x, this->u *= _x, this->v *= _x;
			return *this;
		}

		/* Add _x to this, return result */
		__device__ ColorYUV operator+(const ColorYUV& _x) const {
			ColorYUV result;
			result.y = this->y + _x.y, result.u = this->u + _x.u, result.v = this->v + _x.v;
			return result;
		}

		/* Multiple this by _x, return result */
		__device__ ColorYUV operator*(const float _x) const {
			ColorYUV result;
			result.y = this->y * _x, result.u = this->u * _x, result.v = this->v * _x;
			return result;
		}

		// /* Convert to a RGB format, according to BT.601 */
		// void ConvertRGB(pcl::PointXYZRGB& _p) {
		//     _p.r = static_cast<uint8_t>(std::round(this->y + 1.4020f * this->v));
		//     _p.g = static_cast<uint8_t>(std::round(this->y - 0.3441f * this->u - 0.7141f * this->v));
		//     _p.b = static_cast<uint8_t>(std::round(this->y + 1.7720f * this->u));
		// }
	};

	struct MotionVector {
		float data[16];

		__device__ MotionVector() : data{} {}

		__device__ MotionVector(const MotionVector& _x) {
			for (int i = 0; i < 16; ++i) {
				this->data[i] = _x.data[i];
			}
		}

		__device__ MotionVector& operator=(const MotionVector& _x) {
			for (int i = 0; i < 16; ++i) {
				this->data[i] = _x.data[i];
			}
			return *this;
		}

		__device__ float& operator()(uint32_t _x, uint32_t _y) {
			return this->data[_x * 4 + _y];
		}
	};

	struct Slice_t {
		int          timestamp;
		int          index;
		uint8_t      type;
		MotionVector mv;
		uint32_t     size;
		uint8_t      qp;
		uint8_t*     geometry;
		uint32_t     geometry_size;
		uint8_t*     color;
		uint32_t     color_size;

		Slice_t() = default;

		__device__ Slice_t(const Slice_t& _x) {
			this->timestamp     = _x.timestamp;
			this->index         = _x.index;
			this->type          = _x.type;
			this->mv            = _x.mv;
			this->size          = _x.size;
			this->qp            = _x.qp;
			this->geometry_size = _x.geometry_size;
			this->color_size    = _x.color_size;

			this->geometry   = _x.geometry;
			this->color_size = _x.color_size;
		}

		__device__ Slice_t& operator=(const Slice_t& _x) {
			this->timestamp     = _x.timestamp;
			this->index         = _x.index;
			this->type          = _x.type;
			this->mv            = _x.mv;
			this->size          = _x.size;
			this->qp            = _x.qp;
			this->geometry_size = _x.geometry_size;
			this->color_size    = _x.color_size;

			this->geometry   = _x.geometry;
			this->color_size = _x.color_size;
			return *this;
		}
	};
}  // namespace common
}  // namespace vvc
#endif
