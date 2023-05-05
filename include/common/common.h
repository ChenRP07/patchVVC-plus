/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/03/17 16:59
 * Last Modified : 2023/03/17 16:59
 *
 */

#ifndef _PVVC_COMMON_H_
#define _PVVC_COMMON_H_

#include "common/exception.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <sys/time.h>
#include <sys/types.h>

namespace vvc {
namespace common {
	/*
	 * From low to high : valid 1 | I 0 P 1 | none 0 skip 1 | none 0 zstd 1 | none 0 zstd 1
	 * */
	enum PVVC_SLICE_TYPE { PVVC_SLICE_TYPE_VALID, PVVC_SLICE_TYPE_PREDICT, PVVC_SLICE_TYPE_SKIP, PVVC_SLICE_TYPE_GEO_ZSTD, PVVC_SLICE_TYPE_COLOR_ZSTD };

	enum PVVC_SLICE_TYPE_CONFIG {
		PVVC_SLICE_TYPE_CONFIG_INVALID,
		PVVC_SLICE_TYPE_CONFIG_VALID,
		PVVC_SLICE_TYPE_CONFIG_INTRA,
		PVVC_SLICE_TYPE_CONFIG_PREDICT,
		PVVC_SLICE_TYPE_CONFIG_NOSKIP,
		PVVC_SLICE_TYPE_CONFIG_SKIP,
		PVVC_SLICE_TYPE_CONFIG_GEO_NOZSTD,
		PVVC_SLICE_TYPE_CONFIG_GEO_ZSTD,
		PVVC_SLICE_TYPE_CONFIG_COLOR_NOZSTD,
		PVVC_SLICE_TYPE_CONFIG_COLOR_ZSTD
	};

	static uint8_t PVVC_SLICE_TYPE_MASK[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

	static uint8_t PVVC_SLICE_TYPE_DEFAULT_INTRA   = 0b00000001;
	static uint8_t PVVC_SLICE_TYPE_DEFAULT_PREDICT = 0b00000011;

	inline bool CheckSliceType(uint8_t _type, PVVC_SLICE_TYPE _MASK) {
		return _type & PVVC_SLICE_TYPE_MASK[_MASK];
	}

	inline void SetSliceType(uint8_t& _type, PVVC_SLICE_TYPE_CONFIG _MASK) {
		if (_MASK & 0x01) {
			_type |= PVVC_SLICE_TYPE_MASK[_MASK >> 1];
		}
		else {
			_type &= (~PVVC_SLICE_TYPE_MASK[_MASK >> 1]);
		}
	}

	/* Three channels color implementation, Y/Cb/Cr or Y/U/V */
	struct ColorYUV {
		float y, u, v; /* Channels */

		/* Default constructor */
		ColorYUV() : y{}, u{}, v{} {}

		/* Copy constructor */
		ColorYUV(const ColorYUV& _x) : y{_x.y}, u{_x.u}, v{_x.v} {}

		/* Assign constructor */
		ColorYUV& operator=(const ColorYUV& _x) {
			this->y = _x.y, this->u = _x.u, this->v = _x.v;
			return *this;
		}

		/* Construct by data of three channels */
		ColorYUV(float _r, float _g, float _b, bool _type = true) {
			if (_type) {
				this->y = _r, this->u = _g, this->v = _b;
			}
			else {
				this->y = _r * 0.299f + _g * 0.587f + _b * 0.114f;
				this->u = _r * -0.168736f - _g * 0.331264f + _b * 0.5f + 128;
				this->v = _r * 0.5f - _g * 0.418688f - _b * 0.081312f + 128;
			}
		}
		/* Construct by a RGB format color, reference BT601 */
		ColorYUV(const pcl::PointXYZRGB& _p) {
			this->y = _p.r * 0.299f + _p.g * 0.587f + _p.b * 0.114f;
			this->u = _p.r * -0.168736f - _p.g * 0.331264f + _p.b * 0.5f + 128;
			this->v = _p.r * 0.5f - _p.g * 0.418688f - _p.b * 0.081312f + 128;
		}

		/* Assign constructor by a RGB format color, reference BT601 */
		ColorYUV& operator=(const pcl::PointXYZRGB& _p) {
			this->y = _p.r * 0.299f + _p.g * 0.587f + _p.b * 0.114f;
			this->u = _p.r * -0.168736f - _p.g * 0.331264f + _p.b * 0.5f + 128;
			this->v = _p.r * 0.5f - _p.g * 0.418688f - _p.b * 0.081312f + 128;
			return *this;
		}

		/* Add data with _x relative channel */
		ColorYUV& operator+=(const ColorYUV& _x) {
			this->y += _x.y, this->u += _x.u, this->v += _x.v;
			return *this;
		}

		/* Change _p into ColorYUV and add to this */
		ColorYUV& operator+=(const pcl::PointXYZRGB& _p) {
			ColorYUV _x(_p);
			this->y += _x.y, this->u += _x.u, this->v += _x.v;
			return *this;
		}

		/* Divide data by _x */
		ColorYUV& operator/=(const int _x) {
			this->y /= _x, this->u /= _x, this->v /= _x;
			return *this;
		}

		/* Multiple data by _x */
		ColorYUV& operator*=(const float _x) {
			this->y *= _x, this->u *= _x, this->v *= _x;
			return *this;
		}

		/* Add _x to this, return result */
		ColorYUV operator+(const ColorYUV& _x) const {
			ColorYUV result;
			result.y = this->y + _x.y, result.u = this->u + _x.u, result.v = this->v + _x.v;
			return result;
		}

		/* Multiple this by _x, return result */
		ColorYUV operator*(const float _x) const {
			ColorYUV result;
			result.y = this->y * _x, result.u = this->u * _x, result.v = this->v * _x;
			return result;
		}

		/* Convert to a RGB format, according to BT.601 */
		void ConvertRGB(pcl::PointXYZRGB& _p) {
			_p.r = static_cast<uint8_t>(std::round(this->y + 1.4020f * this->v));
			_p.g = static_cast<uint8_t>(std::round(this->y - 0.3441f * this->u - 0.7141f * this->v));
			_p.b = static_cast<uint8_t>(std::round(this->y + 1.7720f * this->u));
		}
	};

	/* Source patch */
	struct Patch {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;     /* Source point cloud patch */
		int                                    timestamp; /* Patch time stamp */
		int                                    index;     /* Patch index in point cloud frame */
		Eigen::Matrix4f                        mv;        /* Motion vector */

		/* Default constructor */
		Patch() : cloud{nullptr}, timestamp{-1}, index{-1}, mv{Eigen::Matrix4f::Identity()} {}

		/* Copy constructor */
		Patch(const Patch& _p) : cloud{_p.cloud}, timestamp{_p.timestamp}, index{_p.index}, mv{_p.mv} {}

		/* Assign constructor */
		Patch& operator=(const Patch& _p) {
			this->cloud     = _p.cloud;
			this->timestamp = _p.timestamp;
			this->index     = _p.index;
			this->mv        = _p.mv;
			return *this;
		}

		/*
		 * @description : Overload dereference operator, return reference of member cloud.
		 * @param  : {}
		 * @return : {pcl::PointCloud<pcl::PointXYZRGB>&}
		 *  */
		pcl::PointCloud<pcl::PointXYZRGB>& operator*() {
			try {
				if (!this->cloud) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				return *(this->cloud);
			}
			catch (const common::Exception& e) {
				e.Log();
				throw __EXCEPT__(ERROR_OCCURED);
			}
		}

		/*
		 * @description : Overload subscript operator, return reference of element in member cloud.
		 * @param  : {}
		 * @return : {pcl::PointXYZRGB&}
		 * */
		pcl::PointXYZRGB& operator[](int idx) {
			try {
				if (!this->cloud) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}

				if (idx < 0 || idx >= this->size()) {
					throw __EXCEPT__(OUT_OF_RANGE);
				}

				return this->cloud->at(idx);
			}
			catch (const common::Exception& e) {
				e.Log();
				throw __EXCEPT__(ERROR_OCCURED);
			}
		}

		/*
		 * @description : Overload not operator, return true if member cloud is an null pointer.
		 * @param  : {}
		 * @return : {bool}
		 * */
		bool operator!() {
			return this->cloud == nullptr;
		}

		/*
		 * @description : Return true if member cloud point to an empty cloud.
		 * @param  : {}
		 * @return : {}
		 * */
		bool empty() const {
			try {
				if (!this->cloud) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				return this->cloud->empty();
			}
			catch (const common::Exception& e) {
				e.Log();
				throw __EXCEPT__(ERROR_OCCURED);
			}
		}

		/*
		 * @description : Return size of dereference member cloud.
		 * */
		size_t size() const {
			try {
				if (!this->cloud) {
					throw __EXCEPT__(EMPTY_POINT_CLOUD);
				}
				return this->cloud->size();
			}
			catch (const common::Exception& e) {
				e.Log();
				throw __EXCEPT__(ERROR_OCCURED);
			}
		}

		using Ptr = std::shared_ptr<vvc::common::Patch>;
	};

	/* Coded Patch, named Slice */
	struct Slice {
		int                                   timestamp; /* Time stamp */
		int                                   index;     /* Patch index */
		uint8_t                               type;      /* Slice type, intra / inter / TODO:skip / direct */
		Eigen::Matrix4f                       mv;        /* Motion vector */
		size_t                                size;      /* Total point number */
		uint8_t                               qp;        /* Quantization parameter */
		std::shared_ptr<std::vector<uint8_t>> geometry;  /* Compressed geometry, only valid if type is intra */
		std::shared_ptr<std::vector<uint8_t>> color;     /* Compressed color, valid if type is intra or inter */

		Slice() : timestamp{-1}, index{-1}, type{0x00}, mv{Eigen::Matrix4f::Identity()}, size{0}, qp{1}, geometry{}, color{} {}

		Slice(const Slice& _x) : timestamp{_x.timestamp}, index{_x.index}, type{_x.type}, mv{_x.mv}, size{_x.size}, qp{_x.qp}, geometry{_x.geometry}, color{_x.color} {}

		Slice& operator=(const Slice& _x) {
			this->timestamp = _x.timestamp;
			this->index     = _x.index;
			this->type      = _x.type;
			this->geometry  = _x.geometry;
			this->color     = _x.color;
			this->size      = _x.size;
			this->qp        = _x.qp;
			this->mv        = _x.mv;
			return *this;
		}

		void clear() {
			this->timestamp = this->index = -1;
			this->type                    = 0;
			this->mv                      = Eigen::Matrix4f::Identity();
			this->size                    = 0;
			this->qp                      = 1;
			this->geometry.reset();
			this->color.reset();
		}
		using Ptr = std::shared_ptr<vvc::common::Slice>;
	};

	/* Clocker
	 * How to use?
	 * PVVCTime_t clock;
	 * clock.SetTimeBegin();
	 * ...
	 * clock.SetTimeEnd();
	 *
	 * std::cout << clock.GetTimeMs/S() << '\n';
	 * */
	struct PVVCTime_t {
		timeval time[2]; /* Clocker */

		/*
		 * @description : Set clocker time start.
		 * */
		void SetTimeBegin() {
			gettimeofday(&(this->time[0]), nullptr);
		}

		/*
		 * @description : Set clocker time end.
		 * */
		void SetTimeEnd() {
			gettimeofday(&(this->time[1]), nullptr);
		}

		/*
		 * @description : Get time by second.
		 * @param  : {}
		 * @return : {float}
		 * */
		inline float GetTimeS() const {
			return static_cast<float>(this->time[1].tv_sec - this->time[0].tv_sec) + static_cast<float>(this->time[1].tv_usec - this->time[0].tv_usec) / 1000000.0f;
		}

		/*
		 * @description : Get time by millisecond.
		 * @param  : {}
		 * @return : {float}
		 * */
		inline float GetTimeMs() const {
			return (static_cast<float>(this->time[1].tv_sec - this->time[0].tv_sec) + static_cast<float>(this->time[1].tv_usec - this->time[0].tv_usec) / 1000000.0f) * 1000.0f;
		}
	};

	struct Frame {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		int                                    timestamp;
		std::string                            name;
		uint8_t                                check_point_type;

		Frame() : cloud{nullptr}, timestamp{-1}, name{""}, check_point_type{0x00} {}
	};

}  // namespace common
}  // namespace vvc
#endif
