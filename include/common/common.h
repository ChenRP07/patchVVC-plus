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

    /* Three channels color implementation, Y/Cb/Cr or Y/U/V */
	struct ColorYUV {
		float y, u, v; /* Channels */

        /* Default constructor */
		ColorYUV() {
			this->y = this->u = this->v = 0.0f;
		}

        /* Copy constructor */
		ColorYUV(const ColorYUV& _x) {
			this->y = _x.y, this->u = _x.u, this->v = _x.v;
		}

        /* Assign constructor */
		ColorYUV& operator=(const ColorYUV& _x) {
			this->y = _x.y, this->u = _x.u, this->v = _x.v;
			return *this;
		}

        /* Construct by data of three channels */
		ColorYUV(float _y, float _u, float _v) {
			this->y = _y, this->u = _u, this->v = _v;
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

        /* Add _x to this, return result */
		ColorYUV operator+(const ColorYUV& _x) const {
			ColorYUV result;
			result.y = this->y + _x.y, result.u = this->u + _x.u, result.v = this->v + _x.v;
			return result;
		}

        /* Divide this by _x, return result */
		ColorYUV operator*(const float _x) const {
			ColorYUV result;
			result.y = this->y * _x, result.u = this->u * _x, result.v = this->v * _x;
			return result;
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
		std::shared_ptr<std::vector<uint8_t>> geometry;  /* Compressed geometry, only valid if type is intra */
		std::shared_ptr<std::vector<uint8_t>> color;     /* Compressed color, valid if type is intra or inter */

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
}  // namespace common
}  // namespace vvc
#endif
