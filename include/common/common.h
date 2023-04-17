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

namespace vvc {
namespace common {

	struct ColorYUV {
		float y, u, v;

		ColorYUV() {
			this->y = this->u = this->v = 0.0f;
		}

		ColorYUV(const ColorYUV& _x) {
			this->y = _x.y, this->u = _x.u, this->v = _x.v;
		}

		ColorYUV& operator=(const ColorYUV& _x) {
			this->y = _x.y, this->u = _x.u, this->v = _x.v;
			return *this;
		}

		ColorYUV(float _y, float _u, float _v) {
			this->y = _y, this->u = _u, this->v = _v;
		}

		ColorYUV(const pcl::PointXYZRGB& _p) {
			this->y = _p.r * 0.299f + _p.g * 0.587f + _p.b * 0.114f;
			this->u = _p.r * -0.168736f - _p.g * 0.331264f + _p.b * 0.5f + 128;
			this->v = _p.r * 0.5f - _p.g * 0.418688f - _p.b * 0.081312f + 128;
		}

		ColorYUV& operator=(const pcl::PointXYZRGB& _p) {
			this->y = _p.r * 0.299f + _p.g * 0.587f + _p.b * 0.114f;
			this->u = _p.r * -0.168736f - _p.g * 0.331264f + _p.b * 0.5f + 128;
			this->v = _p.r * 0.5f - _p.g * 0.418688f - _p.b * 0.081312f + 128;
			return *this;
		}

		ColorYUV& operator+=(const ColorYUV& _x) {
			this->y += _x.y, this->u += _x.u, this->v += _x.v;
			return *this;
		}

		ColorYUV& operator+=(const pcl::PointXYZRGB& _p) {
			ColorYUV _x(_p);
			this->y += _x.y, this->u += _x.u, this->v += _x.v;
			return *this;
		}

		ColorYUV& operator/=(const int _x) {
			this->y /= _x, this->u /= _x, this->v /= _x;
			return *this;
		}

        ColorYUV operator+(const ColorYUV& _x) const {
            ColorYUV result;
            result.y = this->y + _x.y, result.u = this->u + _x.u, result.v = this->v + _x.v;
            return result;
        }

        ColorYUV operator*(const float _x) const {
            ColorYUV result;
            result.y = this->y * _x, result.u = this->u * _x, result.v = this->v * _x;
            return result;
        }
	};

	struct Patch {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		int                                    timestamp;
		int                                    index;
		Eigen::Matrix4f                        mv;

		Patch() : cloud{nullptr}, timestamp{-1}, index{-1}, mv{Eigen::Matrix4f::Identity()} {}

		Patch(const Patch& _p) : cloud{_p.cloud}, timestamp{_p.timestamp}, index{_p.index}, mv{_p.mv} {}

		Patch& operator=(const Patch& _p) {
			this->cloud     = _p.cloud;
			this->timestamp = _p.timestamp;
			this->index     = _p.index;
			this->mv        = _p.mv;
			return *this;
		}

		pcl::PointCloud<pcl::PointXYZRGB> operator*() {
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

		bool operator!() {
			return this->cloud == nullptr;
		}

		bool empty() const {
			return this->cloud->empty();
		}

		size_t size() const {
			return this->cloud->size();
		}

		using Ptr = std::shared_ptr<vvc::common::Patch>;
	};

	struct PVVCTime_t {
		timeval time[2];
		void    SetTimeBegin() {
            gettimeofday(&(this->time[0]), nullptr);
		}
		void SetTimeEnd() {
			gettimeofday(&(this->time[1]), nullptr);
		}
		inline float GetTimeS() const {
			return static_cast<float>(this->time[1].tv_sec - this->time[0].tv_sec) + static_cast<float>(this->time[1].tv_usec - this->time[0].tv_usec) / 1000000.0f;
		}

		inline float GetTimeMs() const {
			return (static_cast<float>(this->time[1].tv_sec - this->time[0].tv_sec) + static_cast<float>(this->time[1].tv_usec - this->time[0].tv_usec) / 1000000.0f) * 1000.0f;
		}
	};
}  // namespace common
}  // namespace vvc
#endif
