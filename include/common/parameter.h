/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : VVC parameters definition.
 * Create Time   : 2022/12/12 16:11
 * Last Modified : 2022/12/14 11:26
 *
 */

#ifndef _PVVC_PARAMETER_H_
#define _PVVC_PARAMETER_H_

#include "common/exception.h"
#include <memory>
#include <stdint.h>

namespace vvc {
namespace common {
    enum {
        DENSE_SEGMENT
    };
	struct PVVCParam_t {
		uint8_t log_level; /* quiet brief normal complete */
        struct {
            int num;
            int type;
            int iter;
            int nn;
            float block_num;
        } segment;
		int     thread_num;
		struct {
			float correspondence_ths;
			int   iteration_ths;
			float mse_ths;
			float transformation_ths;
			bool  centroid_alignment;
		} icp;
		struct {
			float resolution;
		} octree;

		using Ptr = std::shared_ptr<PVVCParam_t>;
	};

	/*
	 * @description : set default parameters
	 * @param : {std::shared_ptr<VVCParam_t> _ptr}
	 * @return : {}
	 * */
	extern void SetDefaultParams(PVVCParam_t::Ptr _ptr);
}  // namespace common
}  // namespace vvc

#endif
