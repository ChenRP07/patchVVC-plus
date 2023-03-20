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

	struct ICPParam_t {
		float correspondence_ths_;
		int   iteration_ths_;
		float mse_ths_;
		float transformation_ths_;
        bool centroid_alignment_;
	};

	/* vvc parameters */
	struct PVVCParam_t {
		uint8_t    log_level_; /* quiet brief normal complete */
		int        patch_num_; /* expected patches number */
		int        thread_num_;
		ICPParam_t icp_;
	};

	/*
	 * @description : set default icp parameters
	 * @param : {std::shared_ptr<VVCParam_t> _ptr}
	 * @return : {}
	 * */
	extern void SetDefaultICPParams(ICPParam_t& _param);

	/*
	 * @description : set default parameters
	 * @param : {std::shared_ptr<VVCParam_t> _ptr}
	 * @return : {}
	 * */
	extern void SetDefaultParams(std::shared_ptr<PVVCParam_t> _ptr);
}  // namespace common
}  // namespace vvc

#endif
