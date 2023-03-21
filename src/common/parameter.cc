/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of parameters.
 * Create Time   : 2022/12/12 16:38
 * Last Modified : 2022/12/27 14:06
 *
 */

#include "common/parameter.h"

using namespace vvc;

void common::SetDefaultParams(common::PVVCParam_t::Ptr _ptr) {
	try {
		if (!_ptr) {
			throw __EXCEPT__(EMPTY_PARAMS);
		}

		/* 0Bxxxxxxx1 brief 0Bxxxxxx1x normal 0Bxxxxx1xx complete */
		_ptr->log_level = 0x03;

		_ptr->patch_num = 100;

		_ptr->thread_num = 30;

		_ptr->icp.centroid_alignment = true;
		_ptr->icp.correspondence_ths = 100.0f;
		_ptr->icp.iteration_ths      = 100;
		_ptr->icp.mse_ths            = 0.01f;
		_ptr->icp.transformation_ths = 1e-6;
	}
	catch (const common::Exception& e) {
		e.Log();
		throw __EXCEPT__(ERROR_OCCURED);
	}
}
