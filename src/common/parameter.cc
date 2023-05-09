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
namespace vvc {
namespace common {
	void SetDefaultParams(PVVCParam_t::Ptr& _ptr) {
		try {
			if (_ptr) {
				throw __EXCEPT__(INITIALIZER_ERROR);
			}

			PVVCParam_t p;

			/* 0Bxxxxxxx1 brief 0Bxxxxxx1x normal 0Bxxxxx1xx complete 0Bxxxx1xxx total 0Bxxx1xxxx GoP 0Bxx1xxxxx patch */
			p.log_level = 0x03;

			p.segment.type      = common::DENSE_SEGMENT;
			p.segment.num       = 2048;
			p.segment.nn        = 10;
			p.segment.block_num = 8.0f;

			p.thread_num = 30;
			p.zstd_level = 22;

			p.icp.centroid_alignment = true;
			p.icp.correspondence_ths = 100.0f;
			p.icp.iteration_ths      = 100;
			p.icp.mse_ths            = 0.01f;
			p.icp.transformation_ths = 1e-6;
			p.icp.radius_search_ths  = 10.0f;
			p.icp.type               = common::SIMPLE_ICP;

			p.octree.resolution = 1.0f;

			_ptr = std::make_shared<const PVVCParam_t>(p);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	PVVCParam_t::Ptr CopyParams(PVVCParam_t::Ptr _ptr) {
		try {
			if (!_ptr) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			PVVCParam_t p;

			p.log_level  = _ptr->log_level;
			p.thread_num = _ptr->thread_num;
			p.zstd_level = _ptr->zstd_level;

			p.segment.type      = _ptr->segment.type;
			p.segment.num       = _ptr->segment.num;
			p.segment.nn        = _ptr->segment.nn;
			p.segment.block_num = _ptr->segment.block_num;

			p.icp.centroid_alignment = _ptr->icp.centroid_alignment;
			p.icp.correspondence_ths = _ptr->icp.correspondence_ths;
			p.icp.iteration_ths      = _ptr->icp.iteration_ths;
			p.icp.mse_ths            = _ptr->icp.mse_ths;
			p.icp.transformation_ths = _ptr->icp.transformation_ths;
			p.icp.radius_search_ths  = _ptr->icp.radius_search_ths;
			p.icp.type               = SIMPLE_ICP; 

			return std::make_shared<const PVVCParam_t>(p);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace common
}  // namespace vvc
