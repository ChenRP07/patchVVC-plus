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
	enum { DENSE_SEGMENT };
	enum { SIMPLE_ICP, LM_ICP, NORMAL_ICP, GENERAL_ICP };
	enum { PLANAR_BISECTION, PARTIAL_CLUSTERING };
	struct PVVCParam_t {
		uint8_t log_level;       /* quiet brief normal complete */
		uint8_t check_point;     /* from low to high : none first_segment all_segment fitting encoding saving */
		int     thread_num;      /* Launch how many threads to encode */
		int     zstd_level;      /* Zstd encoder level */
		int     max_keyframe;    /* Max key frame interval */
		int     start_timestamp; /* Start timestamp */
		int     time_interval;   /* Interval between two frames */
		int     frames;          /* Encoding frames number */

		/* File path */
		struct {
			std::string source_file; /* Souce point clouds */
			std::string buffer_file; /* Middle results */
			std::string result_file; /* Coding results */
		} io;
		/* Parameters of segmentation */
		struct {
			int   num;       /* Point number in each patch */
			int   type;      /* Segment method */
			int   nn;        /* k in KNN search */
			float block_num; /* Segmentation blocks number in method DENSE_SEGMENT */
		} segment;
		/* Parameters of ICP registration */
		struct {
			float correspondence_ths; /* Max distance of correspondence point pair */
			int   iteration_ths;      /* Max ICP iterations */
			float mse_ths;            /* Max mse difference in two iterations of ICP */
			float transformation_ths; /* Max difference in two transformation of two iterations */
			bool  centroid_alignment; /* Do centroid alignment before ICP? */
			int   type;               /* ICP method */
			float radius_search_ths;  /* Max radius in neighbor search */
		} icp;
		/* Parameters of patch encoding */
		struct {
			uint8_t qp_i, qp_p; /* Quantization parameter of i_patch and p_patch */
		} slice;
		/* Parameters of octree */
		struct {
			float resolution; /* Min resolution of octree, edge length of cube in the last level */
		} octree;
		/* Parameters of patch fitting */
		struct {
			float fitting_ths;       /* Max MSE ths, deciding whether a patch can be fitted */
			int   split_method;      /* Split method in patch fitting */
			float clustering_ths;    /* Max difference between two iteration in clustering */
			int   max_iter;          /* Max clustering iterations */
			int   interpolation_num; /* k in KNN search of color interpolation */
		} patch;

		using Ptr = std::shared_ptr<const PVVCParam_t>;
	};

	/*
	 * @description : Return a pointer to default parameters
	 * @param  : {}
	 * @return : {PVVCParam_t::Ptr}
	 * */
	extern PVVCParam_t::Ptr SetDefaultParams();

	/*
	 * @description : Copy parameters and generate a new instance.
	 * @param  : {PVVCParam_t::Ptr _ptr}
	 * @return : {PVVCParam_t::Ptr}
	 * */
	extern PVVCParam_t::Ptr CopyParams(PVVCParam_t::Ptr _ptr);
}  // namespace common
}  // namespace vvc

#endif
