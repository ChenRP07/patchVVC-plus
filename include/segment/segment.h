/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Definition of point cloud segmentation.
 * Create Time   : 2022/12/09 11:15
 * Last Modified : 2022/12/14 17:13
 *
 */

#ifndef _PVVC_SEGMENT_H_
#define _PVVC_SEGMENT_H_

#include "common/common.h"
#include "common/exception.h"
#include "common/parameter.h"
#include "common/statistic.h"
#include "io/ply_io.h"
#include <float.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <vector>

namespace vvc {
namespace segment {
	constexpr float NUM_THS = 1.0f / 5.0f;
	/* Base class of point cloud segment */
	class SegmentBase {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_;         /* source point cloud to be segmented */
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> results_; /* segmentation result */
		common::PVVCParam_t::Ptr params_;                             /* vvc parameters */
		int timestamp_;                                               /* point cloud timestamp */

		void KMeans(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _centroids);

	  public:
		/* default constructor */
		SegmentBase() = default;

		/* default deconstructor */
		~SegmentBase() = default;

		/*
		 * @description : Set source point cloud for segmentation.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src}
		 * @return : {}
		 * */
		void SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src);

		/*
		 * @description : Get result point clouds from segmentation.
		 * It do not use std::move because empty patch should be discarded.
		 * @param  : {}
		 * @return : {std::vector<common::Patch>>}
		 * */
		[[nodiscard]] std::vector<common::Patch> GetResultPatches();

		/*
		 * @description :set parameters
		 * @param : {std::shared_ptr<common::vvc_param_t> _ptr}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _ptr);

		/*
		 * @description :set timestamp
		 * @param : {int _time}
		 * @return : {}
		 * */
		void SetTimeStamp(int _time);

		/*
		 * @description : interface of segment
		 * */
		virtual void Segment() = 0;
	};

	/*
	 * Dense clustering, centroids are decided by the dense
	 *
	 * Using example :
	 *
	 *      DenseSegment exp;
	 *      exp.SetSourcePointCloud(_cloud_ptr);
	 *      exp.SetParams(_params);
	 *      exp.SetTimeStamp(_time);
	 *      exp.Segment();
	 *      auto res = exp.GetResultPointClouds();
	 *
	 * It is worth noting that before call Segment, SetSourcePointCloud and SetParams must be called, and
	 * the expected patches number must be set in a VVCParam_t. Otherwise will generate an exception.
	 * */
	class DenseSegment : public SegmentBase {
	  private:
		using VIntPtr = std::shared_ptr<std::vector<int>>;
		/*
		 * @description : segment a block into two subblocks along the largest span dimension, use point index of source_cloud_ to represent a point
		 * @param : {std::vector<size_t>& _old_block} block to be segmented
		 * @param : {std::vector<size_t>& _new_block_a} subblock A
		 * @param : {std::vector<size_t>& _new_block_b} subblock B
		 * @return : {}
		 * */
		void BlockSegment(std::vector<int>& _old_block, std::vector<int>& _new_block_a, std::vector<int>& _new_block_b);

		void TwoMeans(std::vector<int>& _old_block, std::vector<int>& _new_block_a, std::vector<int>& _new_block_b);

	  public:
		/* default constructor */
		DenseSegment() = default;

		/* default deconstructor */
		~DenseSegment() = default;

		/*
		 * @description : use [1] to segment source_cloud_ to _k patches.
		 * @return : {}
		 * */
		void Segment();
	};

	class RefSegment : public SegmentBase {
	  private:
		std::vector<common::Patch> reference_patches_;

	  public:
		RefSegment() = default;
		~RefSegment() = default;

        void SetRefPatches(std::vector<common::Patch> _patches);

		void Segment();
	};
}  // namespace segment
}  // namespace vvc

#endif
