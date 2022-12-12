/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/09 11:15
 * Last Modified : 2022/12/12 15:20
 *
 */

#ifndef _SEGMENT_H_
#define _SEGMENT_H_

#include "common/exception.h"
#include "common/param.h"
#include "common/stat.h"
#include <float.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <vector>

namespace vvc {
namespace segment {
	/* Base class of point cloud segment */
	class SegmentBase {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr              source_cloud_; /* source point cloud to be segmented */
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> results_;      /* segmentation result */
		std::shared_ptr<common::VVCParam_t>                params_;       /* vvc parameters */
		common::SegmentStat_t                              stat_;         /* statistics */

	  public:
		/* default constructor */
		SegmentBase() = default;

		/* default deconstructor */
		~SegmentBase() = default;

		/*
		 * @description : set source point cloud for segmentation.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src}
		 * @return : {}
		 * */
		void SetSourcePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src);

		/*
		 * @description : get result point clouds from segmentation.
		 * @param : {std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _result}
		 * @return : {}
		 * */
		void GetResultPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _result);

		/*
		 * @description ：set parameters
		 * @param : {std::shared_ptr<common::vvc_param_t> _ptr}
		 * @return : {}
		 * */
		void SetParams(std::shared_ptr<common::VVCParam_t> _ptr);

		/*
		 * @description : log statistics
		 * @return : {}
		 * */
		void Log() const;

		/*
		 * @description : interface of segment
		 * */
		virtual void Segment() = 0;
	};

	/* centroids are decided by the dense */
	class DenseSegment : public SegmentBase {
	  private:
		/*
		 * @description : segment a block into two subblocks along the largest span dimension, use point index of source_cloud_ to represent a point
		 * @param : {std::vector<size_t>& _old_block} block to be segmented
		 * @param : {std::vector<size_t>& _new_block_a} subblock A
		 * @param : {std::vector<size_t>& _new_block_b} subblock B
		 * @return : {}
		 * */
		void BlockSegment(std::vector<size_t>& _old_block, std::vector<size_t>& _new_block_a, std::vector<size_t>& _new_block_b);

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
}  // namespace segment
}  // namespace vvc

#endif
