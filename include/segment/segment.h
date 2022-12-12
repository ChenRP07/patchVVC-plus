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
 * Last Modified : 2022/12/09 18:56
 *
 */

#ifndef _SEGMENT_H_
#define _SEGMENT_H_

#include "common/exception.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <vector>
#include <float.h>

namespace vvc {
namespace segment {
	/* Base class of point cloud segment */
	class segment_base {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr              source_cloud_;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> results_;

	  public:
		/* default constructor */
		segment_base() = default;

		/* default deconstructor */
		~segment_base() = default;

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
		 * @description : interface of segment
		 * */
		virtual void Segment() = 0;
	};

	class dense_segment : public segment_base {
	  private:
		void block_segment(std::vector<size_t>& old_block_, std::vector<size_t>& new_block_a, std::vector<size_t>& new_block_b);

	  public:
		/* default constructor */
		dense_segment() = default;

		/* default deconstructor */
		~dense_segment() = default;

		/*
		 * @description : use [1] to segment source_cloud_ to _k patches.
		 * @pararm : {int _k}
		 * @return : {}
		 * */
		void Segment(int _k);
	};
}  // namespace segment
}  // namespace vvc

#endif
