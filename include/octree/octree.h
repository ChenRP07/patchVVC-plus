/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/03/14 10:21
 * Last Modified : 2023/03/14 17:18
 *
 */

#ifndef _PVVC_OCTREE_H_
#define _PVVC_OCTREE_H_

#include "common/common.h"
#include "common/exception.h"
#include "common/parameter.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cfloat>
#include <vector>

namespace vvc {
namespace octree {

	struct OctreeNode_t {
		/*
		    For a node, eight subnodes are
		      6————4
		     /|   /|
		    2—+——0 |
		    | 7——+-5
		    |/   |/
		    3————1
		    x/y/z > center ? 0 : 1 -> xyz from 000 to 111
		*/
		uint8_t value;

		/* Children indexes in next layer or in real point cloud */
		std::vector<int> index;

		/* Default constructor */
		OctreeNode_t() : value{0x00}, index{0} {}
	};

	class OctreeBase {
	  protected:
		/* Branch nodes of this octree, first dimension
		 * is each layer, second dimension is each node */
		std::vector<std::vector<OctreeNode_t>> tree_nodes_;

		/* Range of this tree, for a cube, is half of edge */
		float tree_range_;

		/* TODO : Deformed octree, range x, y, z and
		 * deflection angles x, y, z */

		/* Tree center */
		pcl::PointXYZ tree_center_;

		/* Tree height */
		size_t tree_height_;

		/* patchVVC parameters */
		common::PVVCParam_t::Ptr params_;

	  public:
		/*
		 * @description : Constructor.
		 * */
		OctreeBase();

		/*
		 * @description : Deconstructor.
		 * */
		~OctreeBase();

		/*
		 * @description : Set PVVC parameters.
		 * @params : std::shared_ptr<common::VVCParam_t> _param
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/*
		 * @description : Pure virtual function, an interface used to make octree.
		 * @params : {}
		 * @return : {}
		 * */
		virtual void MakeTree() = 0;
	};

	class SingleOctree : public OctreeBase {};

	class CommonOctree : public OctreeBase {};

	class FittingOctree : public OctreeBase {
	  private:
		std::vector<pcl::PointXYZ>                          point_geometry_;
		std::vector<std::vector<common::ColorYUV>>          point_color_;
		std::vector<std::vector<int>>                       coefs_;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> encoding_clouds_;
		size_t                                              avg_size_;

        void KMeans(std::vector<std::vector<size_t>>& _point_idx, int _cnt);
		void GeneratePoints(std::vector<std::vector<size_t>>& _point_idx, float _res);

	  public:
		FittingOctree();

		~FittingOctree();

		void AddPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		void MakeTree();
	};
}  // namespace octree
}  // namespace vvc
#endif
