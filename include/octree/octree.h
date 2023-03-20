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

#include "common/exception.h"
#include "common/parameter.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace vvc {
namespace octree {

	struct OctreeNode_t {
		uint8_t          value;
		std::vector<int> index;
		OctreeNode_t() : value{0x00}, index{0} {}
	};

	class OctreeBase {
	  protected:
		std::vector<std::vector<OctreeNode_t>> tree_nodes_;
		float                                  tree_range_;
		size_t                                 tree_height_;
		std::shared_ptr<common::PVVCParam_t>   params_;

	  public:
		/*
		 * @description : Constructor.
		 * */
		OctreeBase();

		/*
		 * @description : Deconstructor.
		 * */
		~OctreeBase() = default;

		/*
		 * @description : Set PVVC parameters.
		 * @params : std::shared_ptr<common::VVCParam_t> _param
		 * @return : {}
		 * */
		void SetParams(std::shared_ptr<common::PVVCParam_t> _param);

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
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_geometry_;

    }
}  // namespace octree
}  // namespace vvc
#endif
