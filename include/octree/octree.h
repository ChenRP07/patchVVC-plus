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
 * Last Modified : 2023/05/02 21:49
 *
 */

#ifndef _PVVC_OCTREE_H_
#define _PVVC_OCTREE_H_

#include "common/common.h"
#include "common/entropy_codec.h"
#include "common/exception.h"
#include "common/parameter.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cfloat>
#include <numeric>

namespace vvc {
namespace octree {

	/*
	 * @description : According to space center, half space range, subspace
	 * position computes the subspace center.
	 * @param  : {pcl::PointXYZ _center} space center
	 * @param  : {pcl::PointXYZ _range} subspace range
	 * @param  : {int _pos} subspace position, must be an integer from 0 to 7
	 * @return : {pcl::PointXYZ} subspace center
	 * */
	extern pcl::PointXYZ SubSpaceCenter(pcl::PointXYZ _center, pcl::PointXYZ _range, int _pos);

	/*
	 * @description : Check all eight subspaces, true if each subspace is
	 * empty or contains points from each patch.
	 * @param  : {std::vector<std::vector<std::vector<int>>>& _subspaces}
	 * @return : {bool}
	 * */
	extern bool CheckSubSpace(std::vector<std::vector<std::vector<int>>>& _subspaces);

	/*
	 * @description : Check space, true if it is empty or contains points from each patch.
	 * @param  : {std::vector<std::vector<int>>& _space}
	 * @return : {bool}
	 * */
	extern bool CheckSubSpace(std::vector<std::vector<int>>& _space);

	/*
	 * @description : Haar transform
	 * @param  : {std::pair<int, int>& _w} weights
	 * @param  : {std::pair<common::ColorYUV, common::ColorYUV>& _g} signals
	 * @return : {std::pair<common::ColorYUV, common::ColorYUV>}
	 * */
	extern std::pair<common::ColorYUV, common::ColorYUV> HaarTransform(std::pair<int, int>& _w, std::pair<common::ColorYUV, common::ColorYUV>& _g);

	/*
	 * @description : Invert haar transform
	 * @param  : {std::pair<int, int>& _w} weights
	 * @param  : {std::pair<common::ColorYUV, common::ColorYUV>& _g} signals
	 * @return : {std::pair<common::ColorYUV, common::ColorYUV>}
	 * */
	extern std::pair<common::ColorYUV, common::ColorYUV> InvertHaarTransform(std::pair<int, int>& _w, std::pair<common::ColorYUV, common::ColorYUV>& _g);

	/* Used to check if some bit of uint8_t is 0 or 1 */
	static uint8_t NodeValue[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

	/* Used to specify two children indexes in haar transform */
	static int NodeWeight[8][2] = {{0, 0}, {2, 3}, {4, 6}, {5, 7}, {8, 12}, {9, 13}, {10, 14}, {11, 15}};

	/* Node of octree */
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

		/* Points number that this node contains
		 * null/0-7/0246/1357/04/15/26/37/0/1/2/3/4/5/6/7 */
		int weight[16];

		/* g_DC/h_xyz/h_xy/h_xy/h_x/h_x/h_x/h_x/g0/g1/g2/g3/g4/g5/g6/g7 */
		common::ColorYUV raht[16];

		/* Default constructor */
		OctreeNode_t() : value{}, index{}, weight{}, raht{} {}

		/* Copy constructor and assign constructor */
		OctreeNode_t(const OctreeNode_t& _x) : value{_x.value}, index{_x.index} {
			for (int i = 0; i < 16; ++i) {
				this->weight[i] = _x.weight[i];
			}

			for (int i = 0; i < 16; ++i) {
				this->raht[i] = _x.raht[i];
			}
		}

		OctreeNode_t& operator=(const OctreeNode_t& _x) {
			this->value = _x.value;
			this->index = _x.index;

			for (int i = 0; i < 16; ++i) {
				this->weight[i] = _x.weight[i];
			}

			for (int i = 0; i < 16; ++i) {
				this->raht[i] = _x.raht[i];
			}
			return *this;
		}

		/* Do RAHT in this node, need signals in raht[8] to raht[15], generate signal in raht[0] and coefficients in raht[1] to raht[7]
		 * NOTE: Coefficient of raht[idx] is valid only if both NodeWeight[idx][0] and NodeWeight[idx][1] are not zero.
		 * */
		void HierarchicalTransform();

		/* Do InvertRAHT in this node, need signal in raht[0] and coefficients in raht[1] to raht[7], genreate signals in raht[8] to raht[15]
		 * NOTE: Signals of raht[idx] is valid only if value & NodeValue[idx - 8] is not zero, i.e., this child node has weight > 0
		 * */
		void InvertHierarchicalTransform();
	};

	/*
	 * @description : Change float3 data _center, _range into 24 uint8_t data, save it with _height into _p
	 * @param  : {pcl::PointXYZ _center}
	 * @param  : {pcl::PointXYZ _range}
	 * @param  : {int _height}
	 * @param  : {std::shared_ptr<std::vector<uint8_t>> _p}
	 * @return : {}
	 * */
	extern void SaveTreeCore(pcl::PointXYZ _center, pcl::PointXYZ _range, int _height, std::shared_ptr<std::vector<uint8_t>> _p);

	/*
	 * @description : Change 25 uint8_t data into 3 float3 and 1 int.
	 * @param  : {pcl::PointXYZ& _center}
	 * @param  : {pcl::PointXYZ& _range}
	 * @param  : {int& _height}
	 * @param  : {uint8_t (&_p)[25]}
	 * @return : {}
	 * */
	extern void LoadTreeCore(pcl::PointXYZ& _center, pcl::PointXYZ& _range, int& _height, uint8_t (&_p)[25]);

	/* Base class of octree, a pure virtual class, need to rewrite MakeTree() */
	class OctreeBase {
	  protected:
		/* Range of this tree, for a cube, is edge */
		pcl::PointXYZ tree_range_;

		/* TODO : Deformed octree, range x, y, z and
		 * deflection angles x, y, z */

		/* Tree center */
		pcl::PointXYZ tree_center_;

		/* Tree height */
		int tree_height_;

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
		virtual ~OctreeBase() = default;

		/*
		 * @description : Set PVVC parameters.
		 * @params : {std::shared_ptr<common::VVCParam_t> _param}
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

	/*
	 * Class RAHTOctree, do octree encoding and RAHT encoding.
	 * How to use?
	 * RAHTOctree tree;
	 * tree.SetParams(param);
	 * tree.SetSourceCloud(cloud_ptr);
	 * tree.MakeTree();
	 * oct_result = tree.GetOctree();
	 * Loop {
	 *     tree.SetSourceColors(colors_ptr);
	 *     tree.RAHT();
	 *     color_result = tree.GetRAHTResult();
	 * }
	 * */
	class RAHTOctree : public OctreeBase {
	  private:
		std::vector<std::vector<OctreeNode_t>>         tree_;          /* Tree nodes */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr         source_cloud_;  /* Geometry of common patch */
		std::shared_ptr<std::vector<common::ColorYUV>> source_colors_; /* YUV colors to be transformed */
		std::shared_ptr<std::vector<common::ColorYUV>> RAHT_result_;   /* RAHT result */

		/*
		 * @description : Add a node in _height layer.
		 * @param  : {std::vector<int>& _points}
		 * @param  : {const int _height}
		 * @param  : {const pcl::PointXYZ _center}
		 * @param  : {const pcl::PointXYZ _range}
		 * @return : {}
		 * */
		void AddNode(std::vector<int>& _points, const int _height, const pcl::PointXYZ _center, const pcl::PointXYZ _range);

	  public:
		/* Default constructor and deconstructor */
		RAHTOctree();

		virtual ~RAHTOctree() = default;

		/*
		 * @description : Set geometry of common patch
		 * @param  : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud}
		 * @return : {}
		 * */
		void SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : Set color YUV which will be transformed
		 * @param  : {std::shared_ptr<std::vector<common::ColorYUV>> _colors}
		 * @return : {}
		 * */
		void SetSourceColors(std::shared_ptr<std::vector<common::ColorYUV>> _colors);

		/*
		 * @description : Output octree by sequence traversal
		 * @param  : {}
		 * @return : {std::shared_ptr<std::vector<uint8_t>>}
		 * */
		std::shared_ptr<std::vector<uint8_t>> GetOctree() const;

		/*
		 * @description : Move color result to return value
		 * @param  : {}
		 * @return : {std::shared_ptr<std::vector<common::ColorYUV>>}
		 * */
		std::shared_ptr<std::vector<common::ColorYUV>> GetRAHTResult();

		/*
		 * @description : Do RAHT
		 * @param  : {}
		 * @return : {}
		 * */
		void RAHT();

		/*
		 * @description : Make octree
		 * @param  : {}
		 * @return : {}
		 * */
		void MakeTree();
	};

	/*
	 * Class InvertRAHTOctree, generate Patch from Slice by reconstructing octree and invert RAHT
	 * How to use?
	 * InvertRAHTOctree exp;
	 * exp.SetParams(params);
	 *
	 * Your loop {
	 *     exp.SetSlice(slice);
	 *     res = exp.GetPatch();
	 * }
	 * It is worth noting that a intra slice must be sent into this class first.
	 * */
	class InvertRAHTOctree : public OctreeBase {
	  private:
		std::vector<std::vector<OctreeNode_t>>         tree_;             /* Octree */
		pcl::PointCloud<pcl::PointXYZ>::Ptr            source_cloud_;     /* Common geometry */
		std::shared_ptr<std::vector<uint8_t>>          node_values_;      /* Octree node sequence */
		std::shared_ptr<std::vector<common::ColorYUV>> coefficients_;     /* RAHT result coefficients */
		std::shared_ptr<std::vector<common::ColorYUV>> reference_colors_; /* Reference ColorYUV */
		std::shared_ptr<std::vector<common::ColorYUV>> source_colors_;    /* Result ColorYUV, after invert compensation */
		common::Slice                                  slice_;            /* Slice to be decoded */

		/*
		 * @description : Back root traversal of octree, compute center and update weight.
		 * @param  : {const int _height}
		 * @param  : {const int _index}
		 * @param  : {const pcl::PointXYZ _center}
		 * @param  : {const pcl::PointXYZ _range}
		 * @return : {}
		 * */
		void AddPoints(const int _height, const int _index, const pcl::PointXYZ _center, const pcl::PointXYZ _range);

	  public:
		/* Default constructor and deconstructor */
		InvertRAHTOctree();

		virtual ~InvertRAHTOctree() = default;

		/*
		 * @description : Set decoded Slice
		 * @param  : {const common::Slice& _slice}
		 * @return : {}
		 * */
		void SetSlice(const common::Slice& _slice);

		/*
		 * @description : Get result Patch
		 * @param  : {}
		 * @return : {common::Patch}
		 * */
		common::Patch GetPatch() const;

	  private:
		/* Do invert RAHT */
		void InvertRAHT();

		/* Create common octree */
		void MakeTree();
	};
}  // namespace octree
}  // namespace vvc
#endif
