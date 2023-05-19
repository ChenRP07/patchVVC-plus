/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin
 * Description   :
 * Create Time   : 2023/05/12 18:42
 * Last Modified : 2023/05/12 18:42
 *
 */

#include "cuda/base.cuh"
#include <math.h>

#ifndef _PVVC_CUDA_OCTREE_CUH_
#define _PVVC_CUDA_OCTREE_CUH_

namespace vvc {
namespace client{
namespace octree {

    /*
	 * @description : Invert haar transform
	 * @param  : {std::pair<int, int>& _w} weights
	 * @param  : {std::pair<common::ColorYUV, common::ColorYUV>& _g} signals
	 * @return : {std::pair<common::ColorYUV, common::ColorYUV>}
	 * */
	extern __device__ void InvertHaarTransform(int _w0, int _w1, const common::ColorYUV& _g0, const common::ColorYUV& _g1, common::ColorYUV& _res0, common::ColorYUV& _res1);

    /* Used to check if some bit of uint8_t is 0 or 1 */
    __device__ static uint8_t NodeValue[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

    /* Used to specify two children indexes in haar transform */
    __device__ static int NodeWeight[8][2] = {{0, 0}, {2, 3}, {4, 6}, {5, 7}, {8, 12}, {9, 13}, {10, 14}, {11, 15}};

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

        /* Node center */
        common::PointXYZ center;
        /* Node range */
        common::PointXYZ range;

        /* Children indexes in next layer or in real point cloud */
        int index[8];

        /* Points number that this node contains
        * null/0-7/0246/1357/04/15/26/37/0/1/2/3/4/5/6/7 */
        int weight[16];

        /* g_DC/h_xyz/h_xy/h_xy/h_x/h_x/h_x/h_x/g0/g1/g2/g3/g4/g5/g6/g7 */
        common::ColorYUV raht[16];

        /* Default constructor */
        __device__ OctreeNode_t() : value{}, index{}, weight{}, raht{} {}

            /* Copy constructor and assign constructor */
        __device__ OctreeNode_t(const OctreeNode_t& _x) : value{_x.value} {
            for (int i=0; i<8; i++){
                this->index[i] = _x.index[i];
            }
            for (int i = 0; i < 16; ++i) {
                this->weight[i] = _x.weight[i];
            }

            for (int i = 0; i < 16; ++i) {
                this->raht[i] = _x.raht[i];
            }
        }

        __device__ OctreeNode_t& operator=(const OctreeNode_t& _x) {
            this->value = _x.value;
            for (int i=0; i<8; i++){
                this->index[i] = _x.index[i];
            }

            for (int i = 0; i < 16; ++i) {
                this->weight[i] = _x.weight[i];
            }

            for (int i = 0; i < 16; ++i) {
                this->raht[i] = _x.raht[i];
            }
            return *this;
        }

        // /* Do RAHT in this node, need signals in raht[8] to raht[15], generate signal in raht[0] and coefficients in raht[1] to raht[7]
        // * NOTE: Coefficient of raht[idx] is valid only if both NodeWeight[idx][0] and NodeWeight[idx][1] are not zero.
        // * */
        // __device__ void HierarchicalTransform();

        /* Do InvertRAHT in this node, need signal in raht[0] and coefficients in raht[1] to raht[7], genreate signals in raht[8] to raht[15]
        * NOTE: Signals of raht[idx] is valid only if value & NodeValue[idx - 8] is not zero, i.e., this child node has weight > 0
        * */
        __device__ void InvertHierarchicalTransform();
    };

    /* Layer of octree Node*/
    struct OctreeLayer_t{
        OctreeNode_t*   nodes;
        int             length;

        OctreeLayer_t() = default;
        ~OctreeLayer_t(){
            free(nodes);
        }
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
    class InvertRAHTOctree{
        public:
            common::PointXYZ                               tree_center_;
            common::PointXYZ                               tree_range_;
            int                                            tree_height_;
            OctreeLayer_t*                                  tree_;             /* Octree */
            common::PointXYZ*                               source_cloud_;     /* Common geometry */
            int                                             source_cloud_index_;    /* Index of source_colors_*/
            uint8_t*          node_values_;      /* Octree node sequence */
            common::ColorYUV* coefficients_;     /* RAHT result coefficients */
            common::ColorYUV* reference_colors_; /* Reference ColorYUV */
            common::ColorYUV* source_colors_;    /* Result ColorYUV, after invert compensation */
            common::Slice_t                                  slice_;            /* Slice to be decoded */

        /*
		 * @description : Back root traversal of octree, compute center and update weight.
		 * @param  : {const int _height}
		 * @param  : {const int _index}
		 * @param  : {const pcl::PointXYZ _center}
		 * @param  : {const pcl::PointXYZ _range}
		 * @return : {}
		 * */
		// [[discarded]] __device__ void AddPoints(const int _height, const int _index, const common::PointXYZ _center, const common::PointXYZ _range);

	    public:
            /* Default constructor and deconstructor */
            __host__ __device__ InvertRAHTOctree() : tree_center_{}, tree_range_{}, tree_height_{}, tree_{}, source_cloud_{}, source_cloud_index_{}, node_values_{}, coefficients_{}, reference_colors_{}, source_colors_{}, slice_{} {}
            
            // __device__ ~InvertRAHTOctree();

            /*
            * @description : Set decoded Slice
            * @param  : {const common::Slice& _slice}
            * @return : {}
            * */
            __device__ void SetSlice(const common::Slice_t& _slice);

            /*
            * @description : Get result Patch
            * @param  : {}
            * @return : {common::Patch}
            * */
            __device__ void GetPatch();

        private:
            /* Do invert RAHT */
            __device__ void InvertRAHT();
            
            /* Create common octree */
            __device__  void MakeTree();
    };
}
}
}
#endif
