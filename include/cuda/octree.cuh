#include "cuda/base.cuh"
#ifndef _PVVC_CUDA_OCTREE_CUH_
#define _PVVC_CUDA_OCTREE_CUH_
namespace vvc{
    namespace octree{

        /* Used to check if some bit of uint8_t is 0 or 1 */
	    __device__ static uint8_t NodeValue[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

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
            int index[8];

            /* Points number that this node contains
            * null/0-7/0246/1357/04/15/26/37/0/1/2/3/4/5/6/7 */
            int weight[16];

            /* g_DC/h_xyz/h_xy/h_xy/h_x/h_x/h_x/h_x/g0/g1/g2/g3/g4/g5/g6/g7 */
            common::ColorYUV raht[16];

            /* Default constructor */
            OctreeNode_t() : value{}, index{}, weight{}, raht{} {}

            /* Copy constructor and assign constructor */
            OctreeNode_t(const OctreeNode_t& _x) : value{_x.value} {
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

            OctreeNode_t& operator=(const OctreeNode_t& _x) {
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

            /* Do RAHT in this node, need signals in raht[8] to raht[15], generate signal in raht[0] and coefficients in raht[1] to raht[7]
            * NOTE: Coefficient of raht[idx] is valid only if both NodeWeight[idx][0] and NodeWeight[idx][1] are not zero.
            * */
            void HierarchicalTransform();

            /* Do InvertRAHT in this node, need signal in raht[0] and coefficients in raht[1] to raht[7], genreate signals in raht[8] to raht[15]
            * NOTE: Signals of raht[idx] is valid only if value & NodeValue[idx - 8] is not zero, i.e., this child node has weight > 0
            * */
            void InvertHierarchicalTransform();
        };

        class InvertRAHTOctree{
        public:
            common::PointXYZ                               tree_center_;
            common::PointXYZ                               tree_range_;
            int                                            tree_height_;
            OctreeNode_t**                                  tree_;             /* Octree */
            common::PointXYZ*                               source_cloud_;     /* Common geometry */
            uint8_t*          node_values_;      /* Octree node sequence */
            common::ColorYUV* coefficients_;     /* RAHT result coefficients */
            common::ColorYUV* reference_colors_; /* Reference ColorYUV */
            common::ColorYUV* source_colors_;    /* Result ColorYUV, after invert compensation */
            // common::Slice                                  slice_;            /* Slice to be decoded */

            __device__ void AddPoints(const int _height, const int _index, const common::PointXYZ _center, const common::PointXYZ _range);
        public:
            InvertRAHTOctree() = default;
            ~InvertRAHTOctree() = default;
        public:
            // TODO SetSlice 类型暂无
            // void InvertRAHTOctree::SetSlice(const common::Slice& _slice)；

            // TODO 返回类型 common::Patch
            __device__ void GetPatch() const;

           __device__  void MakeTree();
        };
    }
}
#endif