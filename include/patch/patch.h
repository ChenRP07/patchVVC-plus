/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/02 16:07
 * Last Modified : 2023/04/02 16:07
 *
 */

#ifndef _PVVC_PATCH_H_
#define _PVVC_PATCH_H_

#include "common/common.h"
#include "common/exception.h"
#include "common/parameter.h"
#include "common/statistic.h"

#include "octree/octree.h"
#include "registration/registration.h"

namespace vvc {
namespace patch {

	/*
	 * Do octree based common patch fitting.
	 * How to use?
	 * PatchFitting exp;
	 * exp.SetParams(param_ptr);
	 * while (exp.AddPatch(patch_exp)) {
	 *     exp.Log();
	 * }
	 * cloud = exp.GetFittingCloud();
	 * Patches = exp.GetSourcePatches();
	 * then do sth...
	 * NOTICE : SetParams must be called firstly.
	 * */
	class PatchFitting {
	  private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr fitting_cloud_;  /* Fitting point cloud */
		std::vector<vvc::common::Patch>        source_patches_; /* Patches to generate fitting cloud */
		vvc::common::PVVCParam_t::Ptr          params_;         /* Parameters */
		vvc::common::FittingPatchStat_t        stat_;           /* Statistic */
		vvc::common::PVVCTime_t                clock_;
		int                                    max_height_;

		/*
		 * @description : Compute fitting_cloud_
		 * */
		void Compute();

		/*
		 * @description : Do clustering in _points, add cluster centroids into fitting_cloud_
		 * @param  : {std::vector<std::vector<int>>& _points} point idx in this
		 * space, first dimension is patch idx, second demension is point idx.
		 * @return : {}
		 * */
		void Clustering(std::vector<std::vector<int>>& _points);

		/*
		 * @description : Split _points into eight sub-spaces, notice _points
		 * will be released by this function.
		 * @param  : {std::vector<std::vector<int>>& _points} point idx in this
		 * space, first dimension is patch idx, second demension is point idx.
		 * @param  : {pcl::PointXYZ _center} space center
		 * @param  : {pcl::PointXYZ _range} space range
		 * @param  : {int _height} current tree height
		 * @return : {}
		 * */
		void Split(std::vector<std::vector<int>>& _points, pcl::PointXYZ _center, pcl::PointXYZ _range, int _height);

		/*
		 * @description : Try to split _points along x/y/z plane.
		 * @param  : {std::vector<std::vector<int>>& _points} point idx in this
		 * space, first dimension is patch idx, second demension is point idx.
		 * @param  : {pcl::PointXYZ _center}
		 * @param  : {std::vector<std::vector<std::vector<int>>>& _result}
		 * @return : {int} -1 None, 0 x, 1 y, 2 z
		 * */
		int PlanarBisection(std::vector<std::vector<int>>& _points, pcl::PointXYZ _center, std::vector<std::vector<std::vector<int>>>& _result);

	  public:
		/* Default constructor and deconstructor */
		PatchFitting();

		~PatchFitting() = default;

		/*
		 * @description : Set patchVVC parameters
		 * @param  : {vvc::common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/*
		 * @description : Try to add new patch into GOP, return true success, false failure
		 * @param  : {common::Patch _patch}
		 * @return : {bool}
		 * */
		bool AddPatch(common::Patch _patch);

		/*
		 * @description : Get fitting point cloud
		 * @param  : {}
		 * @return : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetFittingCloud() const;

		/*
		 * @description : Get source patches, using std::move
		 * @param  : {}
		 * @return : {std::vector<common::Patch>}
		 * */
		std::vector<common::Patch> GetSourcePatches();

		/*
		 * @description : Log information
		 * */
		void Log() const;

		/*
		 * @description : Reset this object
		 * */
		void Clear();
	};

	class GoPEncoding {
	  private:
		octree::RAHTOctree                     tree_;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr fitting_cloud_;
	};
}  // namespace patch
}  // namespace vvc

#endif
