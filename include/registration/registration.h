/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/26 09:37
 * Last Modified : 2022/12/26 09:37
 *
 */

#ifndef _PVVC_REGISTRATION_H_
#define _PVVC_REGISTRATION_H_

#include "common/exception.h"
#include "common/parameter.h"
#include "common/statistic.h"

#include <mutex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <thread>

namespace vvc {
namespace registration {

	class RegistrationBase {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_; /* target point cloud which is the registration reference */
        common::PVVCParam_t::Ptr    params_;       /* parameters */

	  public:
		/* default constructor and deconstructor */
		RegistrationBase()  = default;
		~RegistrationBase() = default;

		/*
		 * @description : set target point cloud which is the registration reference.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void SetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : get target point cloud which is the registration reference.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void GetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud) const;

		/*
		 * @description : set registration parameters.
		 * @param : {std::shared_ptr<common::VVCParam_t>}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);
	};

	/*
	 * Implementation of Iterative Closest Point.
	 * How to use?
	 * Example:
	 * ICP eg;
	 * eg.SetParams(param_ptr);
	 * eg.SetSourceCloud(source_cloud_ptr)
	 * eg.SetTargetCloud(target_cloud_ptr)
	 * eg.Align()
	 * eg.GetResultCloud(your_result_cloud_ptr)
	 * your_mv = eg.GetMotionVector();
	 * your_mse = eg.GetMSE();
	 *
	 * NOTICE:Make sure SetParams, SetSourceCloud and SetTargetCloud are called before Align.
	 * */
	class ICP : public RegistrationBase {
	  private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_;  /* source point cloud which will be transformed */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud_;  /* transformation result */
		Eigen::Matrix4f                        motion_vector_; /* motion vector */
		float                                  mse_;           /* mean squared error */

		/*
		 * @description : do centroid alignment for source_cloud_ to target_cloud_.
		 * @param : {}
		 * @return : {}
		 * */
		void CentroidAlignment();

		/*
		 * @description : calculate mse from source to target.
		 * @param : {}
		 * @return : {}
		 * */
		float CloudMSE();

	  public:
		/* constructor and default deconstructor */
		ICP();

		~ICP() = default;

		/*
		 * @description : set source point cloud which will be transformed.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : get result point cloud which is transformed by source_cloud_, should be called after Align().
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void GetResultCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : get motion vector, should be called after Align().
		 * @param : {}
		 * @return : {Eigen::Matrix4f} MV
		 * */
		Eigen::Matrix4f GetMotionVector() const;

		/*
		 * @description : get mean squared error, should be called after Align().
		 * @param : {}
		 * @return : {float} mse
		 * */
		float GetMSE() const;

		/*
		 * @description : do iterative closest point.
		 * */
		void Align();
	};

    /* 
     * Parallel implementation of Iterative Closest Point, multi source to single target.
     * How to use?
     * Example:
     * ParallelICP eg;
     * eg.SetParams(param_ptr);
     * eg.SetSourceClouds(source_clouds_ptr_vector);
     * eg.SetTargetCloud(target_cloud_ptr)
     * eg.Align();
     * eg.GetResultClouds(your_result_clouds_ptr_vector);
     * your_mv_vector = eg.GetMotionVectors();
     * your_mse_vector = eg.GetMSEs();
     *
     * NOTICE:Make sure SetParams, SetSourceClouds and SetTargetCloud are called before Align;
     * */
	class ParallelICP : public RegistrationBase {
	  private:
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_clouds_;  /* source point cloud patches which will be transformed */
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> result_clouds_;  /* transformed point cloud patches result */
		std::vector<Eigen::Matrix4f>                        motion_vectors_; /* transformation matrices */
		std::vector<float>                                  mses_;           /* mean squared errors */

		std::mutex         task_mutex_; /* mutex for atomic procession */
		std::queue<size_t> task_queue_; /* task queue */

		common::ParallelICPStat_t stat_; /* statistic */
		/*
		 * @description : do centroid alignment for source_clouds_ to target_cloud_.
		 * @param : {}
		 * @return : {}
		 * */
		void CentroidAlignment();

		/*
		 * @description : calculate mse from source to target.
		 * @param : {}
		 * @return : {}
		 * */
		void CloudMSE();

        /* 
         * @description : task function for each thread.
         * @param : {}
         * @return : {}
         * */
		void Task();

        /*
         * @description : log statistic.
         * @param : {}
         * @return : {}
         * */
		void Log();

	  public:
		/* constructor and default deconstructor */
		ParallelICP() = default;

		~ParallelICP() = default;

		/*
		 * @description : set source point cloud which will be transformed.
		 * @param : {std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>}
		 * @return : {}
		 * */
		void SetSourceClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _clouds);

		/*
		 * @description : get result point cloud which is transformed by source_cloud_, should be called after Align().
		 * @param : {std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>}
		 * @return : {}
		 * */
		void GetResultClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& _clouds);

		/*
		 * @description : get motion vector, should be called after Align().
		 * @param : {}
		 * @return : {std::vector<Eigen::Matrix4f>} MV
		 * */
		std::vector<Eigen::Matrix4f> GetMotionVectors() const;

		/*
		 * @description : get mean squared error, should be called after Align().
		 * @param : {}
		 * @return : {std::vector<float>} mse
		 * */
		std::vector<float> GetMSEs() const;

		/*
		 * @description : do iterative closest point.
		 * */
		void Align();
	};
}  // namespace registration
}  // namespace vvc

#endif
