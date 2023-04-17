/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Defination of module registration
 *                 RegistrationBase|->ParallelICP
 *                                 |->ICPBase|->ICP
 *                                           |->NICP
 * Create Time   : 2022/12/26 09:37
 * Last Modified : 2023/03/29 16:21
 *
 */

#ifndef _PVVC_REGISTRATION_H_
#define _PVVC_REGISTRATION_H_

#include "common/common.h"
#include "common/exception.h"
#include "common/parameter.h"
#include "common/statistic.h"

#include <mutex>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <thread>

namespace vvc {
namespace common {
	/*
	 * Calculate geometry color-yuv MSE between two point clouds.
	 * How to use?
	 * MSE m;
	 * m.SetClouds(cloud_x, cloud_y);
	 * m.Compute();
	 * your_mse_pair = m.Get...MSEs();
	 * your_mse_pair = <x error according to y, y error according to x>, -1 means empty cloud
	 * */
	class MSE {
	  private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_, q_;                               /* point clouds */
		std::pair<float, float>                geo_mses_, y_mses_, u_mses_, v_mses_; /* mses */

	  public:
		/* default constructor and deconstructor */
		MSE();
		~MSE() = default;

		/*
		 * @description : Set point clouds
		 * @params : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr _x}
		 * @params : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr _y}
		 * @return : {}
		 * */
		void SetClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _x, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _y);

		/*
		 * @description : Compute MSEs
		 * @params : {}
		 * @return : {}
		 * */
		void Compute();

		/*
		 * @description : Get geometry MSE
		 * @params : {}
		 * @return : {std::pair<float, float>}
		 * */
		std::pair<float, float> GetGeoMSEs() const;

		/*
		 * @description : Get color-Y MSE
		 * @params : {}
		 * @return : {std::pair<float, float>}
		 * */
		std::pair<float, float> GetYMSEs() const;

		/*
		 * @description : Get color-U MSE
		 * @params : {}
		 * @return : {std::pair<float, float>}
		 * */
		std::pair<float, float> GetUMSEs() const;

		/*
		 * @description : Get color-V MSE
		 * @params : {}
		 * @return : {std::pair<float, float>}
		 * */
		std::pair<float, float> GetVMSEs() const;
	};
}  // namespace common

namespace registration {

	/* Base class of all registration class, an abstract class */
	class RegistrationBase {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_; /* target point cloud which is the registration reference */
		common::PVVCParam_t::Ptr               params_;       /* parameters */
		common::PVVCTime_t                     clock_;         /* Clock */

	  public:
		/* default constructor and deconstructor */
		RegistrationBase();

		virtual ~RegistrationBase() = default;

		/*
		 * @description : set target point cloud which is the registration reference.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void SetTargetCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : get target point cloud which is the registration reference.
		 * @param : {}
		 * @return : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetTargetCloud() const;

		/*
		 * @description : set registration parameters.
		 * @param : {std::shared_ptr<common::VVCParam_t>}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/*
		 * @description : interface, instantiate registration algorithm
		 * */
		virtual void Align() = 0;
	};

	/* Base class of single-single icp, an abstract class */
	class ICPBase : public RegistrationBase {
	  protected:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_;  /* Point cloud which will be transformed */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud_;  /* Transformed point cloud */
		Eigen::Matrix4f                        motion_vector_; /* Transformation matrix */
		float                                  mse_;           /* Mean squared error */
		bool                                   converged_;     /* Algorithm converged ? */

		/*
		 * @description : Calculate mse between result_cloud_ and target_cloud_
		 * @param : {}
		 * @return : {float}
		 * */
		float CloudMSE() const;

	  public:
		/* Default constructor and deconstructor*/
		ICPBase();

		virtual ~ICPBase() = default;

		using Ptr = std::shared_ptr<ICPBase>;

		/*
		 * @description : set source point cloud which will be transformed.
		 * @param : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * @return : {}
		 * */
		void SetSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

		/*
		 * @description : get result point cloud which is transformed by source_cloud_, should be called after Align().
		 * @param : {}
		 * @return : {pcl::PointCloud<pcl::PointXYZRGB>::Ptr}
		 * */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetResultCloud() const;

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
		 * @description : get converged_, should be called after Align().
		 * @param : {}
		 * @return : {bool} converged
		 * */
		bool Converged() const;

		/*
		 * @description : interface, set normals
		 * * */
		virtual void SetSourceNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal) {}

		/*
		 * @description : interface, set normals
		 * * */
		virtual void SetTargetNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal) {}

		/*
		 * @description : interface, instantiate registration algorithm
		 * */
		virtual void Align() = 0;
	};

	/*
	 * Implementation of ICP, LM_ICP, GICP.
	 * How to use?
	 * Example:
	 * ICPBase::Ptr icp(new ICP());
	 * icp->SetParams(param_ptr);
	 * icp->SetSourceCloud(source_cloud_ptr);
	 * icp->SetTargetCloud(target_cloud_ptr);
	 * icp->Align();
	 * res_ptr = icp->GetResultCloud();
	 * res_mv = icp->GetMotionVector();
	 * res_mse = icp->GetMSE();
	 *
	 * Make sure SetParams, SetSourceCloud and SetTargetCloud are called before Align.
	 * It is recommended to use a shared_ptr vvc::registration::ICPBase::Ptr to manage.
	 * */
	class ICP : public ICPBase {
	  private:
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp_; /* Algorithm instance */

	  public:
		/* constructor and default deconstructor */
		ICP();

		virtual ~ICP() = default;

		void SetSourceNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal) final {}

		void SetTargetNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal) final {}

		/*
		 * @description : Do ICP algorithm
		 * @param : {}
		 * @return : {}
		 * */
		virtual void Align();
	};

	/*
	 * Implementation of NICP.
	 * How to use?
	 * Example:
	 * ICPBase::Ptr icp(new NICP());
	 * icp->SetParams(param_ptr);
	 * icp->SetSourceCloud(source_cloud_ptr);
	 * icp->SetTargetCloud(target_cloud_ptr);
	 * icp->Align();
	 * res_ptr = icp->GetResultCloud();
	 * res_mv = icp->GetMotionVector();
	 * res_mse = icp->GetMSE();
	 *
	 * Make sure SetParams, SetSource(Target)Cloud, and SetSource(Target)Cloud are called before Align.
	 * It is recommended to use a shared_ptr vvc::registration::ICPBase::Ptr to manage.
	 * */
	class NICP : public ICPBase {
	  private:
		pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr nicp_; /* Algorithm instance */

		pcl::PointCloud<pcl::Normal>::Ptr source_normal_; /* Normals */
		pcl::PointCloud<pcl::Normal>::Ptr target_normal_;

	  public:
		/* Default constructor and deconstructor */
		NICP();

		virtual ~NICP() = default;

		/*
		 * @description : Set normals of source point cloud
		 * @param : {pcl::PointCloud<pcl::PointNormal>::Ptr _normal}
		 * @return : {}
		 * */
		virtual void SetSourceNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal);

		/*
		 * @description : Set normals of target point cloud
		 * @param : {pcl::PointCloud<pcl::PointNormal>::Ptr _normal}
		 * @return : {}
		 * */
		virtual void SetTargetNormal(pcl::PointCloud<pcl::Normal>::Ptr _normal);

		/*
		 * @description : Do NICP algorithm
		 * @param : {}
		 * @return : {}
		 * */
		virtual void Align();
	};

	/*
	 * Parallel implementation of icp, multi source to single target.
	 * How to use?
	 * Example:
	 * ParallelICP picp;
	 * picp.SetParams(param_ptr);
	 * picp.SetSourceClouds(source_clouds_ptr_vector);
	 * picp.SetTargetCloud(target_cloud_ptr)
	 * picp.Align();
	 * picp.GetResultClouds(your_result_clouds_ptr_vector);
	 * res_mv = picp.GetMotionVectors();
	 * res_mse = picp.GetMSEs();
	 *
	 * Make sure SetParams, SetSourceClouds and SetTargetCloud are called before Align;
	 * */
	class ParallelICP : public RegistrationBase {
	  protected:
		std::vector<common::Patch> source_clouds_; /* source point cloud patches which will be transformed */
		std::vector<common::Patch> result_clouds_; /* transformed point cloud patches result */
        std::vector<float> mse_;
        std::vector<bool> converged_;

		std::vector<pcl::PointCloud<pcl::Normal>::Ptr> source_normals_; /* point cloud normals */
		pcl::PointCloud<pcl::Normal>::Ptr              target_normal_;

		std::mutex         task_mutex_; /* mutex for atomic procession */
		std::queue<size_t> task_queue_; /* task queue */

        /* XXX : stat_ is obsolete */
		// common::ParallelICPStat_t stat_; [> statistic <]
        
		/*
		 * @description : do centroid alignment for source_clouds_ to target_cloud_.
		 * @param : {}
		 * @return : {}
		 * */
		void CentroidAlignment();

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

		virtual ~ParallelICP() = default;

		/*
		 * @description : Set source point cloud which will be transformed.
		 * @param : {std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>}
		 * @return : {}
		 * */
		void SetSourceClouds(std::vector<common::Patch>& _clouds);

		/*
		 * @description : Get result point cloud which is transformed by source_cloud_, should be called after Align().
         * It do not use std::move because empty patch should be discarded.
		 * @param : {std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>}
		 * @return : {}
		 * */
		void GetResultClouds(std::vector<common::Patch>& _clouds);

	    std::vector<bool> GetConverge() const;

		/*
		 * @description : do iterative closest point.
		 * */
		virtual void Align();
	};
}  // namespace registration
}  // namespace vvc

#endif
