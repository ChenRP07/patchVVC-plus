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

#ifndef _REGISTRATION_H_
#define _REGISTRATION_H_

#include "common/exception.h"
#include "common/parameter.h"
#include "common/statistic.h"

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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_;
		std::shared_ptr<common::VVCParam_t>    params_;

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
		void SetParams(std::shared_ptr<common::VVCParam_t> _param);
	};

	class ICP : public RegistrationBase {
	  private:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud_;
		Eigen::Matrix4f                        motion_vector_;
		float                                  mse_;

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

    class ParallelICP : public RegistrationBase {
	  private:
          std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_clouds_;
          std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> result_clouds_;
          std::vector<Eigen::Matrix4f> motion_vectors_;
          std::vector<float> mse_;

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
}  // namespace registration
}  // namespace vvc

#endif
