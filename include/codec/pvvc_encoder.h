/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/26 20:06
 * Last Modified : 2023/04/26 20:06
 *
 */

#ifndef _PVVC_ENCODER_H_
#define _PVVC_ENCODER_H_

#include "common/common.h"
#include "common/parameter.h"
#include "common/statistic.h"

#include "io/patch_io.h"
#include "io/ply_io.h"
#include "io/slice_io.h"

#include "segment/segment.h"

#include "patch/patch.h"

#include <sys/stat.h>
#include <sys/types.h>

namespace vvc {
namespace codec {
	enum RAWFRAMETYPE {
		EMPTY_FRAME,
		FORCE_KEY_FRAME,
		PREDICTIVE_FRAME,
	};

	struct RawFrame {
		/* Frame data */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		/* Timestamp */
		int timestamp;
		/* Frame type */
		RAWFRAMETYPE type;

		RawFrame() : cloud{}, timestamp{-1}, type{RAWFRAMETYPE::EMPTY_FRAME} {}
	};

	struct GoP {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		std::vector<common::Patch>             patches;
		int                                    start, end;
		GoP() : cloud{}, patches{}, start{-1}, end{-1} {}
	};

	/*
	 * Segmentation.
	 * How to use?
	 * PVVCSegmentation exp;
	 * exp.SetParams(_p);
	 * exp.LoadFrames();
	 * exp.Segmentation();
	 * res = exp.GetPatches();
	 * */
	class PVVCSegmentation {
		/* Common module */
	  private:
		common::PVVCParam_t::Ptr params_;
		common::PVVCTime_t       clock_;

	  public:
		/* Constructor and deconstructor */
		PVVCSegmentation();

		~PVVCSegmentation() = default;

		/*
		 * @description : Set parameters.
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/* Data and main module */
	  private:
		/* Raw frames data */
		std::vector<RawFrame> frames_;
		/* Frame patches, 1-dim timestamp, 2-dim index */
		std::shared_ptr<std::vector<std::vector<common::Patch>>> patches_;
		/* Current handled frame index */
		int current_frame_idx_;

	  public:
		/* Load raw frame data from disk. */
		void LoadFrames();

		/* Dense segmentation for K-frame and icp segmentation for P-frame */
		void Segmentation();

		/* TODO: Save patches if check_point enable */
		void SavePatches();

		/* Return results */
		std::shared_ptr<std::vector<std::vector<common::Patch>>> GetPatches();
	};

	class PVVCDeformation {
		/* Common module */
	  private:
		common::PVVCParam_t::Ptr params_;
		common::PVVCTime_t       clock_;

	  public:
		/* Constructor and deconstructor */
		PVVCDeformation();

		~PVVCDeformation() = default;

		/*
		 * @description : Set parameters.
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/* Data module */
	  private:
		std::shared_ptr<std::vector<std::vector<common::Patch>>>    patches_;
		std::shared_ptr<std::vector<std::vector<GoP>>>              gops_;
		std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>> results_;

	  public:
		void SetPatches(std::shared_ptr<std::vector<std::vector<common::Patch>>> _patches);

		/* TODO: Load patches if check_point enable */
		void LoadPatches();

		/* TODO: Save deform patches if check_point enable */
		void SaveDeformPatches();

		std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>> GetResults();

	  private:
		std::vector<patch::PatchFitting> handler_;
		std::vector<std::thread>         threads_;
		std::vector<bool>                handler_data_;
		std::queue<int>                  task_queue_;
		std::mutex                       task_queue_mutex_;
		std::mutex                       log_mutex_;
		int                              current_frame_idx_;

		void Task();

	  public:
		void Deformation();
	};

	class PVVCCompression {
		/* Common module */
	  private:
		common::PVVCParam_t::Ptr params_;
		common::PVVCTime_t       clock_;

	  public:
		/* Constructor and deconstructor */
		PVVCCompression();

		~PVVCCompression() = default;

		/*
		 * @description : Set parameters.
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

	  private:
		std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>>           patches_;
		std::shared_ptr<std::vector<std::vector<common::Slice>>>              slices_;
		std::vector<std::shared_ptr<std::vector<std::vector<common::Slice>>>> results_;

	  public:
		void SetPatches(std::vector<std::shared_ptr<std::vector<std::vector<GoP>>>> _patches);

		void LoadPatches();

		void SaveSlices();

		std::vector<std::shared_ptr<std::vector<std::vector<common::Slice>>>> GetResults();

	  private:
		std::vector<patch::GoPEncoding> handler_;
		std::vector<std::thread>        threads_;
		std::queue<int>                 task_queue_;
		std::mutex                      task_queue_mutex_;
		std::mutex                      log_mutex_;

		void Task();

	  public:
		void Compression();
	};
}  // namespace codec
}  // namespace vvc
#endif
