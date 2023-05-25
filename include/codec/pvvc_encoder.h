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

	/* Group of Pictures */
	struct GoP {
		/* Fitting cloud, common geometry */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		/* Patches of this GoP */
		std::vector<common::Patch> patches;
		/* Start timestamp, end timestamp */
		int start, end;

		/* Constructors */
		GoP() : cloud{}, patches{}, start{-1}, end{-1} {}

		GoP(const GoP& _x) : cloud{_x.cloud}, patches{_x.patches}, start{_x.start}, end{_x.end} {}

		GoP& operator=(const GoP& _x) {
			this->cloud = _x.cloud;
			this->patches = _x.patches;
			this->start = _x.start;
			this->end = _x.end;
			return *this;
		}
	};

#ifndef _PVVC_SEGMENT_VERSION_
#	define _PVVC_SEGMENT_VERSION_
// #define _PVVC_230525_
#	define _PVVC_SEGMENT_230525_
#endif

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
		common::PVVCTime_t clock_;

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
		common::PVVCTime_t clock_;

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
		/* 1st dim--Frames/Timestamp, 2nd dim--Patches/Index */
		std::shared_ptr<std::vector<std::vector<common::Patch>>> patches_;
		/* 1st dim--Patches/Index 2nd dim--Frames/Timestamp */
		std::vector<std::vector<GoP>> gops_;

	  public:
		void SetPatches(std::shared_ptr<std::vector<std::vector<common::Patch>>> _patches);

		/* Load patches if check_point enable */
		void LoadPatches();

		/* Save deform patches if check_point enable */
		void SaveDeformPatches();

		std::vector<std::vector<GoP>> GetResults();

	  private:
		std::vector<patch::PatchFitting> handler_;
		std::vector<std::thread> threads_;
		std::vector<bool> handler_data_;
		std::queue<int> task_queue_;
		std::mutex task_queue_mutex_;
		std::mutex log_mutex_;
		int current_frame_idx_;

		void Task();

	  public:
		void Test();
		void Deformation();
	};

	class PVVCCompression {
		/* Common module */
	  private:
		common::PVVCParam_t::Ptr params_;
		common::PVVCTime_t clock_;

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
		std::vector<std::vector<GoP>> gops_;
		std::vector<std::vector<common::Slice>> results_;

	  public:
		void SetGoPs(std::vector<std::vector<GoP>> _gops);

		void LoadGoPs();

		void SaveSlices();

		std::vector<std::vector<common::Slice>> GetResults();

	  private:
		std::vector<std::thread> threads_;
		std::queue<int> task_queue_;
		std::mutex task_queue_mutex_;
		std::mutex log_mutex_;

		void Task();

        void SplitCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _old, decltype(_old) _new_0, decltype(_old) _new_1);
		void SplitGoP(GoP& _g, std::vector<GoP>& _res);

	  public:
		void Compression();
		void Test();
	};

	class PVVCDecompression {
	  private:
		common::PVVCParam_t::Ptr params_;
		common::PVVCTime_t clock_;

	  public:
		/* Constructor and deconstructor */
		PVVCDecompression();

		~PVVCDecompression() = default;

		/*
		 * @description : Set parameters.
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

	  private:
		std::vector<std::vector<common::Slice>> slices_;
		std::vector<std::vector<common::Patch>> patches_;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> results_;

	  public:
		void LoadSlices();

		void SaveClouds();

	  private:
		std::vector<octree::InvertRAHTOctree> handler_;
		std::vector<std::thread> threads_;
		std::queue<int> task_queue_;
		std::mutex task_queue_mutex_;

		void Task(int k);

	  public:
		void Decompression();
	};

	extern void BuildFrames(common::PVVCParam_t::Ptr _param);
}  // namespace codec
}  // namespace vvc
#endif
