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

#include "segment/segment.h"

#include "patch/patch.h"

namespace vvc {
namespace codec {
	enum RAWFRAMETYPE {
		FORCE_KEY_FRAME,
		PREDICTIVE_FRAME,
	};

	struct RawFrame {
		/* Frame data */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
		/* Timestamp */
		int timestamp;
		/* Frame type */
		RAWFRAMETYPE type;
	};

	class PVVCCompensation {
	  private:
		std::vector<RawFrame> frames_; /* Raw frames data */
	};
	class PVVCEncoder {
	  private:
		std::vector<common::Frame>              frames_;       /* Clouds to be encoded */
		std::vector<std::vector<common::Patch>> patches_;      /* All patches, first dimension is timestamp, second is index */
		common::PVVCTime_t                      clock_;        /* Clocker */
		common::PVVCParam_t::Ptr                params_;       /* Parameters */
		common::EncoderStat_t                   stat_;         /* Statistic */
		std::vector<patch::GoPEncoding::Ptr>    GoP_encoders_; /* Encoders for each GoP */

		std::mutex                      task_mutex_;
		std::queue<std::pair<int, int>> task_queue_;

		void Task();

	  public:
		/*
		 * @description : Set parameters.
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/*
		 * @description : Load frames from disk.
		 * */
		void LoadFrame();

		/*
		 * @description : Encode.
		 * */
		void Encode();
	};
}  // namespace codec
}  // namespace vvc
#endif
