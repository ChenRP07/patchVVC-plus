/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Statistic of each processing stage.
 * Create Time   : 2022/12/12 15:20
 * Last Modified : 2022/12/14 11:26
 *
 */

#ifndef _PVVC_STATISTIC_H_
#define _PVVC_STATISTIC_H_

#include "common/exception.h"
#include <algorithm>
#include <float.h>
#include <math.h>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace vvc {
namespace common {

	/* segmentation statistic */
	struct SegmentStat_t {
		std::vector<int> expect_; /* expected patches size */
		std::vector<int> fact_;   /* in fact patches size */
	};

	struct ParallelICPStat_t {
		std::vector<uint8_t>                 converged;
		std::vector<std::pair<float, float>> geo_score, y_score, u_score, v_score;
		float                                min_score(int type, int idx);
		float                                max_score(int type, int idx);
		float                                dev_score(int type, int idx);
		float                                avg_score(int type, int idx);
	};

	struct FittingPatchStat_t {
		std::vector<int>   iters;
		std::vector<float> score;
		std::vector<float> costs;
		std::vector<float> avg_iters;
	};

    struct EncoderStat_t {};
	extern float Deviation(const std::vector<int>& _src);
	extern float Deviation(const std::vector<float>& _src);
}  // namespace common
}  // namespace vvc

#endif
