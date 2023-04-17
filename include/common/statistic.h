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

#include <algorithm>
#include <math.h>
#include <numeric>
#include <float.h>
#include <string>
#include <vector>
#include <memory>
#include <boost/format.hpp>

/* Colored command line message. */

#define CONTACT3(x, y, z) x##y##z
#define STR(x) #x
#define STR2(x) STR(x)

/* 
 * Examples :
 * __BLACKT__(This is a book.)
 * */
#define __BLACKT__(Msg) vvc::common::color_cmd%30%#Msg
#define __REDT__(Msg) vvc::common::color_cmd%31%#Msg
#define __GREENT__(Msg) vvc::common::color_cmd%32%#Msg
#define __YELLOWT__(Msg) vvc::common::color_cmd%33%#Msg
#define __BLUET__(Msg) vvc::common::color_cmd%34%#Msg
#define __PURPLET__(Msg) vvc::common::color_cmd%35%#Msg
#define __AZURET__(Msg)  vvc::common::color_cmd%36%#Msg
#define __WHITET__(Msg) vvc::common::color_cmd%37%#Msg

namespace vvc {
namespace common {

	/* segmentation statistic */
	struct SegmentStat_t {
		std::vector<int> expect_; /* expected patches size */
		std::vector<int> fact_;   /* in fact patches size */
	};

    struct ParallelICPStat_t {
        std::vector<uint8_t> converged;
        std::vector<std::pair<float, float>> geo_score, y_score, u_score, v_score;
        float min_score(int type, int idx);
        float max_score(int type, int idx);
        float dev_score(int type, int idx);
        float avg_score(int type, int idx);
    };

    struct FittingPatchStat_t {
        std::vector<int> iters;
        std::vector<float> score;
        std::vector<float> costs;
    };
	extern float Deviation(const std::vector<int>& _src);
	extern float Deviation(const std::vector<float>& _src);
    static boost::format color_cmd("\033[%dm%s\033[0m"); 
}  // namespace common
}  // namespace vvc

#endif
