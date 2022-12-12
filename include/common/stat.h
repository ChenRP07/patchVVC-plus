/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/12 15:20
 * Last Modified : 2022/12/12 15:20
 *
 */

#ifndef _STAT_H_
#define _STAT_H_

#include <string>
#include <vector>
#include <math.h>
#include <numeric>
#include <algorithm>

#define __BLACKT__(Msg) "\033[30mMsg\033[0m"
#define __REDT__(Msg) "\033[31mMsg\033[0m"
#define __GREENT__(Msg) "\033[32mMsg\033[0m"
#define __YELLOWT__(Msg) "\033[33mMsg\033[0m"
#define __BLUET__(Msg) "\033[34mMsg\033[0m"
#define __PURPLET__(Msg) "\033[35mMsg\033[0m"
#define __AZURET__(Msg) "\033[36mMsg\033[0m"
#define __WHITET__(Msg) "\033[37mMsg\033[0m"

namespace vvc {
namespace common {

	struct SegmentStat_t {
		std::vector<size_t> expect_; /* expected patches size */
		std::vector<size_t> fact_;   /* in fact patches size */
	};

    extern double Deviation(const std::vector<size_t>& _src);
}  // namespace common
}  // namespace vvc

#endif
