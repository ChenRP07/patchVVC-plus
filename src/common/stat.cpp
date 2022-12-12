/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/12 19:30
 * Last Modified : 2022/12/12 19:30
 *
 */

#include "common/stat.h"

using namespace vvc;

double common::Deviation(const std::vector<size_t>& _src) {
	double sum      = std::accumulate(_src.begin(), _src.end(), 0);
	double mean     = sum / _src.size();
	double variance = 0.0;
	std::for_each(_src.begin(), _src.end(), [&](const size_t x) { variance += std::pow(x - mean, 2); });
    variance /= _src.size();
    return std::sqrt(variance);
}
