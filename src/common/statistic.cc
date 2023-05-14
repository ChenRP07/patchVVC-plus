/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of statistics.
 * Create Time   : 2022/12/12 19:30
 * Last Modified : 2022/12/14 17:25
 *
 */

#include "common/statistic.h"

using namespace vvc;

float common::ParallelICPStat_t::min_score(int type, int idx) {
	type %= 4;
	idx %= 2;
	std::vector<std::pair<float, float>>* p;
	if (type == 0) {
		p = &(this->geo_score);
	}
	else if (type == 1) {
		p = &(this->y_score);
	}
	else if (type == 2) {
		p = &(this->u_score);
	}
	else if (type == 3) {
		p = &(this->v_score);
	}

	float ans = FLT_MAX;
	for (auto i : *p) {
		if (idx == 0) {
			if (i.first >= 0) {
				ans = std::min(ans, i.first);
			}
		}
		else {
			if (i.second >= 0) {
				ans = std::min(ans, i.second);
			}
		}
	}
	return ans;
}

float common::ParallelICPStat_t::max_score(int type, int idx) {
	type %= 4;
	idx %= 2;
	std::vector<std::pair<float, float>>* p;
	if (type == 0) {
		p = &(this->geo_score);
	}
	else if (type == 1) {
		p = &(this->y_score);
	}
	else if (type == 2) {
		p = &(this->u_score);
	}
	else if (type == 3) {
		p = &(this->v_score);
	}

	float ans = 0.0f;
	for (auto i : *p) {
		if (idx == 0) {
			if (i.first >= 0) {
				ans = std::max(ans, i.first);
			}
		}
		else {
			if (i.second >= 0) {
				ans = std::max(ans, i.second);
			}
		}
	}
	return ans;
}

float common::ParallelICPStat_t::avg_score(int type, int idx) {
	type %= 4;
	idx %= 2;
	std::vector<std::pair<float, float>>* p;
	if (type == 0) {
		p = &(this->geo_score);
	}
	else if (type == 1) {
		p = &(this->y_score);
	}
	else if (type == 2) {
		p = &(this->u_score);
	}
	else if (type == 3) {
		p = &(this->v_score);
	}

	float ans = 0.0f;
	int   cnt = 0;
	for (auto i : *p) {
		if (idx == 0) {
			if (i.first >= 0) {
				ans += i.first;
				cnt++;
			}
		}
		else {
			if (i.second >= 0) {
				ans += i.second;
				cnt++;
			}
		}
	}
	if (cnt == 0) {
		return FLT_MAX;
	}
	return ans / static_cast<float>(cnt);
}

float common::ParallelICPStat_t::dev_score(int type, int idx) {
	type %= 4;
	idx %= 2;
	std::vector<std::pair<float, float>>* p;
	if (type == 0) {
		p = &(this->geo_score);
	}
	else if (type == 1) {
		p = &(this->y_score);
	}
	else if (type == 2) {
		p = &(this->u_score);
	}
	else if (type == 3) {
		p = &(this->v_score);
	}

	float ans = 0.0f;
	int   cnt = 0;
	for (auto i : *p) {
		if (idx == 0) {
			if (i.first >= 0) {
				ans += i.first;
				cnt++;
			}
		}
		else {
			if (i.second >= 0) {
				ans += i.second;
				cnt++;
			}
		}
	}

	if (cnt == 0) {
		return FLT_MAX;
	}

	float mean = ans / static_cast<float>(cnt);
	float var  = 0.0f;
	for (auto i : *p) {
		if (idx == 0) {
			if (i.first >= 0) {
				var += std::pow(i.first - mean, 2);
			}
		}
		else {
			if (i.second >= 0) {
				var += std::pow(i.second - mean, 2);
			}
		}
	}

	var /= static_cast<float>(cnt);
	return std::sqrt(var);
}

float common::Deviation(const std::vector<int>& _src) {
    if (_src.empty()) {
        return FLT_MAX;
    }
	float sum      = std::accumulate(_src.begin(), _src.end(), 0);
	float mean     = sum / _src.size();
	float variance = 0.0;
	std::for_each(_src.begin(), _src.end(), [&](const size_t x) { variance += std::pow(x - mean, 2); });
	variance /= _src.size();
	return std::sqrt(variance);
}

float common::Deviation(const std::vector<float>& _src) {
	float sum = 0.0f;
	int   cnt = 0;
	std::for_each(_src.begin(), _src.end(), [&sum, &cnt](float x) {
		if (x >= 0.0f) {
			sum += x, cnt++;
		}
	});
	if (cnt == 0) {
		return -1.0f;
	}
	float  mean     = sum / static_cast<float>(cnt);
	double variance = 0.0;
	std::for_each(_src.begin(), _src.end(), [&](const float x) {
		if (x >= 0.0f) {
			variance += std::pow(x - mean, 2);
		}
	});
	variance /= static_cast<float>(cnt);
	return std::sqrt(variance);
}
