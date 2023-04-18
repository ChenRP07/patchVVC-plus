/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/18 10:59
 * Last Modified : 2023/04/18 10:59
 *
 */

#ifndef _PVVC_ENTROPY_CODEC_H_
#define _PVVC_ENTROPY_CODEC_H_

#include <math.h>
#include <memory>
#include <numeric>
#include <vector>

#include "common/exception.h"
#include "common/parameter.h"

namespace vvc {
namespace common {

#define __MASK__(k) ((static_cast<uint64_t>(0x01) << k) - 1)

	typedef uint64_t FIX_INT;

	static int BIT_COUNT_8   = 8;
	static int FIX_BIT_COUNT = sizeof(FIX_INT) * 8;

	/* Run Length Golomb Rice Coder */
	class RLGREncoder {
	  private:
		PVVCParam_t::Ptr     params_;
		FIX_INT              buffer_;
		int                  cnt_;
		std::vector<uint8_t> result_;

        /*
         * @description : Flush buffer_, output highest 8-bit data.
         * */
		void Flush();

        /*
         * @description : Write 1-bit into buffer_.
         * */
		void Write(uint8_t _x);

        /*
         * @description : Write _bits data into buffer_.
         * */
		void Write(FIX_INT _x, int _bits);
	};
}  // namespace common
}  // namespace vvc

#endif
