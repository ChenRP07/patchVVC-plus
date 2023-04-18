/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/18 11:34
 * Last Modified : 2023/04/18 11:34
 *
 */

#include "common/entropy_codec.h"

namespace vvc {
namespace common {
	void RLGREncoder::Flush() {
		uint8_t temp;
		while (this->cnt_ >= BIT_COUNT_8) {
			this->cnt_ -= BIT_COUNT_8;
			temp = ((this->buffer_) >> (this->cnt_)) & UINT_LEAST8_MAX;
			this->result_.emplace_back(temp);
		}
	}

	void RLGREncoder::Write(uint8_t _x) {
		this->buffer_ <<= 1;
		this->buffer_ |= (_x & 0x01);
		this->Flush();
	}

    void RLGREncoder::Write(FIX_INT _x, int _bits) {
        if (_bits > (FIX_BIT_COUNT - BIT_COUNT_8)) {
            this->Write(_x >> (FIX_BIT_COUNT / 2), _bits - FIX_BIT_COUNT / 2);
            this->Write(_x & __MASK__(FIX_BIT_COUNT / 2), FIX_BIT_COUNT / 2);
            return;
        }
        this->buffer_ = (this->buffer_ << _bits) + _x;
        this->cnt_ += _bits;
        this->Flush();
    }
}  // namespace common
}  // namespace vvc
