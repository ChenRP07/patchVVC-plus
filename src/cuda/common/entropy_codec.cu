/*
 * @Author: lixin
 * @Date: 2023-05-13 15:41:53
 * @LastEditTime: 2023-05-13 16:21:38
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "cuda/entropy_codec.cuh"

namespace vvc {
namespace common {

    __device__ void RLGRDecoder::Fill() {
		uint8_t data;
		while (this->cnt_ <= (FIX_BIT_COUNT - BIT_COUNT_8)) {
			if (!this->End()) {
				data          = *(this->now_);
				this->now_ ++;
				this->buffer_ = (this->buffer_ << BIT_COUNT_8) + data;
				this->cnt_ += BIT_COUNT_8;
			}
			else {
				return;
			}
		}
	}

    __device__ uint8_t RLGRDecoder::Read() {
		if (this->cnt_ == 0) {
			this->Fill();
		}
		--this->cnt_;
		return (this->buffer_ >> this->cnt_) & 0x1;
	}
    
    __device__ FIX_INT RLGRDecoder::Read(int _bits) {
		if (_bits > (FIX_BIT_COUNT - BIT_COUNT_8)) {
			FIX_INT data = this->Read(_bits - HALF_FIX_BIT_COUNT) << HALF_FIX_BIT_COUNT;
			return data + this->Read(HALF_FIX_BIT_COUNT);
		}
		this->Fill();
		this->cnt_ -= _bits;
		return (this->buffer_ >> this->cnt_) & __MASK__(_bits);
	}

    __device__ FIX_INT RLGRDecoder::GRRead(int _bits) {
		FIX_INT p = 0;
		while (this->Read()) {
			p++;
			if (p >= HALF_FIX_BIT_COUNT) {
				return this->Read(HALF_FIX_BIT_COUNT);
			}
		}
		return (p << _bits) + this->Read(_bits);
	}

    __device__ void RLGRDecoder::Decode(uint8_t* _data, int _datasize, int _size) {
		this->Reset(_size);
		this->now_ = &_data[0];
		this->end_ = &_data[_datasize];
		FIX_INT u_data;
		FIX_INT k_P  = 0;
		FIX_INT k_RP = 2 * L;
		FIX_INT m    = 0;

		FIX_INT k;
		FIX_INT k_R;
		FIX_INT p;

		int n = 0;
		while (n < _size) {
			k   = k_P / L;
			k_R = k_RP / L;

			/* Run Length coding */
			if (k) {
				m = 0;
				while (this->Read()) {
					m += 0x1 << k;
					k_P += U1;
					k = k_P / L;
				}
				m += this->Read(k);
				while (m--) {
					this->result_[result_index_++] = 0;
					++n;
				}
				if (n >= _size) {
					break;
				}
				u_data = this->GRRead(k_R);
				this->result_[result_index_++] = Unsign2Sign(u_data + 1);
				n++;

				/* Adapt k_RP */
				p = u_data >> k_R;
				if (p) {
					k_RP += p - 1;
					k_RP = k_RP > HALF_FIX_BIT_COUNT * L ? HALF_FIX_BIT_COUNT * L : k_RP;
				}
				else {
					k_RP = k_RP < 2 ? 0 : k_RP - 2;
				}

				/* Adapt k_P */
				k_P = k_P < D1 ? 0 : k_P - D1;
			}
			/* No Run Length coding */
			else {
				u_data = this->GRRead(k_R);
				this->result_[result_index_++] = Unsign2Sign(u_data);
				n++;

				/* Adapt k_RP */
				p = u_data >> k_R;
				if (p) {
					k_RP = k_RP + p - 1;
					k_RP = k_RP > HALF_FIX_BIT_COUNT * L ? HALF_FIX_BIT_COUNT * L : k_RP;
				}
				else {
					k_RP = k_RP < 2 ? 0 : k_RP - 2;
				}

				/* Adapt k_P */
				if (u_data) {
					k_P = k_P < D0 ? 0 : k_P - D0;
				}
				else {
					k_P += U0;
				}
			}
		}
	}

    __device__ FIX_DATA_INT* RLGRDecoder::GetResult() {
		return this->result_;
	}
}
}