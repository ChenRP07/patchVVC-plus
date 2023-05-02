/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implement of entropy_codec.h, check it for details.
 * Create Time   : 2023/04/18 11:34
 * Last Modified : 2023/04/24 10:56
 *
 */

#include "common/entropy_codec.h"

namespace vvc {
namespace common {

	RLGREncoder::RLGREncoder() : buffer_{0}, cnt_{0}, result_{} {}

	void RLGREncoder::Flush() {
		uint8_t temp;
		while (this->cnt_ >= BIT_COUNT_8) {
			this->cnt_ -= BIT_COUNT_8;
			temp = ((this->buffer_) >> (this->cnt_)) & UINT_LEAST8_MAX;
			this->result_->emplace_back(temp);
		}
	}

	void RLGREncoder::Write(uint8_t _x) {
		this->buffer_ <<= 1;
		this->buffer_ |= (_x & 0x01);
		this->cnt_++;
		this->Flush();
	}

	void RLGREncoder::Write(FIX_INT _x, int _bits) {
		if (_bits > (FIX_BIT_COUNT - BIT_COUNT_8)) {
			this->Write(_x >> (HALF_FIX_BIT_COUNT), _bits - HALF_FIX_BIT_COUNT);
			this->Write(_x & __MASK__(HALF_FIX_BIT_COUNT), HALF_FIX_BIT_COUNT);
			return;
		}
		this->buffer_ = (this->buffer_ << _bits) + _x;
		this->cnt_ += _bits;
		this->Flush();
	}

	void RLGREncoder::GRWrite(FIX_INT _x, int _bits) {
		FIX_INT p = _x >> _bits;

		if (p < HALF_FIX_BIT_COUNT) {
			/* Quotient, 1...10 */
			this->Write(__MASK__(p + 1) - 1, p + 1);

			/* Reminder, lower _bits bit, binary */
			this->Write(_x & __MASK__(_bits), _bits);
		}
		else {
			this->Write(__MASK__(HALF_FIX_BIT_COUNT), HALF_FIX_BIT_COUNT);
			this->Write(_x, HALF_FIX_BIT_COUNT);
		}
	}

	void RLGREncoder::Encode(std::shared_ptr<std::vector<FIX_DATA_INT>> _data) {
		this->Reset();
		FIX_INT u_data;
		FIX_INT k_P  = 0;
		FIX_INT k_RP = 2 * L;
		FIX_INT m    = 0;

		FIX_INT k;
		FIX_INT k_R;
		FIX_INT p;

		for (auto i : *_data) {
			u_data = Sign2Unsign(i);

			k   = k_P / L;
			k_R = k_RP / L;

			/* Run Length coding */
			if (k) {
				if (u_data) {
					--u_data;

					/* End run of 0 */
					this->Write(0);
					this->Write(m, k);
					this->GRWrite(u_data, k_R);

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

					m = 0;
				}
				else {
					/* Continue run of 0 */
					++m;
					if (m == (0x1 << k)) {
						this->Write(1);
						/* Adapt k_P */
						k_P += U1;
						m = 0;
					}
				}
			}
			/* No Run Length coding */
			else {
				this->GRWrite(u_data, k_R);

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

				m = 0;
			}
		}

		if (k && !u_data) {
			this->Write(0);
			this->Write(m, k_P / L);
		}
	}

	std::shared_ptr<std::vector<uint8_t>> RLGREncoder::GetResult() {
		int r = this->cnt_ % BIT_COUNT_8;
		if (r) {
			this->Write(0, BIT_COUNT_8 - r);
		}
		else {
			this->Flush();
		}
		return this->result_;
	}

	RLGRDecoder::RLGRDecoder() : buffer_{0}, cnt_{0}, now_{}, end_{}, result_{} {}

	void RLGRDecoder::Fill() {
		uint8_t data;
		while (this->cnt_ <= (FIX_BIT_COUNT - BIT_COUNT_8)) {
			if (!this->End()) {
				data          = *(this->now_);
				this->now_    = std::next(this->now_, 1);
				this->buffer_ = (this->buffer_ << BIT_COUNT_8) + data;
				this->cnt_ += BIT_COUNT_8;
			}
			else {
				return;
			}
		}
	}

	uint8_t RLGRDecoder::Read() {
		if (this->cnt_ == 0) {
			this->Fill();
		}
		--this->cnt_;
		return (this->buffer_ >> this->cnt_) & 0x1;
	}

	FIX_INT RLGRDecoder::Read(int _bits) {
		if (_bits > (FIX_BIT_COUNT - BIT_COUNT_8)) {
			FIX_INT data = this->Read(_bits - HALF_FIX_BIT_COUNT) << HALF_FIX_BIT_COUNT;
			return data + this->Read(HALF_FIX_BIT_COUNT);
		}
		this->Fill();
		this->cnt_ -= _bits;
		return (this->buffer_ >> this->cnt_) & __MASK__(_bits);
	}

	FIX_INT RLGRDecoder::GRRead(int _bits) {
		FIX_INT p = 0;
		while (this->Read()) {
			p++;
			if (p >= HALF_FIX_BIT_COUNT) {
				return this->Read(HALF_FIX_BIT_COUNT);
			}
		}
		return (p << _bits) + this->Read(_bits);
	}

	void RLGRDecoder::Decode(std::shared_ptr<std::vector<uint8_t>> _data, int _size) {
		this->Reset();
		this->now_ = _data->begin();
		this->end_ = _data->end();
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
					this->result_->emplace_back(0);
					++n;
				}
				if (n >= _size) {
					break;
				}
				u_data = this->GRRead(k_R);
				this->result_->emplace_back(Unsign2Sign(u_data + 1));
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
				this->result_->emplace_back(Unsign2Sign(u_data));
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

	std::shared_ptr<std::vector<FIX_DATA_INT>> RLGRDecoder::GetResult() {
		return this->result_;
	}

	ZstdEncoder::ZstdEncoder() : params_{nullptr}, result_{nullptr} {}

	void ZstdEncoder::SetParams(PVVCParam_t::Ptr _param) {
		try {
			if (!_param) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			this->params_ = _param;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void ZstdEncoder::Encode(std::shared_ptr<std::vector<uint8_t>> _data) {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			size_t buffer_size = ZSTD_compressBound(_data->size());
			this->result_.reset(new std::vector<uint8_t>(buffer_size));
			int z_level        = this->params_->zstd_level;
			z_level            = z_level > ZSTD_maxCLevel() ? ZSTD_maxCLevel() : z_level;
			z_level            = z_level < ZSTD_minCLevel() ? ZSTD_minCLevel() : z_level;
			size_t result_size = ZSTD_compress(this->result_->data(), buffer_size, _data->data(), _data->size(), z_level);
			if (ZSTD_isError(result_size) != 0) {
				throw __EXCEPT__(ZSTD_ERROR);
			}
			this->result_->resize(result_size);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::shared_ptr<std::vector<uint8_t>> ZstdEncoder::GetResult() const {
		try {
			if (!this->result_ || this->result_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			return this->result_;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	ZstdDecoder::ZstdDecoder() : result_{nullptr} {}

	void ZstdDecoder::Decode(std::shared_ptr<std::vector<uint8_t>> _data) {
		try {
			size_t buffer_size = ZSTD_getFrameContentSize(_data->data(), _data->size());

			if (buffer_size == 0 || buffer_size == ZSTD_CONTENTSIZE_UNKNOWN || buffer_size == ZSTD_CONTENTSIZE_ERROR) {
				throw __EXCEPT__(ZSTD_ERROR);
			}
			this->result_.reset(new std::vector<uint8_t>(buffer_size));

			size_t result_size = ZSTD_decompress(this->result_->data(), buffer_size, _data->data(), _data->size());
			if (ZSTD_isError(result_size) != 0) {
				throw __EXCEPT__(ZSTD_ERROR);
			}
			this->result_->resize(result_size);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	std::shared_ptr<std::vector<uint8_t>> ZstdDecoder::GetResult() const {
		try {
			if (!this->result_ || this->result_->empty()) {
				throw __EXCEPT__(EMPTY_RESULT);
			}
			return this->result_;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}
}  // namespace common
}  // namespace vvc
