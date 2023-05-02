/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Definition of Adaptive Run-Length Golomb-Rice Codec and API of Zstandard Codec.
 * Create Time   : 2023/04/18 10:59
 * Last Modified : 2023/04/19 19:08
 *
 */

#ifndef _PVVC_ENTROPY_CODEC_H_
#define _PVVC_ENTROPY_CODEC_H_

#include <math.h>
#include <memory>
#include <numeric>
#include <vector>
#include <zstd.h>

#include "common/exception.h"
#include "common/parameter.h"

namespace vvc {
namespace common {

	/* Unsigned integer */
	typedef uint64_t FIX_INT;
	/* Signed integer */
	typedef int64_t FIX_DATA_INT;

	/* 0b0000111..11 kx1 */
#define __MASK__(k) ((static_cast<FIX_INT>(0x01) << (k)) - 1)

	static int L  = 4;
	static int D1 = 1;
	static int D0 = 1;
	static int U1 = 2;
	static int U0 = 3;
	/* Bit count */
	static int BIT_COUNT_8 = 8;
	/* 64 */
	static int FIX_BIT_COUNT = sizeof(FIX_INT) * 8;
	/* 32 */
	static int HALF_FIX_BIT_COUNT = FIX_BIT_COUNT / 2;

	/*
	 * @description : Change a signed int to unsigned int, _x = {2 * |_x|, _x >= 0} {2 * |_x| - 1, _x < 0}
	 * @param  : {FIX_DATA_INT _x}
	 * @return : {FIX_INT}
	 * */
	inline FIX_INT Sign2Unsign(FIX_DATA_INT _x) {
		FIX_INT ans;
		if (_x < 0) {
			ans = -_x;
			ans = (ans << 1) - 1;
		}
		else {
			ans = _x;
			ans <<= 1;
		}
		return ans;
	}

	/*
	 * @description : Change an unsigned into to signed int, _x = 2y - 1 -> -y, _x = 2y -> y
	 * @param  : {FIX_INT _x}
	 * @return : {FIX_DATA_INT}
	 * */
	inline FIX_DATA_INT Unsign2Sign(FIX_INT _x) {
		FIX_DATA_INT ans = _x >> 1;
		if (_x & 0x01) {
			return -ans - 1;
		}
		else {
			return ans;
		}
	}

	/*
	 * Run Length Golomb Rice Encoder
	 * How to use ?
	 * RLGREncoder enc;
	 * enc.Encode(Your_data);
	 * result = enc.GetResult();
	 * */
	class RLGREncoder {
	  private:
		FIX_INT                               buffer_; /* Bitstream buffer */
		int                                   cnt_;    /* Valid bit count in buffer */
		std::shared_ptr<std::vector<uint8_t>> result_; /* Encoding result */

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

		/*
		 * @description : Write _x into buffer with Golomb-Rice codec, base is pow(2, _bits)
		 * @param  : {FIX_INT _x}
		 * @param  : {int _bits}
		 * */
		void GRWrite(FIX_INT _x, int _bits);

		/*
		 * @description : Reset encoder
		 * */
		inline void Reset() {
			this->result_ = std::make_shared<std::vector<uint8_t>>();
			this->cnt_    = 0;
			this->buffer_ = 0;
		}

	  public:
		/*
		 * @description : Default constructor and deconstructor
		 * */
		RLGREncoder();

		~RLGREncoder() = default;

		/*
		 * @description : According to [1], encode _data
		 * @param  : {std::shared_ptr<std::vector<FIX_DATA_INT>> _data}
		 * @return : {}
		 * */
		void Encode(std::shared_ptr<std::vector<FIX_DATA_INT>> _data);

		/*
		 * @description : Get result, using std::move
		 * @param  : {}
		 * @return : {std::vector<uint8_t>}
		 * */
		std::shared_ptr<std::vector<uint8_t>> GetResult();
	};

	/*
	 * Run-Length Golomb-Rice Decoder
	 * How to use?
	 * RLGRDecoder dec;
	 * dec.Decode(Your_data);
	 * result = dec.GetResult();
	 * */
	class RLGRDecoder {
	  private:
		FIX_INT                                    buffer_;    /* Bitstream buffer */
		int                                        cnt_;       /* Valid bits in buffer */
		std::shared_ptr<std::vector<FIX_DATA_INT>> result_;    /* Decoding result */
		std::vector<uint8_t>::iterator             now_, end_; /* Iterator of decoded data */

		/*
		 * @description : Get data to fill the buffer
		 * */
		void Fill();

		/*
		 * @description : Read 1-bit from buffer_
		 * @param  : {}
		 * @return : {uint8_t}
		 * */
		uint8_t Read();

		/*
		 * @description : Read _bits data from buffer_
		 * @param  : {int _bits}
		 * @return : {FIX_INT}
		 * */
		FIX_INT Read(int _bits);

		/*
		 * @description : Golomb-Rice decode _bits data
		 * @param  : {int _bits}
		 * @return : {FIX_INT}
		 * */
		FIX_INT GRRead(int _bits);

		/*
		 * @description : Check decoded data stream reaches end
		 * @param  : {}
		 * @return : {bool}
		 * */
		inline bool End() const {
			return this->now_ == this->end_;
		}

		/*
		 * @description : Reset decoder
		 * */
		inline void Reset() {
			this->result_ = std::make_shared<std::vector<FIX_DATA_INT>>();
			this->cnt_    = 0;
			this->buffer_ = 0;
		}

	  public:
		/*
		 * @description : Default constructor and deconstructor
		 * */
		RLGRDecoder();

		~RLGRDecoder() = default;

		/*
		 * @description : Decode data, should generate _size decoding result
		 * @param  : {std::shared_ptr<std::vector<uint8_t>> _data}
		 * @param  : {int _size}
		 * @return : {}
		 * */
		void Decode(std::shared_ptr<std::vector<uint8_t>> _data, int _size);

		/*
		 * @description : Get decoding result, using std::move
		 * @return : {std::vector<FIX_DATA_INT>}
		 * */
		std::shared_ptr<std::vector<FIX_DATA_INT>> GetResult();
	};

	class ZstdEncoder {
	  private:
		PVVCParam_t::Ptr                      params_; /* patchVVC parameters */
		std::shared_ptr<std::vector<uint8_t>> result_; /* Encoding result */

	  public:
		/* Default constructor and deconstructor */
		ZstdEncoder();

		~ZstdEncoder() = default;

		/*
		 * @description : Get encoding result
		 * @param  : {}
		 * @return : {std::shared_ptr<std::vector<uint8_t>>}
		 * */
		std::shared_ptr<std::vector<uint8_t>> GetResult() const;

		/*
		 * @description : Set patchVVC parameters
		 * @param  : {PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(PVVCParam_t::Ptr _param);

		/*
		 * @description : Encode data
		 * @param  : {std::shared_ptr<std::vector<uint8_t>> _data}
		 * @return : {}
		 * */
		void Encode(std::shared_ptr<std::vector<uint8_t>> _data);
	};

	class ZstdDecoder {
	  private:
		std::shared_ptr<std::vector<uint8_t>> result_; /* Decoding result */

	  public:
		/* Default constructor and deconstructor */
		ZstdDecoder();

		~ZstdDecoder() = default;

		/*
		 * @description : Get decoding result
		 * @param  : {}
		 * @return : {std::shared_ptr<std::vector<uint8_t>>}
		 * */
		std::shared_ptr<std::vector<uint8_t>> GetResult() const;

		/*
		 * @description : Decode data
		 * @param  : {std::shared_ptr<std::vector<uint8_t>> _data}
		 * @return : {}
		 * */
		void Decode(std::shared_ptr<std::vector<uint8_t>> _data);
	};

	/*
	 * [1] H.S. Malvar "Adapt Run-Length/Golomb-Rice Encoding of Quantized Generalized Gaussian Sources with Unknow Statistics", Data Compression Conference 2006.
	 * */
}  // namespace common
}  // namespace vvc

#endif
