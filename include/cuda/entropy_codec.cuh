typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long      uint64_t;

namespace vvc {
namespace common {
	/* Unsigned integer */
	typedef uint64_t FIX_INT;
	/* Signed integer */
	typedef int64_t FIX_DATA_INT;

	/* 0b0000111..11 kx1 */
#define __MASK__(k) ((static_cast<FIX_INT>(0x01) << (k)) - 1)

	__device__ static int L  = 4;
	__device__ static int D1 = 1;
	__device__ static int D0 = 1;
	__device__ static int U1 = 2;
	__device__ static int U0 = 3;

	/* Bit count */
	__device__ static int BIT_COUNT_8 = 8;
	/* 64 */
	__device__ static int FIX_BIT_COUNT = sizeof(FIX_INT) * 8;
	/* 32 */
	__device__ static int HALF_FIX_BIT_COUNT = sizeof(FIX_INT) * 8 / 2;


	/*
	 * @description : Change a signed int to unsigned int, _x = {2 * |_x|, _x >= 0} {2 * |_x| - 1, _x < 0}
	 * @param  : {FIX_DATA_INT _x}
	 * @return : {FIX_INT}
	 * */
	__device__ inline FIX_INT Sign2Unsign(FIX_DATA_INT _x) {
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
	__device__ inline FIX_DATA_INT Unsign2Sign(FIX_INT _x) {
		FIX_DATA_INT ans = _x >> 1;
		if (_x & 0x01) {
			return -ans - 1;
		}
		else {
			return ans;
		}
	}

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
		FIX_DATA_INT* result_;    /* Decoding result */
		uint8_t             *now_, *end_; /* Iterator of decoded data */
		int 				result_index_;

		/*
		 * @description : Get data to fill the buffer
		 * */
		__device__ void Fill();

		/*
		 * @description : Check decoded data stream reaches end
		 * @param  : {}
		 * @return : {bool}
		 * */
		__device__ inline bool End() const {
			return this->now_ == this->end_;
		}

		/*
		 * @description : Read 1-bit from buffer_
		 * @param  : {}
		 * @return : {uint8_t}
		 * */
		__device__ uint8_t Read();

		/*
		 * @description : Read _bits data from buffer_
		 * @param  : {int _bits}
		 * @return : {FIX_INT}
		 * */
		__device__ FIX_INT Read(int _bits);

		/*
		 * @description : Golomb-Rice decode _bits data
		 * @param  : {int _bits}
		 * @return : {FIX_INT}
		 * */
		__device__ FIX_INT GRRead(int _bits);

		/*
		 * @description : Reset decoder
		 * */
		__device__ inline void Reset(int _size) {
			this->result_ = new FIX_DATA_INT[_size];
			this->cnt_    = 0;
			this->buffer_ = 0;
		}

	  public:
		/*
		 * @description : Default constructor and deconstructor
		 * */
		RLGRDecoder() = default;

		~RLGRDecoder() = default;

		/*
		 * @description : Decode data, should generate _size decoding result
		 * @param  : {std::shared_ptr<std::vector<uint8_t>> _data}
		 * @param  : {int _size}
		 * @return : {}
		 * */
		__device__ void Decode(uint8_t* _data, int _datasize, int _size);

		/*
		 * @description : Get decoding result, using std::move
		 * @return : {std::vector<FIX_DATA_INT>}
		 * */
		__device__ FIX_DATA_INT* GetResult();
	};
}
}