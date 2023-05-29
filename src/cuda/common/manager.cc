/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07, Lixin
 * Description   :
 * Create Time   : 2023/05/18 17:11
 * Last Modified : 2023/05/29 16:18
 *
 */

#include "cuda/manager.h"
namespace vvc {
namespace client {
	/* Global GPU memory for decoding */
	CudaFrame_t CUDAFrame[MAX_LOAD_FRAME_CNT]{};

	/* Global GPU memory for decoders */
	octree::InvertRAHTOctree* Decoders{};

	/* Global memory for copying 2-dim array to GPU */
	float** temp_mv[MAX_LOAD_FRAME_CNT]{};
	uint8_t** temp_geo[MAX_LOAD_FRAME_CNT]{};
	uint8_t** temp_color[MAX_LOAD_FRAME_CNT]{};

	/* Global renderer */
	render::Render Renderer{};

	/* Static counter for manager */
	int Manager::LOADED_FRAME_CNT{};
	int Manager::DECODED_FRAME_CNT{};
	int Manager::RENDERED_FRAME_CNT{};
	int Manager::PATCH_SIZE{};

	/* Frame name prev, like "/data/where/name_" */
	std::string Manager::frame_name_prev{};

	void FrameCpu2Gpu(vvc::client::common::Frame_t& _frame, int index) {
		cudaStream_t stream1;
		cudaError_t result;
		result = cudaStreamCreate(&stream1);
		cudaMemcpyAsync(CUDAFrame[index].index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1);

		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].type_gpu, _frame.type, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].size_gpu, _frame.size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].qp_gpu, _frame.qp, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].geometry_size_gpu, _frame.geometry_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].color_size_gpu, _frame.color_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));

		int* inner_offset_cpu = (int*)malloc(sizeof(int) * _frame.slice_cnt);
		int inner_offset = 0;
		for (int i = 0; i < _frame.slice_cnt; i++) {
			gpuErrchk(cudaMemcpyAsync(temp_mv[index][i], _frame.mv[i], sizeof(float) * 16, cudaMemcpyHostToDevice, stream1));
			gpuErrchk(cudaMemcpyAsync(temp_geo[index][i], _frame.geometry[i], sizeof(uint8_t) * _frame.geometry_size[i], cudaMemcpyHostToDevice, stream1));
			gpuErrchk(cudaMemcpyAsync(temp_color[index][i], _frame.color[i], sizeof(uint8_t) * _frame.color_size[i], cudaMemcpyHostToDevice, stream1));
			inner_offset_cpu[i] = inner_offset;
			inner_offset += _frame.size[i];
		}
		CUDAFrame[index].point_number = inner_offset;

		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].mv_gpu, temp_mv[index], sizeof(float*) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].geometry_gpu, temp_geo[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].color_gpu, temp_color[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));

		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].inner_offset_gpu, inner_offset_cpu, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1));
		free(inner_offset_cpu);
		CUDAFrame[index].slice_number = _frame.slice_cnt;
	}

	void MallocGpuBuffer() {
		for (int i = 0; i < MAX_LOAD_FRAME_CNT; i++) {
			/* GPU 申请一维数组空间 */
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].inner_offset_gpu), sizeof(int) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].index_gpu), sizeof(int) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].type_gpu), sizeof(uint8_t) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].qp_gpu), sizeof(uint8_t) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].geometry_size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void**)&(CUDAFrame[i].color_size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));

			/* GPU 申请二维数组空间 */
			gpuErrchk(cudaMalloc((void***)&(CUDAFrame[i].mv_gpu), sizeof(float*) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void***)&(CUDAFrame[i].geometry_gpu), sizeof(uint8_t*) * Manager::PATCH_SIZE));
			gpuErrchk(cudaMalloc((void***)&(CUDAFrame[i].color_gpu), sizeof(uint8_t*) * Manager::PATCH_SIZE));

			temp_mv[i] = (float**)malloc(sizeof(float*) * Manager::PATCH_SIZE);
			temp_geo[i] = (uint8_t**)malloc(sizeof(uint8_t*) * Manager::PATCH_SIZE);
			temp_color[i] = (uint8_t**)malloc(sizeof(uint8_t*) * Manager::PATCH_SIZE);

			for (int j = 0; j < Manager::PATCH_SIZE; ++j) {
				gpuErrchk(cudaMalloc((void**)&(temp_mv[i][j]), sizeof(float) * 16));
				gpuErrchk(cudaMalloc((void**)&(temp_geo[i][j]), sizeof(uint8_t) * MAX_SLICE_SIZE));
				gpuErrchk(cudaMalloc((void**)&(temp_color[i][j]), sizeof(uint8_t) * MAX_SLICE_SIZE));
			}
		}
	}

	void Manager::AddVBOMem(VBOMemZone _mem) {
		if (_mem.size <= 0) {
			printf("Error, try to push %d VBO memory to queue.\n", _mem.size);
			exit(1);
		}
		this->frames_queue_mutex_.lock();
		this->frames_.push(_mem);
		this->frames_queue_mutex_.unlock();
	}

	VBOMemZone Manager::AllocVBOMem(int _size) {
		if (_size <= 0) {
			printf("Error, try to allocate %d VBO memory.\n", _size);
			exit(1);
		}
		while (true) {
			VBOMemZone mem;
			this->unused_memory_mutex.lock();
			if (this->unused_size_ >= _size) {
				int possible_start = this->unused_start_ + _size;
				if (possible_start >= Manager::MAX_VBO_SIZE) {
					mem.start = this->unused_start_;
					mem.size = Manager::MAX_VBO_SIZE - mem.start;
					mem.type = -1;
					this->AddVBOMem(mem);
					this->unused_start_ = 0;
					this->unused_size_ -= mem.size;
				}
				else {
					mem.start = this->unused_start_;
					mem.size = _size;
					mem.type = 1;
					this->unused_start_ += _size;
					this->unused_size_ -= _size;
				}
			}
			this->unused_memory_mutex.unlock();
			if (mem.type == 1 && mem.size == _size) {
				return mem;
			}
		}
	}

	void Manager::Start(int _patch_size, const std::string& _name_prev) {
		Manager::PATCH_SIZE = _patch_size;
		Manager::frame_name_prev = _name_prev;

		size_t size{1024 * 1024 * 1024};
		cudaDeviceSetLimit(cudaLimitMallocHeapSize, size);

		// 申请 GPU 缓冲队列
		MallocGpuBuffer();
		for (int i = 0; i < MAX_LOAD_FRAME_CNT; i++) {
			this->unusedGpuBuffer.push(i);
		}
		this->frame_loader_ = std::thread(&Manager::StartFrameLoader, this);

		Renderer.InitWindow();

		Renderer.InitOpenGL();

		this->decoder_ = std::thread(&Manager::StartDecoder, this);

		Renderer.Rendering(0, 0);
		Manager::RENDERED_FRAME_CNT = 0;
		timeval t0, t1, t2, t3;
		double render_time{};
		gettimeofday(&t0, nullptr);
		while (Manager::RENDERED_FRAME_CNT < TOTAL_FRAME_CNT) {
			VBOMemZone mem = this->GetVBOMem();
			if (mem.type != 1) {
				printf("Unknown error, get a VBO memory type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				exit(1);
			}
			gettimeofday(&t2, nullptr);
			Renderer.Rendering(mem.start, mem.size);
			gettimeofday(&t3, nullptr);
			this->ReleaseRender(mem);
			Manager::RENDERED_FRAME_CNT++;
            render_time += (t3.tv_sec - t2.tv_sec) * 1000.0f + (t3.tv_usec - t2.tv_usec) / 1000.0f;
		}
		gettimeofday(&t1, nullptr);
		printf("Average rendering time : %.2fms\n", render_time / TOTAL_FRAME_CNT);
		printf("Total rendering time : %.2fms\n", (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f);

		this->frame_loader_.join();
		this->decoder_.join();
	}

	void Manager::StartFrameLoader() {
		Manager::LOADED_FRAME_CNT = 0;
		timeval total0, total1;
		gettimeofday(&total0, nullptr);
		double total_load_time = 0.0f;
		while (Manager::LOADED_FRAME_CNT < TOTAL_FRAME_CNT) {
			/* Check should load frame */
			int load = -1;

			this->unusedGpuBufferMutex.lock();
			if (!this->unusedGpuBuffer.empty()) {
				load = this->unusedGpuBuffer.front();
				this->unusedGpuBuffer.pop();
			}
			this->unusedGpuBufferMutex.unlock();
			if (load != -1) {
				timeval tt0, tt1;
				/* Load frame */
				gettimeofday(&tt0, nullptr);
				auto frame_ptr = std::make_shared<common::Frame_t>();
				std::string name = Manager::frame_name_prev + std::to_string(Manager::LOADED_FRAME_CNT) + ".frame";
				if (io::LoadFrame(*frame_ptr, name)) {
					exit(1);
				}
				FrameCpu2Gpu(*frame_ptr, load);
				gettimeofday(&tt1, nullptr);
				this->filledGpuBufferMutex.lock();
				this->filledGpuBuffer.push(load);
				this->filledGpuBufferMutex.unlock();
				Manager::LOADED_FRAME_CNT++;
				double cur_load_time = (tt1.tv_sec - tt0.tv_sec) * 1000.0f + (tt1.tv_usec - tt0.tv_usec) / 1000.0f;
				total_load_time += cur_load_time;
			}
		}
		gettimeofday(&total1, nullptr);
		printf("Average loading time : %.2fms\n", total_load_time / TOTAL_FRAME_CNT);
		printf("Total loading time : %.2fms\n", (total1.tv_sec - total0.tv_sec) * 1000.0f + (total1.tv_usec - total0.tv_usec) / 1000.0f);
	}

	void Manager::StartDecoder() {
		Manager::DECODED_FRAME_CNT = 0;

		Renderer.MakeContextShareWindow();

		octree::InvertRAHTOctree* temp_Decoders = new octree::InvertRAHTOctree[Manager::PATCH_SIZE];
		gpuErrchk(cudaMalloc((void**)&(Decoders), sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMemcpy(Decoders, temp_Decoders, sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE, cudaMemcpyHostToDevice));
		delete[] temp_Decoders;

		double decode_total_time = 0.0f;
		double decode_time = 0.0f;
		timeval total0, total1;
		gettimeofday(&total0, nullptr);
		while (Manager::DECODED_FRAME_CNT < TOTAL_FRAME_CNT) {
			int buffer_index = -1;
			this->filledGpuBufferMutex.lock();
			if (!this->filledGpuBuffer.empty()) {
				buffer_index = this->filledGpuBuffer.front();
				this->filledGpuBuffer.pop();
			}
			this->filledGpuBufferMutex.unlock();

			if (buffer_index != -1) {
				timeval t0, t1, t2, t3;
				VBOMemZone mem = this->AllocVBOMem(CUDAFrame[buffer_index].point_number);
				gettimeofday(&t1, nullptr);
				Renderer.CUDADecode(mem.start, 0, CUDAFrame[buffer_index].slice_number, buffer_index);
				gettimeofday(&t2, nullptr);
				this->AddVBOMem(mem);
				this->unusedGpuBufferMutex.lock();
				this->unusedGpuBuffer.push(buffer_index);
				this->unusedGpuBufferMutex.unlock();
				Manager::DECODED_FRAME_CNT++;
				decode_time += (t2.tv_sec - t1.tv_sec) * 1000.0f + (t2.tv_usec - t1.tv_usec) / 1000.0f;
			}
		}
		gettimeofday(&total1, nullptr);
		printf("Average decoding time : %.2fms\n", decode_time / TOTAL_FRAME_CNT);
		printf("Total decoding time : %.2fms\n", (total1.tv_sec - total0.tv_sec) * 1000.0f + (total1.tv_usec - total0.tv_usec) / 1000.0f);
	}

	void Manager::ReleaseRender(VBOMemZone _mem) {
		if (_mem.size <= 0) {
			printf("Error, try to release %d VBO memory.\n", _mem.size);
			exit(1);
		}
		this->unused_memory_mutex.lock();
		if (_mem.type == -1) {}
		else if (_mem.type == 1) {}
		else {
			printf("Unknown error, try to release VBO memory type %d [%d , %d].\n", _mem.type, _mem.start, _mem.start + _mem.size);
		}
		this->unused_size_ += _mem.size;
		this->unused_memory_mutex.unlock();
	}

	VBOMemZone Manager::GetVBOMem() {
		while (true) {
			VBOMemZone mem{};
			this->frames_queue_mutex_.lock();
			if (!this->frames_.empty()) {
				mem = this->frames_.front();
				this->frames_.pop();
			}
			this->frames_queue_mutex_.unlock();
			if (mem.type == -1) {
				this->ReleaseRender(mem);
			}
			else if (mem.type == 1) {
				return mem;
			}
			else if (mem.type == 0) {
				continue;
			}
			else {
				printf("Unknown error, get a VBO memory type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				exit(1);
			}
		}
	}
}  // namespace client
}  // namespace vvc
