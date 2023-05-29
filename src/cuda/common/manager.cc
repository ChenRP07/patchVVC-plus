/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/18 17:11
 * Last Modified : 2023/05/19 11:00
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
	float**   temp_mv[MAX_LOAD_FRAME_CNT]{};
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
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].type_gpu, _frame.type, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].size_gpu, _frame.size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].qp_gpu, _frame.qp, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].geometry_size_gpu, _frame.geometry_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].color_size_gpu, _frame.color_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));

		cudaStream_t stream1;
		cudaError_t result;
		result = cudaStreamCreate(&stream1);
		cudaMemcpyAsync(CUDAFrame[index].index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice, stream1);

		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].type_gpu, _frame.type, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].size_gpu, _frame.size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].qp_gpu, _frame.qp, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].geometry_size_gpu, _frame.geometry_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].color_size_gpu, _frame.color_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));

		int* inner_offset_cpu = (int*)malloc(sizeof(int) * _frame.slice_cnt);
		int  inner_offset     = 0;
		for (int i = 0; i < _frame.slice_cnt; i++) {
			// gpuErrchk(cudaMemcpy(temp_mv[index][i], _frame.mv[i], sizeof(float) * 16, cudaMemcpyHostToDevice));
			// gpuErrchk(cudaMemcpy(temp_geo[index][i], _frame.geometry[i], sizeof(uint8_t) * _frame.geometry_size[i], cudaMemcpyHostToDevice));
			// gpuErrchk(cudaMemcpy(temp_color[index][i], _frame.color[i], sizeof(uint8_t) * _frame.color_size[i], cudaMemcpyHostToDevice));
			gpuErrchk(cudaMemcpyAsync(temp_mv[index][i], _frame.mv[i], sizeof(float) * 16, cudaMemcpyHostToDevice,stream1 ));
			gpuErrchk(cudaMemcpyAsync(temp_geo[index][i], _frame.geometry[i], sizeof(uint8_t) * _frame.geometry_size[i], cudaMemcpyHostToDevice,stream1 ));
			gpuErrchk(cudaMemcpyAsync(temp_color[index][i], _frame.color[i], sizeof(uint8_t) * _frame.color_size[i], cudaMemcpyHostToDevice,stream1 ));
			inner_offset_cpu[i] = inner_offset;
			inner_offset += _frame.size[i];
		}
		CUDAFrame[index].point_number = inner_offset;
		// 最后统一的将多个 xxx 的地址拷贝到 mv_gpu 中
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].mv_gpu, temp_mv[index], sizeof(float*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].geometry_gpu, temp_geo[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].color_gpu, temp_color[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));

		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].mv_gpu, temp_mv[index], sizeof(float*) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].geometry_gpu, temp_geo[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].color_gpu, temp_color[index], sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));

		// 将一维数组进行拷贝
		// gpuErrchk(cudaMemcpy(CUDAFrame[index].inner_offset_gpu, inner_offset_cpu, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpyAsync(CUDAFrame[index].inner_offset_gpu, inner_offset_cpu, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice,stream1 ));
		free(inner_offset_cpu);
		CUDAFrame[index].slice_number = _frame.slice_cnt;
	}

	void MallocGpuBuffer(){
		for(int i=0; i<MAX_LOAD_FRAME_CNT; i++){
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

			temp_mv[i]    = (float**)malloc(sizeof(float*) * Manager::PATCH_SIZE);
			temp_geo[i]   = (uint8_t**)malloc(sizeof(uint8_t*) * Manager::PATCH_SIZE);
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
					mem.size  = Manager::MAX_VBO_SIZE - mem.start;
					mem.type  = -1;
					// printf("Add VBO memory garbage type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
					// printf("Add VBO memory garbage type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
					this->AddVBOMem(mem);
					this->unused_start_ = 0;
					this->unused_size_ -= mem.size;
					// printf("Update unused VBO memory [%d , %d]\n", this->unused_start_, this->unused_start_ + this->unused_size_);
				}
				else {
					mem.start = this->unused_start_;
					mem.size  = _size;
					mem.type  = 1;
					this->unused_start_ += _size;
					this->unused_size_ -= _size;
				}
			}
			this->unused_memory_mutex.unlock();
			if (mem.type == 1 && mem.size == _size) {
				// printf("Alloc VBO memory for decoding [%d , %d]\n", mem.start, mem.start + mem.size);
				// printf("Alloc VBO memory for decoding [%d , %d]\n", mem.start, mem.start + mem.size);
				return mem;
			}
		}
	}

	void Manager::Start(int _patch_size, const std::string& _name_prev) {
		Manager::PATCH_SIZE      = _patch_size;
		Manager::frame_name_prev = _name_prev;

        size_t size{1024 * 1024 * 1024};
        cudaDeviceSetLimit(cudaLimitMallocHeapSize, size);
		// printf("Client start, frame from %s, max patch number %d.\n", Manager::frame_name_prev.c_str(), Manager::PATCH_SIZE);

		// 申请 GPU 缓冲队列
		MallocGpuBuffer();
		for(int i=0; i<MAX_LOAD_FRAME_CNT; i++){
			this->unusedGpuBuffer.push(i);
		}
		// printf("Launch thread to load frame ......\n");
		this->frame_loader_ = std::thread(&Manager::StartFrameLoader, this);

		// printf("Initializing Window ...... \n");
		Renderer.InitWindow();
		// printf("Initializing Window success.\n");

		// printf("Initializing OpenGL ...... \n");
		Renderer.InitOpenGL();
		// printf("Initializing OpenGL success.\n");

		// printf("Launch thread to decode frame ......\n");
		this->decoder_ = std::thread(&Manager::StartDecoder, this);

		// printf("Main thread to render frame ......\n");
		// printf("Client working ......\n");

		Renderer.Rendering(0, 0);
		Manager::RENDERED_FRAME_CNT = 0;
		timeval t0, t1;
		gettimeofday(&t0, nullptr);
		while (Manager::RENDERED_FRAME_CNT < TOTAL_FRAME_CNT) {
			VBOMemZone mem = this->GetVBOMem();
			if (mem.type != 1) {
				printf("Unknown error, get a VBO memory type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				exit(1);
			}
			// printf("OpenGL will render frame #%d in VBO [%d , %d].\n", Manager::RENDERED_FRAME_CNT, mem.start, mem.start + mem.size);
			// printf("OpenGL will render frame #%d in VBO [%d , %d].\n", Manager::RENDERED_FRAME_CNT, mem.start, mem.start + mem.size);
			Renderer.Rendering(mem.start, mem.size);
			// printf("OpenGL successfully render frame #%d in VBO [%d, %d].\n", Manager::RENDERED_FRAME_CNT, mem.start, mem.start + mem.size);
			this->ReleaseRender(mem);
			Manager::RENDERED_FRAME_CNT++;
			/* XXX: should only be used in testing */
			// sleep(1);
		}
		gettimeofday(&t1, nullptr);
		printf("render_time = %.2f\n", (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f);

		/* XXX: should only be used in testing */
		// sleep(2);
		// printf("Successfully rendered all frames.\n");
		this->frame_loader_.join();
		this->decoder_.join();
		// printf("Client exit successfully.\n");
	}

	void Manager::StartFrameLoader() {
		printf("Lauch thread to load frame successfully.\n");
		Manager::LOADED_FRAME_CNT = 0;
		timeval total0, total1;
		gettimeofday(&total0, nullptr);
		double total_load_time = 0.0f;
		while (Manager::LOADED_FRAME_CNT < TOTAL_FRAME_CNT) {
			/* Check should load frame */
			int load = -1;

			// this->stream_queue_mutex.lock();
			// /* stream queue should have space to load frame */
			// if (this->stream_.size() < MAX_LOAD_FRAME_CNT) {
			// 	load = true;
			// }
			// this->stream_queue_mutex.unlock();

			this->unusedGpuBufferMutex.lock();
			if(!this->unusedGpuBuffer.empty()){
				load = this->unusedGpuBuffer.front();
				this->unusedGpuBuffer.pop();
			}
			this->unusedGpuBufferMutex.unlock();
			if (load != -1) {
				timeval tt0, tt1;
				/* Load frame */

				gettimeofday(&tt0, nullptr);
				auto        frame_ptr = std::make_shared<common::Frame_t>();
				std::string name      = Manager::frame_name_prev + std::to_string(Manager::LOADED_FRAME_CNT) + ".frame";
				// printf("Try to load frame #%d from %s.\n", Manager::LOADED_FRAME_CNT, name.c_str());
				// printf("Try to load frame #%d from %s.\n", Manager::LOADED_FRAME_CNT, name.c_str());
				if (io::LoadFrame(*frame_ptr, name)) {
					// printf("Load frame %s failed, program exit.\n", name.c_str());
					exit(1);
				}
				// printf("Load frame from %s successfully, slice count %d.\n", name.c_str(), frame_ptr->slice_cnt);
				// this->stream_queue_mutex.lock();
				// this->stream_.push(frame_ptr);
				// printf("Push frame into stream buffer, buffer size is %lu now.\n", this->stream_.size());
				// this->stream_queue_mutex.unlock();
				FrameCpu2Gpu(*frame_ptr, load);
				// gpuErrchk(cudaDeviceSynchronize());
				gettimeofday(&tt1, nullptr);
				this->filledGpuBufferMutex.lock();
				this->filledGpuBuffer.push(load);
				this->filledGpuBufferMutex.unlock();
				Manager::LOADED_FRAME_CNT++;
				double cur_load_time = (tt1.tv_sec - tt0.tv_sec) * 1000.0f + (tt1.tv_usec - tt0.tv_usec) / 1000.0f;
				printf("导入第 %d 帧用时 = %.2f\n", LOADED_FRAME_CNT, cur_load_time);
				total_load_time += cur_load_time;
			}
		}
		gettimeofday(&total1, nullptr);
		printf("数据导入总耗时：= %.2f\n", total_load_time);
		printf("数据导入线程总耗时：= %.2f\n",(total1.tv_sec - total0.tv_sec) * 1000.0f + (total1.tv_usec - total0.tv_usec) / 1000.0f);
		printf("Thread to load frame exit successfully.\n");
	}

	void Manager::StartDecoder() {
		// printf("Lauch thread to decode frame successfully.\n");
		Manager::DECODED_FRAME_CNT = 0;

		// printf("Make OpenGL sharing context.\n");
		Renderer.MakeContextShareWindow();

		octree::InvertRAHTOctree* temp_Decoders = new octree::InvertRAHTOctree[Manager::PATCH_SIZE];
		// printf("Malloc GPU memory ......\n");
		gpuErrchk(cudaMalloc((void**)&(Decoders), sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMemcpy(Decoders, temp_Decoders, sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE, cudaMemcpyHostToDevice));
		delete[] temp_Decoders;

		double decode_total_time = 0.0f;
		double decode_time = 0.0f;
		timeval total0, total1;
		gettimeofday(&total0, nullptr);
		// printf("Malloc GPU memory successfully.\n");
		while (Manager::DECODED_FRAME_CNT < TOTAL_FRAME_CNT) {
			// std::shared_ptr<common::Frame_t> frame_ptr{};
			// this->stream_queue_mutex.lock();
			// if (!this->stream_.empty()) {
			// 	frame_ptr = this->stream_.front();
			// 	this->stream_.pop();
			// }
			// this->stream_queue_mutex.unlock();

			int buffer_index = -1;
			this->filledGpuBufferMutex.lock();
			if(!this->filledGpuBuffer.empty()){
				buffer_index = this->filledGpuBuffer.front();
				this->filledGpuBuffer.pop();
			}
			this->filledGpuBufferMutex.unlock();

			if (buffer_index!=-1) {
				timeval t0, t1, t2, t3;
				// printf("Successfuly get frame #%d from stream buffer, frame point count %d.\n", Manager::DECODED_FRAME_CNT, CUDAFrame[buffer_index].point_number);
				// printf("Transmit frame to GPU ......\n");
				gettimeofday(&t0, nullptr);
				/* Transmit frame to GPU memory */
				// FrameCpu2Gpu(*frame_ptr, MAX_LOAD_FRAME_CNT-1);
				// printf("Transmit frame to GPU successfully.\n");
				// printf("Malloc VBO memory for decoding ......\n");
				VBOMemZone mem = this->AllocVBOMem(CUDAFrame[buffer_index].point_number);
				// printf("Malloc VBO memory for decoding, type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				// printf("Decoding frame ......\n");
				gettimeofday(&t1, nullptr);
				Renderer.CUDADecode(mem.start, 0, CUDAFrame[buffer_index].slice_number, buffer_index);
				gettimeofday(&t2, nullptr);
				// printf("Decoding frame successfully.\n");
				this->AddVBOMem(mem);
				// printf("Add VBO memory into frames queue, type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				gettimeofday(&t3, nullptr);
				printf("decoder#%d_time 解码用时  = %.2f\n",Manager::DECODED_FRAME_CNT , (t2.tv_sec - t1.tv_sec) * 1000.0f + (t2.tv_usec - t1.tv_usec) / 1000.0f);
				printf("decoder#%d_time 输出传输+解码用时 = %.2f\n",Manager::DECODED_FRAME_CNT , (t3.tv_sec - t0.tv_sec) * 1000.0f + (t3.tv_usec - t0.tv_usec) / 1000.0f);
				this->unusedGpuBufferMutex.lock();
				this->unusedGpuBuffer.push(buffer_index);
				this->unusedGpuBufferMutex.unlock();
				Manager::DECODED_FRAME_CNT++;
				decode_time += (t2.tv_sec - t1.tv_sec) * 1000.0f + (t2.tv_usec - t1.tv_usec) / 1000.0f;
				decode_total_time += (t3.tv_sec - t0.tv_sec) * 1000.0f + (t3.tv_usec - t0.tv_usec) / 1000.0f;
			}
		}
		gettimeofday(&total1, nullptr);
		printf("解码总用时  = %.2f\n",decode_time);
		printf("输出传输+解码总用时 = %.2f\n",decode_total_time);
		printf("线程总用时 = %.2f\n",(total1.tv_sec - total0.tv_sec) * 1000.0f + (total1.tv_usec - total0.tv_usec) / 1000.0f);
		printf("Thread to decode frame exit successfully.\n");
	}

	void Manager::ReleaseRender(VBOMemZone _mem) {
		if (_mem.size <= 0) {
			printf("Error, try to release %d VBO memory.\n", _mem.size);
			exit(1);
		}
		this->unused_memory_mutex.lock();
		if (_mem.type == -1) {
			// printf("Garbage collection [%d , %d]\n", _mem.start, _mem.size + _mem.start);
		}
		else if (_mem.type == 1) {
			// printf("Release render memory in VBO [%d , %d]\n", _mem.start, _mem.size + _mem.start);
		}
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
				// printf("OpenGL will render data in VBO [%d , %d]\n", mem.start, mem.start + mem.size);
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
