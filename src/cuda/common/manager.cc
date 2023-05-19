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
	CudaFrame_t CUDAFrame{};

	/* Global GPU memory for decoders */
	octree::InvertRAHTOctree* Decoders{};

	/* Global memory for copying 2-dim array to GPU */
	float**   temp_mv{};
	uint8_t** temp_geo{};
	uint8_t** temp_color{};

	/* Global renderer */
	render::Render Renderer{};

	/* Static counter for manager */
	int Manager::LOADED_FRAME_CNT{};
	int Manager::DECODED_FRAME_CNT{};
	int Manager::RENDERED_FRAME_CNT{};
	int Manager::PATCH_SIZE{};

	/* Frame name prev, like "/data/where/name_" */
	std::string Manager::frame_name_prev{};

	void FrameCpu2Gpu(vvc::client::common::Frame_t& _frame) {
		gpuErrchk(cudaMemcpy(CUDAFrame.index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.type_gpu, _frame.type, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.size_gpu, _frame.size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.qp_gpu, _frame.qp, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.geometry_size_gpu, _frame.geometry_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.color_size_gpu, _frame.color_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));

		int* inner_offset_cpu = (int*)malloc(sizeof(int) * _frame.slice_cnt);
		int  inner_offset     = 0;
		for (int i = 0; i < _frame.slice_cnt; i++) {
			gpuErrchk(cudaMemcpy(temp_mv[i], _frame.mv[i], sizeof(float) * 16, cudaMemcpyHostToDevice));
			gpuErrchk(cudaMemcpy(temp_geo[i], _frame.geometry[i], sizeof(uint8_t) * _frame.geometry_size[i], cudaMemcpyHostToDevice));
			gpuErrchk(cudaMemcpy(temp_color[i], _frame.color[i], sizeof(uint8_t) * _frame.color_size[i], cudaMemcpyHostToDevice));
			inner_offset_cpu[i] = inner_offset;
			inner_offset += _frame.size[i];
		}
		// frame_size = inner_offset;
		// 最后统一的将多个 xxx 的地址拷贝到 mv_gpu 中
		gpuErrchk(cudaMemcpy(CUDAFrame.mv_gpu, temp_mv, sizeof(float*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.geometry_gpu, temp_geo, sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		gpuErrchk(cudaMemcpy(CUDAFrame.color_gpu, temp_color, sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));

		// 将一维数组进行拷贝
		gpuErrchk(cudaMemcpy(CUDAFrame.inner_offset_gpu, inner_offset_cpu, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
		free(inner_offset_cpu);
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
					printf("Add VBO memory garbage type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
					this->AddVBOMem(mem);
					this->unused_start_ = 0;
					this->unused_size_ -= mem.size;
					printf("Update unused VBO memory [%d , %d]\n", this->unused_start_, this->unused_start_ + this->unused_size_);
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
				printf("Allocl VBO memory for decoding [%d , %d]\n", mem.start, mem.start + mem.size);
				return mem;
			}
		}
	}

	void Manager::Start(int _patch_size, const std::string& _name_prev) {
		Manager::PATCH_SIZE      = _patch_size;
		Manager::frame_name_prev = _name_prev;

		printf("Client start, frame from %s, max patch number %d.\n", Manager::frame_name_prev.c_str(), Manager::PATCH_SIZE);

		printf("Launch thread to load frame ......\n");
		this->frame_loader_ = std::thread(&Manager::StartFrameLoader, this);

		printf("Initializing Window ...... \n");
		Renderer.InitWindow();
		printf("Initializing Window success.\n");

		printf("Initializing OpenGL ...... \n");
		Renderer.InitOpenGL();
		printf("Initializing OpenGL success.\n");

		printf("Launch thread to decode frame ......\n");
		this->decoder_ = std::thread(&Manager::StartDecoder, this);

		printf("Main thread to render frame ......\n");
		printf("Client working ......\n");

		Manager::RENDERED_FRAME_CNT = 0;
		while (Manager::RENDERED_FRAME_CNT < TOTAL_FRAME_CNT) {
			VBOMemZone mem = this->GetVBOMem();
			if (mem.type != 1) {
				printf("Unknown error, get a VBO memory type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				exit(1);
			}
			printf("OpenGL will render frame #%d in VBO [%d , %d].\n", Manager::RENDERED_FRAME_CNT, mem.start, mem.start + mem.size);
			Renderer.Rendering(mem.start, mem.size);
			printf("OpenGL successfully render frame #%d in VBO [%d, %d].\n", Manager::RENDERED_FRAME_CNT, mem.start, mem.start + mem.size);
			Manager::RENDERED_FRAME_CNT++;
		}
		this->frame_loader_.join();
		this->decoder_.join();
		printf("Client exit successfully.\n");
	}

	void Manager::StartFrameLoader() {
		printf("Lauch thread to load frame successfully.\n");
		Manager::LOADED_FRAME_CNT = 0;

		while (Manager::LOADED_FRAME_CNT < TOTAL_FRAME_CNT) {
			/* Check should load frame */
			bool load = false;

			this->stream_queue_mutex.lock();
			/* stream queue should have space to load frame */
			if (this->stream_.size() < MAX_LOAD_FRAME_CNT) {
				load = true;
			}
			this->stream_queue_mutex.unlock();
			if (load) {
				/* Load frame */
				auto        frame_ptr = std::make_shared<common::Frame_t>();
				std::string name      = Manager::frame_name_prev + std::to_string(Manager::LOADED_FRAME_CNT) + ".frame";
				printf("Try to load frame #%d from %s.\n", Manager::LOADED_FRAME_CNT, name.c_str());
				if (io::LoadFrame(*frame_ptr, name)) {
					printf("Load frame %s failed, program exit.\n", name.c_str());
					exit(1);
				}
				printf("Load frame from %s successfully, slice count %d.\n", name.c_str(), frame_ptr->slice_cnt);
				this->stream_queue_mutex.lock();
				this->stream_.push(frame_ptr);
				printf("Push frame into stream buffer, buffer size is %lu now.\n", this->stream_.size());
				this->stream_queue_mutex.unlock();
				Manager::LOADED_FRAME_CNT++;
			}
		}
		printf("Thread to load frame exit successfully.\n");
	}

	void Manager::StartDecoder() {
		printf("Lauch thread to decode frame successfully.\n");
		Manager::DECODED_FRAME_CNT = 0;

		printf("Make OpenGL sharing context.\n");
		Renderer.MakeContextShareWindow();

		octree::InvertRAHTOctree* temp_Decoders = new octree::InvertRAHTOctree[Manager::PATCH_SIZE];
		printf("Malloc GPU memory ......\n");
		gpuErrchk(cudaMalloc((void**)&(Decoders), sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMemcpy(Decoders, temp_Decoders, sizeof(octree::InvertRAHTOctree) * Manager::PATCH_SIZE, cudaMemcpyHostToDevice));
		delete[] temp_Decoders;

		/* GPU 申请一维数组空间 */
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.inner_offset_gpu), sizeof(int) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.index_gpu), sizeof(int) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.type_gpu), sizeof(uint8_t) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.qp_gpu), sizeof(uint8_t) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.geometry_size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void**)&(CUDAFrame.color_size_gpu), sizeof(uint32_t) * Manager::PATCH_SIZE));

		/* GPU 申请二维数组空间 */
		gpuErrchk(cudaMalloc((void***)&(CUDAFrame.mv_gpu), sizeof(float*) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void***)&(CUDAFrame.geometry_gpu), sizeof(uint8_t*) * Manager::PATCH_SIZE));
		gpuErrchk(cudaMalloc((void***)&(CUDAFrame.color_gpu), sizeof(uint8_t*) * Manager::PATCH_SIZE));

		temp_mv    = (float**)malloc(sizeof(float*) * Manager::PATCH_SIZE);
		temp_geo   = (uint8_t**)malloc(sizeof(uint8_t*) * Manager::PATCH_SIZE);
		temp_color = (uint8_t**)malloc(sizeof(uint8_t*) * Manager::PATCH_SIZE);

		for (int i = 0; i < Manager::PATCH_SIZE; ++i) {
			gpuErrchk(cudaMalloc((void**)&(temp_mv[i]), sizeof(float) * 16));
			gpuErrchk(cudaMalloc((void**)&(temp_geo[i]), sizeof(uint8_t) * MAX_SLICE_SIZE));
			gpuErrchk(cudaMalloc((void**)&(temp_color[i]), sizeof(uint8_t) * MAX_SLICE_SIZE));
		}

		printf("Malloc GPU memory successfully.\n");
		while (Manager::DECODED_FRAME_CNT < TOTAL_FRAME_CNT) {
			std::shared_ptr<common::Frame_t> frame_ptr{};
			this->stream_queue_mutex.lock();
			if (!this->stream_.empty()) {
				frame_ptr = this->stream_.front();
				this->stream_.pop();
			}
			this->stream_queue_mutex.unlock();

			if (frame_ptr) {
				int frame_point_cnt{};
				for (int idx = 0; idx < frame_ptr->slice_cnt; ++idx) {
					frame_point_cnt += frame_ptr->size[idx];
				}
				printf("Successfuly get frame #%d from stream buffer, frame point count %d.\n", Manager::DECODED_FRAME_CNT, frame_point_cnt);
				printf("Transmit frame to GPU ......\n");
				/* Transmit frame to GPU memory */
				FrameCpu2Gpu(*frame_ptr);
				printf("Transmit frame to GPU successfully.\n");
				printf("Malloc VBO memory for decoding ......\n");
				VBOMemZone mem = this->AllocVBOMem(frame_point_cnt);
				printf("Malloc VBO memory for decoding, type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				printf("Decoding frame ......\n");
				Renderer.CUDADecode(mem.start, frame_ptr->timestamp, frame_ptr->slice_cnt);
				printf("Decoding frame successfully.\n");
				this->AddVBOMem(mem);
				printf("Add VBO memory into frames queue, type %d [%d , %d]\n", mem.type, mem.start, mem.start + mem.size);
				Manager::DECODED_FRAME_CNT++;
			}
		}
		printf("Thread to decode frame exit successfully.\n");
	}

	void Manager::ReleaseRender(VBOMemZone _mem) {
		if (_mem.size <= 0) {
			printf("Error, try to release %d VBO memory.\n", _mem.size);
			exit(1);
		}
		this->unused_memory_mutex.lock();
		if (_mem.type == -1) {
			printf("Garbage collection [%d , %d]\n", _mem.start, _mem.size + _mem.start);
		}
		else if (_mem.type == 1) {
			printf("Release render memory in VBO [%d , %d]\n", _mem.start, _mem.size + _mem.start);
		}
		else {
			printf("Unknown error, try to release VBO memory type %d [%d , %d].\n", _mem.type, _mem.start, _mem.start + _mem.size);
		}
		this->unused_size_ += _mem.size;
		this->unused_memory_mutex.unlock();
	}

	VBOMemZone Manager::GetVBOMem() {
		VBOMemZone mem{};
		while (true) {
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
				printf("OpenGL will render data in VBO [%d , %d]\n", mem.start, mem.start + mem.size);
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
