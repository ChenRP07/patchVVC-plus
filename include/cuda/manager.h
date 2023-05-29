/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07, Lixin
 * Description   :
 * Create Time   : 2023/05/18 14:41
 * Last Modified : 2023/05/29 15:58
 *
 */

#ifndef _PVVC_CUDA_MANAGER_H_
#define _PVVC_CUDA_MANAGER_H_

#include "cuda/render.cuh"
#include <mutex>
#include <queue>
#include <thread>

namespace vvc {
namespace client {

	/* VBO memory zone */
	struct VBOMemZone {
		/* Start offset */
		int start;
		/* Size */
		int size;
		int type; /* -1 if is little garbage memory, 1 if is frame memory, 0 if is unused */
		VBOMemZone() : start{}, size{}, type{} {}
		~VBOMemZone() = default;
	};

	/* Used in memcpy of XXX** */
	extern float** temp_mv[MAX_LOAD_FRAME_CNT];
	extern uint8_t** temp_geo[MAX_LOAD_FRAME_CNT];
	extern uint8_t** temp_color[MAX_LOAD_FRAME_CNT];

	/* Render */
	extern render::Render Renderer;

	/*
	 * @description : cudaMemcpy _frame to CUDAFrame[_index]
	 * @param  : {common::Frame_t& _frame}
	 * @param  : {int _index}
	 * @return : {}
	 * */
	extern void FrameCpu2Gpu(common::Frame_t& _frame, int _index);

	/* Malloc GPU memory for slice group buffer */
	extern void MallocGpuBuffer();

	/*
	 * Manager of whole client.
	 * How to use?
	 * auto& p = Manager::Init();
	 * p.Start(patch_size, frame_name_prev);
	 * */
	class Manager {
	  public:
		/* Parameters */
		/* VBO size */
		const static int MAX_VBO_SIZE{FRAME_POINT_CNT * MAX_VBO_FRAME_CNT};
		/* Rendered frame count */
		static int RENDERED_FRAME_CNT;
		/* Decoded frame count */
		static int DECODED_FRAME_CNT;
		/* Loaded frame count */
		static int LOADED_FRAME_CNT;
		/* Source file prev */
		static std::string frame_name_prev;
		/* Patch size */
		static int PATCH_SIZE;

	  private:
		/* Which VBOMemZone can be rendered */
		std::queue<VBOMemZone> frames_;
		std::mutex frames_queue_mutex_;

		/* Push decoded VBO zone to queu */
		void AddVBOMem(VBOMemZone _mem);
		/* Allocate VBO zone used to decoding */
		VBOMemZone AllocVBOMem(int _size);

	  private:
		/* Get a VBO mem zone to decode */
		VBOMemZone GetVBOMem();
		/* Release rendered zone */
		void ReleaseRender(VBOMemZone _mem);

	  private:
		/* Unused VBO start */
		int unused_start_;
		/* Unused VBO size */
		int unused_size_;
		std::mutex unused_memory_mutex;

	  private:
		/* Decoding thread */
		std::thread decoder_;

		/* Decoding thread function */
		void StartDecoder();

	  private:
		/* Unused Slice Group buffer index */
		std::queue<int> unusedGpuBuffer;
		/* Slice Group buffer index with data */
		std::queue<int> filledGpuBuffer;
		std::mutex unusedGpuBufferMutex;
		std::mutex filledGpuBufferMutex;
		/* Loader  thread and function */
		std::thread frame_loader_;
		void StartFrameLoader();

	  private:
        /* Constructor and deconstructor */
		Manager() : unused_size_{FRAME_POINT_CNT * MAX_VBO_FRAME_CNT}, frames_{}, unused_start_{} {}
		~Manager() {}

	  public:
		/*
		 * @description : Get single instance of Manager.
		 * @param  : {}
		 * @return : {Manager&}
		 * */
		static Manager& Init() {
			static Manager instance{};
			return instance;
		}

		/* Start task */
		void Start(int _patch_size, const std::string& _name_prev);
	};
}  // namespace client
}  // namespace vvc
#endif
