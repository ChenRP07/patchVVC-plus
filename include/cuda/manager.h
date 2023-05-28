/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/18 14:41
 * Last Modified : 2023/05/18 14:41
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

	struct VBOMemZone {
		int start;
		int size;
		int type; /* -1 if is little garbage memory, 1 if is frame memory, 0 if is unused */
		VBOMemZone() : start{}, size{}, type{} {}
		~VBOMemZone() = default;
	};

	extern float**        temp_mv[MAX_LOAD_FRAME_CNT];
	extern uint8_t**      temp_geo[MAX_LOAD_FRAME_CNT];
	extern uint8_t**      temp_color[MAX_LOAD_FRAME_CNT];
	extern render::Render Renderer;

	extern void FrameCpu2Gpu(vvc::client::common::Frame_t& _frame);

	extern void MallocGpuBuffer();
	/*
	 * How to use?
	 * auto& p = Manager::Init();
	 * p.Start(patch_size, frame_name_prev);
	 * */
	class Manager {
	  public:
		/* Parameters */
		const static int   MAX_VBO_SIZE{FRAME_POINT_CNT * MAX_VBO_FRAME_CNT};
		static int         RENDERED_FRAME_CNT;
		static int         DECODED_FRAME_CNT;
		static int         LOADED_FRAME_CNT;
		static std::string frame_name_prev;
		static int         PATCH_SIZE;
		/* VBO decoded frames zone */
	  private:
		std::queue<VBOMemZone> frames_;
		std::mutex             frames_queue_mutex_;

		void       AddVBOMem(VBOMemZone _mem);
		VBOMemZone AllocVBOMem(int _size);
		/* Render zone */
	  private:
		// [[deprecated]] std::thread render_;
		VBOMemZone  GetVBOMem();
		void        ReleaseRender(VBOMemZone _mem);
		/* render task function */
		// [[deprecated]] void StartRender();

	  private:
		int        unused_start_;
		int        unused_size_;
		std::mutex unused_memory_mutex;

	  private:
		std::thread decoder_;

		void StartDecoder();

	  private:
		// std::queue<std::shared_ptr<common::Frame_t>> stream_;
		// std::mutex                                   stream_queue_mutex;
		std::queue<int> unusedGpuBuffer;
		std::queue<int> filledGpuBuffer;
		std::mutex	unusedGpuBufferMutex;
		std::mutex	filledGpuBufferMutex;
		std::thread                                  frame_loader_;
		void                                         StartFrameLoader();

	  private:
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
