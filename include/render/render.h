/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07, Lixin
 * Description   :
 * Create Time   : 2023/05/09 10:15
 * Last Modified : 2023/05/09 10:15
 *
 */

#ifndef _PVVC_RENDER_H_
#define _PVVC_RENDER_H_

/* OpenGL */
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unistd.h>
#include <vector>

#include "common/common.h"
#include "common/parameter.h"

/* CUDA headers here */
extern "C" {}

namespace vvc {
namespace render {
	static const double WINDOW_FOV_MIN = 1.0;
	static double       WINDOW_FOV_MAX;
	static double       Window_FoV;

	static const float FRUSTUM_MIN = 0.1f, FRUSTUM_MAX = 40000.0f;

	static bool   FirstMouse{true};
	static double LastMouseX, LastMouseY;
	static float  DeltaTime{}, LastTime{};

	static double Sensitivity;

	static const double MAX_PITCH{89.9}, MIN_PITCH{-89.9};

	static double Yaw{-90.0}, Pitch{0.0};

	static glm::vec3 CameraFront{0.0f, 0.0f, -1.0f};
	static glm::vec3 CameraPos{512.0f, 512.0f, 2048.0f};
	static glm::vec3 CameraUp{0.0f, 1.0f, 0.0f};
	static bool      WINDOW_VALID{true};
	static bool      IO_THREAD_ALIVE{false};
	static bool      DECODE_THREAD_ALIVE{false};

	class GLRender {
	  private:
		GLFWwindow*              window_;          /* Window */
		uint32_t                 shader_;          /* OpenGL shader */
		uint32_t                 VAO_;             /* VAO */
		uint32_t                 VBO_;             /* VBO for reference cloud */
		std::string              vertex_shader_;   /* Source code of vertex shader */
		std::string              fragment_shader_; /* Source code of fragment shader */
		common::PVVCParam_t::Ptr params_;          /* Parameters */

		std::thread IO_thread_;     /* Threads to read source compressed file */
		std::thread decode_thread_; /* Threads to call CUDA interface to decode data */

		std::mutex VBO_render_buffer_mutex_; /* Mutex of each buffer */
		std::mutex VBO_unused_buffer_mutex_;
		std::mutex VBO_stage_buffer_mutex_;

		std::queue<uint32_t> VBO_unused_buffer_; /* Buffer queue to store unused vbo */
		std::queue<uint32_t> VBO_stage_buffer_;  /* Buffer queue to store vbos which has loaded compressed data but not been decoded yet */
		std::queue<uint32_t> VBO_render_buffer_; /* Buffer queue to store vbos which has already been decoded but not been rendered yet */

		int frame_read_count;
		int frame_decode_count;
		int frame_render_count;

		std::queue<std::shared_ptr<std::vector<common::Slice>>> source_buffer_;

	  private:
		/*
		 * @decription : Call back function to adjust window size
		 * @param  : {GLFWwindow* _window}
		 * @param  : {int _width}
		 * @param  : {int _height}
		 * @return : {}
		 * */
		static void ViewPortCallBack(GLFWwindow* _window, int _width, int _height);

		/*
		 * @decription : Call back function for mouse scroll
		 * @param  : {GLFWwindow* _window}
		 * @param  : {double _x_offset}
		 * @param  : {double _y_offset}
		 * @return : {}
		 * */
		static void ScrollCallBack(GLFWwindow* _window, double _x_offset, double _y_offset);

		/*
		 * @decription : Call back function for mouse movement
		 * @param  : {GLFWwindow* _window}
		 * @param  : {double _x_pos}
		 * @param  : {double _y_pos}
		 * @return : {}
		 * */
		static void MouseCallBack(GLFWwindow* _window, double _x_pos, double _y_pos);

		/*
		 * @description : Compile OpenGL shader
		 * @param  : {uint32_t _type} shader type, vertex or fragment
		 * @return : {uint32_t shader_id}
		 * */
		uint32_t CompileShader(uint32_t _type);

		/*
		 * @desciption : Read compressed files
		 * */
		void ReadFrames();

		void Decode();

	  public:
		/* Constructor and deconstructor */
		GLRender() = default;

		~GLRender();

		/*
		 * @decription : Initialize OpenGL
		 * */
		void Init();

		/*
		 * @decription : Set patchVVC parameters
		 * @param  : {common::PVVCParam_t::Ptr _param}
		 * @return : {}
		 * */
		void SetParams(common::PVVCParam_t::Ptr _param);

		/*
		 * @decription : Rendering
		 * */
		void Rendering();
	};
}  // namespace render
}  // namespace vvc

#endif
