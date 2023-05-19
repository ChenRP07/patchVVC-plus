/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin, ChenRP07
 * Description   :
 * Create Time   : 2023/05/19 10:59
 * Last Modified : 2023/05/19 10:59
 *
 */

#ifndef _PVVC_CUDA_RENDER_H_
#define _PVVC_CUDA_RENDER_H_

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cuda_gl_interop.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <rendercheck_gl.h>

#include "helper_gl.h"
#include <helper_cuda.h>
#include <helper_functions.h>

#include "cuda/octree.cuh"

#include <iostream>
#include <string>
#include <unistd.h>

namespace vvc {
namespace client {
	namespace render {
		// clang-format off
		extern "C" {
		    void launch_cudaProcess(int grid, int block, 
                                common::Points* cudaData, 
                                int timestamp, 
                                int* inner_offset, 
                                int* index, 
                                uint8_t* type, 
                                float** mv, 
                                uint32_t* size, 
                                uint8_t* qp, 
                                uint8_t** geometry,
		                        uint32_t* geometry_size, 
                                uint8_t** color, 
                                uint32_t* color_size, 
                                vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu, int patch_size);
		}

		// clang-format on

		/* Shared memory of CUDA and OpenGL */
		extern cudaGraphicsResource_t cudaGraphicsResourcePtr;

		/* Window parameters */
		constexpr unsigned int SCR_WIDTH  = 800;
		constexpr unsigned int SCR_HEIGHT = 600;

		/* Camera parameters */
		extern glm::vec3 cameraPos;
		extern glm::vec3 camerasFront;
		extern glm::vec3 cameraUp;

		/* Time parameters */
		extern float deltaTime;
		extern float lastTime;

		/* Mouse and direction parameters */
		extern float lastX;
		extern float lastY;
		extern float fov;
		extern float yaw;
		extern float pitch;
		extern bool  firstMouse;

		/* Memory pointer and size */
		extern common::Points* cudaData;
		extern size_t          numBytes;

		/**
		 * @description: 回调函数_监听鼠标滚轮事件
		 * @param {GLFWwindow*} window  窗口
		 * @param {double} xoffset
		 * @param {double} yoffset  竖直滚动的大小
		 * @return {*}
		 */
		extern void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

		/**
		 * @description: 回调函数_监听鼠标移动事件
		 * @param {GLFWwindow*} window  窗口
		 * @param {double} xpos  当前鼠标横坐标位置
		 * @param {double} ypos  当前鼠标纵坐标位置
		 * @return {*}
		 */
		extern void mouse_callback(GLFWwindow* window, double xpos, double ypos);

		/**
		 * @description: 回调函数_调整视口
		 * @param {GLFWwindow*} window  窗口
		 * @param {int} width 宽度
		 * @param {int} height 高度
		 * @return {*}
		 */
		extern void framebuffer_size_callback(GLFWwindow* window, int width, int height);

		/**
		 * @description: 配置着色器函数
		 * @param {unsigned int} type
		 * @param {const std::string&} source
		 * @return {*}
		 */
		extern unsigned int CompileShader(unsigned int type, const std::string& source);

		/*配置顶点着色器，渲染顶点位置*/
		extern const char* vertexShaderSource;

		/*配置片段着色器，渲染顶点颜色*/
		extern const char* fragmentShaderSource;

		class Render {
		  private:
			GLFWwindow*  window;  // 窗口信息
			GLFWwindow*  shareWindow;
			unsigned int shaderProgram;  // 顶点着色器
			unsigned int VAO;            // 顶点数组对象
			unsigned int VBO;            // 顶点缓冲对象

			/* XXX: deprecated */
			// [[deprecated]]std::vector<int> bufferFrameSize;  // 每帧的大小

			float begin_time;
			float currentFrame;

		  public:
			Render();
			~Render();

			/* @description: 窗口初始化 */
			void InitWindow();

			/* @description: OpenGL初始化 */
			void InitOpenGL();

			/*  XXX: deprecated */
			/* @description: 将渲染的数据放置缓冲区 */
			// [[deprecated]] void InputData(std::vector<std::vector<vvc::client::common::Points>>& _vertices, int start, int end);

			/* @description: 渲染内容 */
			void Rendering(int offset, int frame_size);

			/* @description: 利用CUDA解码更新缓冲区 */
			void CUDADecode(int offset, int timestamp, int patch_size);

			inline void MakeContextWindow() {
				glfwMakeContextCurrent(window);
			}

			inline void MakeContextShareWindow() {
				glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
				shareWindow = glfwCreateWindow(1, 1, "shareWindow", NULL, window);
				glfwMakeContextCurrent(shareWindow);
			}
		};
	}  // namespace render
}  // namespace client
}  // namespace vvc
#endif
