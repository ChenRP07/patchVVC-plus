/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : Lixin, ChenRP07
 * Description   :
 * Create Time   : 2023/05/19 11:36
 * Last Modified : 2023/05/19 11:36
 *
 */

#include "cuda/render.cuh"
namespace vvc {
namespace client {
	namespace render {
		cudaGraphicsResource_t cudaGraphicsResourcePtr{};

		glm::vec3 cameraPos   = glm::vec3(300.0f, 400.0f, 3000.0f);  // 相机初始位置
		glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);        // 方向向量
		glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);         // 上方向向量，用于叉乘出右向量

		float deltaTime = 0.0f;  // 当前帧与上一帧的时间差
		float lastTime  = 0.0f;  // 上一帧的时间

		bool  firstMouse = true;               // 判断用户是否第一次进行鼠标输入
		float lastX      = SCR_WIDTH / 2.0f;   // 鼠标位置初始X值
		float lastY      = SCR_HEIGHT / 2.0f;  // 鼠标位置初始Y值
		float fov        = 45.0f;              // 视野(Field of View)，定义可以看到场景中的多大范围
		float yaw        = -90.0f;             // 偏航角初始化为-90.0度，若为0.0会导致方向向量指向右侧，所以最初会向左旋转一点
		float pitch      = 0.0f;

		common::Points* cudaData{};
		size_t          numBytes{};
		common::Points* interMem{};

		void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
			fov -= (float)yoffset;
			if (fov < 1.0f)
				fov = 1.0f;
			if (fov > 45.0f)
				fov = 45.0f;
		}

		void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
			// 判断是否第一次鼠标输入，赋予鼠标初始值
			if (firstMouse) {
				lastX      = xpos;   // 当前鼠标位置X值
				lastY      = ypos;   // 当前鼠标位置Y值
				firstMouse = false;  // 置为false
			}

			// 计算当前帧和上一帧鼠标位置的偏移量
			float xoffset = xpos - lastX;  // 横向偏移量
			float yoffset = lastY - ypos;  // 纵向偏移量，注意这里是相反的，因为y坐标是从底部往顶部依次增大的
			lastX         = xpos;
			lastY         = ypos;

			float sensitivity = 0.001f;  // 灵敏度值
			xoffset *= sensitivity;      // 偏移量 * 灵敏度
			yoffset *= sensitivity;

			yaw += xoffset;  // 得到横向偏转和纵向偏转
			pitch += yoffset;

			// 添加转动限制
			if (pitch > 89.0f)
				pitch = 89.0f;
			if (pitch < -89.0f)
				pitch = -89.0f;

			// 计算俯仰角和偏航角，得到方向向量
			glm::vec3 front;
			front.x     = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
			front.y     = sin(glm::radians(pitch));
			front.z     = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
			cameraFront = glm::normalize(front);
		}

		void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
			glViewport(0, 0, width, height);
		}

		unsigned int CompileShader(unsigned int type, const std::string& source) {
			const char* src = source.c_str();  // string转C字符串指针
			// 定义、附加源码
			unsigned int id = glCreateShader(type);  // 着色器对象
			glShaderSource(id, 1, &src, NULL);       // 1、着色器对象；2、传递的源码字符串数量；3、源码的地址
			glCompileShader(id);                     // 编译源码
			// 验证基于当前管线状态的程序成功执行
			int success;
			glGetShaderiv(id, GL_COMPILE_STATUS, &success);  // 检查Shader相关错误

			if (success == GL_FALSE)  // GL_FALSE的定义即为0，方便维护查询用
			{
				// 定义
				int length;
				glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);        // 查询错误消息的长度
				char* message = (char*)alloca(length * sizeof(char));  // alloca为C语言函数，可在堆栈上动态分配
				// 打印着色器信息
				glGetShaderInfoLog(id, length, &length, message);
				std::cout << (type == GL_VERTEX_SHADER ? "顶点" : "片段") << "着色器编译失败！\n" << message << std::endl;
				glDeleteShader(id);  // 删除着色器

				return 0;
			}
			return id;
		}

		const char* vertexShaderSource =  // GLSL顶点着色器源码
		    "#version 330 core\n"  // 标题语言版本，core不会使用任何弃用的函数或任何会导致着色器编译的事件//告诉编译器我们的 Shader 程序是针对3.3版本的 GLSL
		    "layout (location = 0) in vec3 aPos1;\n"  // 声明了一个指定为顶点属性的float类型四维向量，告诉编译器顶点缓冲区中的顶点属性与 Shader
		                                              // 中声明的属性的映射关系——在应用程序中使用硬编码，显式地对其进行设置（设置为0）
		    "layout (location = 1) in vec3 aPos2;\n"
		    "uniform mat4 view;\n"        // 观察矩阵
		    "uniform mat4 projection;\n"  // 投影矩阵
		    "out vec3 fragPos;\n"
		    "void main()\n"
		    "{\n"
		    "   gl_Position = projection * view * vec4(aPos1.x, aPos1.y, aPos1.z, 1.0);\n"
		    "	fragPos = aPos2;\n"
		    "}\0";

		const char* fragmentShaderSource =  // 片段着色器源码
		    "#version 330 core\n"
		    "in vec3 fragPos;\n"
		    "out vec4 FragColor;\n"
		    "uniform vec3 myColor;\n"  // 颜色向量
		    "void main()\n"
		    "{\n"
		    "FragColor = vec4(fragPos.x, fragPos.y, fragPos.z, 1.0f);\n"  // 控制输出的颜色，四个分量分别表示R，G，B和A（alpha）
		    "}\n\0";

		Render::Render() : window{}, shaderProgram{}, VAO{}, VBO{}, begin_time{}, currentFrame{} {}

		Render::~Render() {
			// 解绑 VBO
			glBindBuffer(GL_ARRAY_BUFFER, 0);
			// 解绑 VAO
			glBindVertexArray(0);
			glfwTerminate();  // 释放资源
		}

		void Render::InitWindow() {
			// glfw初始化
			if (!glfwInit()) {
				std::cout << " GLFW 初始化失败" << std::endl;
				exit(0);
			}
			// 使用OPenGL 3.3版本
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);  // 设置OpenGL渲染上下文的主版本号
			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);  // 设置OpenGL渲染上下文的次版本号
			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
			// 创建窗口对象
			window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "patchVVC", NULL, NULL);
			if (!window) {
				std::cout << "创建窗口失败！\n" << std::endl;
				glfwTerminate();  // 释放资源
				exit(0);
			}

			// glfw窗口上下文设置为当前线程的上下文
			MakeContextWindow();
			// 向glfw注册回调函数
			glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);  // 调整视口
			glfwSetScrollCallback(window, scroll_callback);                     // 向GLFW注册了回调函数，当鼠标滚动时调用
			glfwSetCursorPosCallback(window, mouse_callback);                   // 向GLFW注册了回调函数，当鼠标移动时调用
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);        // 让GLFW隐藏光标并捕捉(Capture)它（/锁定鼠标）

			if (glewInit() != GLEW_OK)  // GLEW初始化
				std::cout << "GLEW 出错！" << std::endl;
		}

		void Render::InitOpenGL() {
			// 绑定 VAO 和 VBO
			glGenVertexArrays(1, &VAO);
			glBindVertexArray(VAO);  // 绑定VAO
			glGenBuffers(1, &VBO);
			glBindBuffer(GL_ARRAY_BUFFER, VBO);  // 绑定VBO为顶点缓冲类型

			unsigned int vertexShader   = CompileShader(GL_VERTEX_SHADER, vertexShaderSource);      // 顶点着色器对象
			unsigned int fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);  // 片段着色器对象

			/*配置着色器程序*/
			// 定义
			shaderProgram = glCreateProgram();
			// 附加、链接着色器
			glAttachShader(shaderProgram, vertexShader);  // 附加着色器
			glAttachShader(shaderProgram, fragmentShader);
			glLinkProgram(shaderProgram);  // 链接着色器
			// 检查
			glValidateProgram(shaderProgram);  // 检查程序中包含的可执行文件是否可以在当前的OpenGL状态下执行
			// 删除着色器
			glDeleteShader(vertexShader);
			glDeleteShader(fragmentShader);

			glBufferData(GL_ARRAY_BUFFER, MAX_VBO_FRAME_CNT * FRAME_POINT_CNT * sizeof(common::Points), nullptr,
			             GL_DYNAMIC_DRAW);  // 1、缓存区将用于存储顶点数组；2、数据的总字节数；3、顶点数组的地址；4、告诉编译器所绘制的数据几乎不变
			// 设置顶点属性指针，存放点的(x y z)，解析顶点数据用
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(common::Points),
			                      (void*)0);  // 1、顶点属性的位置值；2、顶点属性的大小；3、数据类型；4、是否要标准化；5、步长，指顶点属性每组之间的间隔；6、位置数据在缓冲中起始位置的偏移量
			glEnableVertexAttribArray(0);     // 启用顶点属性

			// 设置顶点属性指针，存放点的(r g b)，解析顶点数据用
			glVertexAttribPointer(
			    1, 3, GL_FLOAT, GL_TRUE, sizeof(common::Points),
			    (void*)(3 * sizeof(float)));  // 1、顶点属性的位置值；2、顶点属性的大小；3、数据类型；4、是否要标准化；5、步长，指顶点属性每组之间的间隔；6、位置数据在缓冲中起始位置的偏移量
			glEnableVertexAttribArray(1);     // 启用顶点属性

			// 注册OpenGL缓冲区对象到CUDA
			gpuErrchk(cudaGraphicsGLRegisterBuffer(&cudaGraphicsResourcePtr, VBO, cudaGraphicsMapFlagsNone));

			// 申请 interMem 的内存
			gpuErrchk(cudaMalloc((void**)&(interMem), sizeof(common::Points) * FRAME_POINT_CNT));
		}

		void Render::Rendering(int offset, int frame_size) {
			/*渲染设置*/
			glClearColor(1.0, 1.0, 1.0, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glfwSwapInterval(1);  // 设置前后缓冲区交换间隔为1，使得每帧更新一次

			// 时间差计算
			currentFrame = glfwGetTime();
			deltaTime    = currentFrame - lastTime;
			lastTime     = currentFrame;
			// 键盘输入
			float cameraSpeed = 200.0f * deltaTime;  // 摄像机移动速度
			// WASD
			if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
				cameraPos += cameraSpeed * cameraFront;
			if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
				cameraPos -= cameraSpeed * cameraFront;
			if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
				cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
			if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
				cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
			if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
				sleep(1);
			// Ese退出
			if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
				glfwSetWindowShouldClose(window, true);

			/*渲染命令*/
			/// 矩阵设置
			glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);  // 观察矩阵
			glUniformMatrix4fv(
			    glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE,
			    glm::value_ptr(view));  // 把矩阵数据发送给着色器。1、查询uniform变量的地址值；2、要发送多少个矩阵；3、是否置换矩阵（交换我们矩阵的行和列）；4、GLM的矩阵数据转为OpenGL所能接受的数据
			glm::mat4 projection = glm::perspective(
			    glm::radians(fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f,
			    40000.0f);  // 投影矩阵,创建可视空间的大平截头体（任何在平截头体以外的东西不会出现在裁剪空间体积内，并且将会受到裁剪）。1、视野，设定观察空间大小；2、宽高比；2、平截头的近平面；4、平截头的远平面
			glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
			glDepthMask(GL_TRUE);
			glEnable(GL_DEPTH_TEST);
			/// 绘制顶点
			glUseProgram(shaderProgram);  // 激活着色器程序对象，之后每个着色器调用和渲染调用都会使用这个程序对象
			glBindVertexArray(VAO);       // 绑定VAO
			glPointSize(1.0);             // 设置点的大小
			glUniform3f(glGetUniformLocation(shaderProgram, "myColor"), 0.1, 3.0, 0.8);
			glDrawArrays(GL_POINTS, offset,
			             frame_size);  // 顺序绘制/有序渲染。1、指定点的拓扑结构，GL_POINTS 意味着顶点缓冲区中的每一个顶点都被绘制成一个点；2、第一个顶点的索引值；3、用于绘制的顶点数量
			glfwSwapBuffers(window);   // 检测有没有触发事件
			glfwPollEvents();          // 交换颜色缓冲，避免图像闪烁
		}

		void Render::CUDADecode(int offset, int timestamp, int patch_size, int index) {
			int numElements = patch_size;
			int blockSize   = 1;
			int numBlocks   = (numElements + blockSize - 1) / blockSize;
			// clang-format off
            // 启动kernel
            launch_cudaProcess(numBlocks, blockSize, interMem, timestamp, 
                               CUDAFrame[index].inner_offset_gpu,
                               CUDAFrame[index].index_gpu,
                               CUDAFrame[index].type_gpu,
                               CUDAFrame[index].mv_gpu,
                               CUDAFrame[index].size_gpu,
                               CUDAFrame[index].qp_gpu,
                               CUDAFrame[index].geometry_gpu,
                               CUDAFrame[index].geometry_size_gpu,
                               CUDAFrame[index].color_gpu,
                               CUDAFrame[index].color_size_gpu, 
                               Decoders, 
                               patch_size);
			// clang-format on
			gpuErrchk(cudaDeviceSynchronize());

			// 将OpenGL缓冲区对象映射到CUDA
			gpuErrchk(cudaGraphicsMapResources(1, &cudaGraphicsResourcePtr, 0));
			gpuErrchk(cudaGraphicsResourceGetMappedPointer((void**)&(cudaData), &numBytes, cudaGraphicsResourcePtr));
			timeval t0, t1;
			gettimeofday(&t0, nullptr);
			gpuErrchk(cudaMemcpy(cudaData + offset, interMem, sizeof(common::Points) * CUDAFrame[index].point_number, cudaMemcpyDeviceToDevice));
			gpuErrchk(cudaDeviceSynchronize());
			gettimeofday(&t1, nullptr);
			printf("Memcpy_time = %.2f\n", (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f);
			// 将结果从CUDA复制回OpenGL缓冲区
			gpuErrchk(cudaGraphicsUnmapResources(1, &cudaGraphicsResourcePtr, 0));
		}
	}  // namespace render
}  // namespace client
}  // namespace vvc
