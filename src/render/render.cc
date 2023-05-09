/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07, Lixin
 * Description   :
 * Create Time   : 2023/05/09 10:41
 * Last Modified : 2023/05/09 10:41
 *
 */

#include "render/render.h"

namespace vvc {
namespace render {

	GLRender::~GLRender() {
		/* Unbind VBO, VAO and release resources */

		/* Set a signal which means there is no window here, need to exit */
		WINDOW_VALID = false;

		/* Wait for all detached thread exit */
		while (IO_THREAD_ALIVE || DECODE_THREAD_ALIVE) {}

        /* XXX:Should release all vbo? */
		/* TODO: Unbind All vbo here */
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		glfwTerminate();
	}

	void GLRender::SetParams(common::PVVCParam_t::Ptr _param) {
		try {
			if (!_param) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			this->params_ = _param;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GLRender::ViewPortCallBack(GLFWwindow* _window, int _width, int _height) {
		/* Update view port */
		glViewport(0, 0, _width, _height);
	}

	void GLRender::ScrollCallBack(GLFWwindow* _window, double _x_offset, double _y_offset) {
		/* Update FoV */
		Window_FoV -= _x_offset;
		Window_FoV = Window_FoV < WINDOW_FOV_MIN ? WINDOW_FOV_MIN : Window_FoV;
		Window_FoV = Window_FoV > WINDOW_FOV_MAX ? WINDOW_FOV_MAX : Window_FoV;
	}

	void GLRender::MouseCallBack(GLFWwindow* _window, double _x_pos, double _y_pos) {
		/* First input mouse pos, init */
		if (FirstMouse) {
			LastMouseX = _x_pos;
			LastMouseY = _y_pos;
			FirstMouse = false;
		}

		/* Compute mouse movement */
		double x_offset{_x_pos - LastMouseX};
		double y_offset{LastMouseY - _y_pos};
		LastMouseX = _x_pos, LastMouseY = _y_pos;

		/* Sensitive */
		x_offset *= Sensitivity;
		y_offset *= Sensitivity;

		/* Real offset */
		Yaw += x_offset;
		Pitch += y_offset;

		Pitch = Pitch > MAX_PITCH ? MAX_PITCH : Pitch;
		Pitch = Pitch < MIN_PITCH ? MIN_PITCH : Pitch;

		/* Compute direction vector */
		glm::vec3 front;
		front.x     = std::cos(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
		front.y     = std::sin(glm::radians(Pitch));
		front.z     = std::sin(glm::radians(Yaw)) * std::cos(glm::radians(Pitch));
		CameraFront = glm::normalize(front);
	}

	uint32_t GLRender::CompileShader(uint32_t _type) {
		try {
			/* Set source according to _type */
			std::string* source;
			if (_type == GL_VERTEX_SHADER) {
				source = &this->vertex_shader_;
			}
			else if (_type == GL_FRAGMENT_SHADER) {
				source = &this->fragment_shader_;
			}
			else {
				throw __EXCEPT__(SHADER_COMPILE_ERROR);
			}
			/* Shader id */
			uint32_t id = glCreateShader(_type);

			/* Compile shader */
			auto c_source = source->c_str();
			glShaderSource(id, 1, &c_source, nullptr);
			glCompileShader(id);

			/* Print failed message */
			int success;
			glGetShaderiv(id, GL_COMPILE_STATUS, &success);
			if (success == GL_FALSE) {
				int len;
				glGetShaderiv(id, GL_INFO_LOG_LENGTH, &len);
				std::string error_message(len, ' ');
				glGetShaderInfoLog(id, len, &len, error_message.data());
				std::string error_type = _type == GL_VERTEX_SHADER ? "vertex" : "fragment";
				printf("\033[91m[Error]\033[0 Failed to compile \033[94m%s shader\033[0m, \033[91m%s\033[0m.\n", error_type.c_str(), error_message.c_str());
				glDeleteShader(id);
				throw __EXCEPT__(SHADER_COMPILE_ERROR);
			}
			return id;
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GLRender::Init() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}

			if (this->params_->render.buffer_size < 1) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			if (this->params_->render.vbo_buffer_size < 1) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}

			if (this->params_->render.FoV <= WINDOW_FOV_MIN) {
				throw __EXCEPT__(BAD_PARAMETERS);
			}
			Window_FoV = this->params_->render.FoV;

			/* Init glfw */
			if (glfwInit() != GLFW_TRUE) {
				throw __EXCEPT__(BAD_GLFW_INIT);
			}

			/* Use OpenGL version, default 3.3 */
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, this->params_->render.gl_major_version);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, this->params_->render.gl_minor_version);

			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

			/* Create window */
			this->window_ =
			    glfwCreateWindow(static_cast<int>(this->params_->render.scr_width), static_cast<int>(this->params_->render.scr_height), this->params_->render.scr_name.c_str(), nullptr, nullptr);

			if (!(this->window_)) {
				throw __EXCEPT__(WINDOW_CREATE_ERROR);
			}

			/* Register callback function */
			glfwMakeContextCurrent(this->window_);
			glfwSetFramebufferSizeCallback(this->window_, GLRender::ViewPortCallBack);
			glfwSetScrollCallback(this->window_, GLRender::ScrollCallBack);
			glfwSetCursorPosCallback(this->window_, GLRender::MouseCallBack);
			glfwSetInputMode(this->window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

			if (glewInit() != GLEW_OK) {
				throw __EXCEPT__(BAD_GLEW_INIT);
			}

			/* TODO:Init unused_vbo_buffer here */
			/* Malloc memory for vbos */

			/* TODO:Load file here */
			this->IO_thread_ = std::thread(&GLRender::ReadFrames, this);
			this->IO_thread_.detach();

			/* TODO:Decode data here */
			this->decode_thread_ = std::thread(&GLRender::Decode, this);
			this->decode_thread_.detach();

			/* Load shader from file */
			std::ifstream input_vertex_shader(this->params_->render.vertex_source);
			if (input_vertex_shader.fail()) {
				switch (errno) {
					case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
					case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
					default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
				}
			}
			std::ostringstream vertex_shader_stream;
			vertex_shader_stream << input_vertex_shader.rdbuf();
			this->vertex_shader_ = vertex_shader_stream.str();

			std::ifstream input_fragment_shader(this->params_->render.fragment_source);
			if (input_fragment_shader.fail()) {
				switch (errno) {
					case ENOENT: throw __EXCEPT__(FILE_NOT_EXIST); break;
					case EACCES: throw __EXCEPT__(PERMISSION_DENIED); break;
					default: throw __EXCEPT__(UNEXPECTED_FILE_ERROR); break;
				}
			}
			std::ostringstream fragment_shader_stream;
			fragment_shader_stream << input_fragment_shader.rdbuf();
			this->fragment_shader_ = fragment_shader_stream.str();

			/* Compile shader */
			uint32_t vertex_shader   = this->CompileShader(GL_VERTEX_SHADER);
			uint32_t fragment_shader = this->CompileShader(GL_FRAGMENT_SHADER);

			/* Config shader program */
			/* Definition */
			this->shader_ = glCreateProgram();
			/* Attach and link */
			glAttachShader(this->shader_, vertex_shader);
			glAttachShader(this->shader_, fragment_shader);
			glLinkProgram(this->shader_);
			/* Check */
			glValidateProgram(this->shader_);
			/* Delete */
			glDeleteShader(vertex_shader);
			glDeleteShader(fragment_shader);
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GLRender::Rendering() {
		try {
			if (!this->params_) {
				throw __EXCEPT__(EMPTY_PARAMS);
			}
			if (!this->window_) {
				throw __EXCEPT__(INITIALIZER_ERROR);
			}

			while (!glfwWindowShouldClose(this->window_)) {
				/* TODO:Load decoded data, data is indexed by render_vbo_id */
				uint32_t render_vbo_id{};
				bool     loop = true;

				this->VBO_render_buffer_mutex_.lock();
				if (!this->VBO_stage_buffer_.empty()) {
					render_vbo_id = this->VBO_render_buffer_.front();
					this->VBO_render_buffer_.pop();
					loop = false;
				}
				this->VBO_render_buffer_mutex_.unlock();
				if (loop) {
					continue;
				}

				int frame_size = 0;
				/* Suppose we now have decoded point cloud in render_vbo_id */
				/* Initialize OpenGL */
				glClearColor(0, 0, 0, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glfwSwapInterval(1);

				/* Set movement by WASD and exit by ESC */
				float CurrentTime = glfwGetTime();
				DeltaTime         = CurrentTime - LastTime;
				LastTime          = CurrentTime;
				float CameraSpeed = this->params_->render.camera_move_ratio * DeltaTime;
				if (glfwGetKey(this->window_, GLFW_KEY_W) == GLFW_PRESS) {
					CameraPos += CameraSpeed * CameraFront;
				}
				if (glfwGetKey(this->window_, GLFW_KEY_S) == GLFW_PRESS) {
					CameraPos -= CameraSpeed * CameraFront;
				}
				if (glfwGetKey(this->window_, GLFW_KEY_A) == GLFW_PRESS) {
					CameraPos -= glm::normalize(glm::cross(CameraFront, CameraUp)) * CameraSpeed;
				}
				if (glfwGetKey(this->window_, GLFW_KEY_D) == GLFW_PRESS) {
					CameraPos += glm::normalize(glm::cross(CameraFront, CameraUp)) * CameraSpeed;
				}
				if (glfwGetKey(this->window_, GLFW_KEY_P) == GLFW_PRESS) {
					sleep(3);
				}
				if (glfwGetKey(this->window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
					glfwSetWindowShouldClose(this->window_, true);
				}

				/* Compute new camera position and direction */
				glm::mat4 view = glm::lookAt(CameraPos, CameraPos + CameraFront, CameraUp);
				glUniformMatrix4fv(glGetUniformLocation(this->shader_, "view"), 1, GL_FALSE, glm::value_ptr(view));
				glm::mat4 projection =
				    glm::perspective(glm::radians(static_cast<float>(Window_FoV)), static_cast<float>(this->params_->render.scr_width / this->params_->render.scr_height), FRUSTUM_MIN, FRUSTUM_MAX);

				glUniformMatrix4fv(glGetUniformLocation(this->shader_, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

				/* Set OpenGL depth test and config shader */
				glDepthMask(GL_TRUE);
				glEnable(GL_DEPTH_TEST);
				glUseProgram(this->shader_);

				/* TODO: Bind VBO with VAO */
				glBindVertexArray(this->VAO_);

				glPointSize(1.0f);
				/* What is it? */
				/* Draw point cloud */
				glUniform3f(glGetUniformLocation(this->shader_, "myColor"), 0.1, 3.0, 0.8);
				glDrawArrays(GL_POINTS, 0, frame_size);
				glfwSwapBuffers(this->window_);
				glfwPollEvents();

				uint32_t unused_vbo_id{};
				/* TODO: Push used VBO to VBO_unused_buffer */
				this->VBO_unused_buffer_mutex_.lock();
				this->VBO_unused_buffer_.push(unused_vbo_id);
				this->VBO_unused_buffer_mutex_.unlock();
			}
		}
		catch (const common::Exception& e) {
			e.Log();
			throw __EXCEPT__(ERROR_OCCURED);
		}
	}

	void GLRender::ReadFrames() {
		this->frame_read_count = 0;
		IO_THREAD_ALIVE        = true;
		while (true) {
			if (this->frame_read_count >= this->params_->frames) {
				break;
			}
			if (!WINDOW_VALID) {
				break;
			}
			uint32_t unused_vbo_id{};
			bool     loop{true};

			this->VBO_unused_buffer_mutex_.lock();
			if (!this->VBO_unused_buffer_.empty()) {
				unused_vbo_id = this->VBO_unused_buffer_.front();
				this->VBO_unused_buffer_.pop();
				loop = false;
			}
			this->VBO_unused_buffer_mutex_.unlock();

			if (loop) {
				continue;
			}

			uint32_t stage_vbo_id{};
			/* TODO: Load frames from disk and send it to VBO when there is vbo in unused_buffer */
			/* stage_vbo_id index this vbo and push it into stage_buffer */

			this->VBO_stage_buffer_mutex_.lock();
			this->VBO_stage_buffer_.push(stage_vbo_id);
			this->VBO_stage_buffer_mutex_.unlock();
		}
		IO_THREAD_ALIVE = false;
	}

	void GLRender::Decode() {
		this->frame_decode_count = 0;
		DECODE_THREAD_ALIVE      = true;
		while (true) {
			if (this->frame_decode_count >= this->params_->frames) {
				break;
			}
			uint32_t stage_vbo_id{};
			bool     loop{true};

			this->VBO_stage_buffer_mutex_.lock();
			if (!this->VBO_stage_buffer_.empty()) {
				stage_vbo_id = this->VBO_stage_buffer_.front();
				this->VBO_stage_buffer_.pop();
				loop = false;
			}
			this->VBO_stage_buffer_mutex_.unlock();

			if (loop) {
				continue;
			}

			uint32_t render_vbo_id{};
			/* TODO: Call cuda interface to decode data in stage_vbo_id vbo */
			/* decoded result in render_vbo_id, same with stage_vbo_id in default */

			this->VBO_render_buffer_mutex_.lock();
			this->VBO_render_buffer_.push(render_vbo_id);
			this->VBO_render_buffer_mutex_.unlock();
		}
		DECODE_THREAD_ALIVE = false;
	}
}  // namespace render
}  // namespace vvc

