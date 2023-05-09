/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/08 19:09
 * Last Modified : 2023/05/08 19:09
 *
 */

#ifndef _PVVC_CUDA_EXCEPTION_H_
#define _PVVC_CUDA_EXCEPTION_H_

#include <stdio.h>
#include <string.h>

namespace vvc {

#define __CUDA_EXCEPT__(msg) vvc::common::CudaException(__LINE__, __FILE__, __FUNCTION__, vvc::common::CudaErrorCode::msg)
namespace common {

	enum CudaErrorCode {
		ERROR_OCCURED,        /* an error occured */
		EMPTY_POINT_CLOUD,    /* error occured when read a ply file but the point number is zero */
		EMPTY_RESULT,         /* error occured when get result before processing function */
		EMPTY_PARAMS,         /* error occured when set vvc_param_t but with no instance */
		INITIALIZER_ERROR,    /* error occured when a object member is used before initialization */
		BAD_PARAMETERS,       /* error occured when input an illegal parameter */
		UNMATCHED_CLOUD_SIZE, /* error occured when try to concate two point cloud with different size */
		BAD_TIME_STAMP,       /* error occured when set or get a illegal timestamp */
		OUT_OF_RANGE,         /* error occured when try to access an out-of-bounds address location */
		BAD_POSITION,         /* error occured when set or get a subspace position not in {0,1,2,3,4,5,6,7} */
		EMPTY_OCTREE,         /* error occured when do RAHT before make an octree */
		UNMATCHED_COLOR_SIZE, /* error occured when size of ColorYUV is not equal to size of point cloud */
		EMPTY_GOP,            /* error occured when try to encode a GoP whose size is zero */
		BAD_SLICE,            /* error occured when the type of one slice is invalid */
		EMPTY_REFERENCE,      /* error occured when try to decode a predictive slice without intra slice */
		BAD_GLFW_INIT,        /* error occured when try to initialize glfw by glfwInit() but failed */
		WINDOW_CREATE_ERROR,  /* error occured when try to create a window by glfwCreateWindow() but failed */
		BAD_GLEW_INIT,        /* error occured when try to initialize glew by glewInit() but failed */
		SHADER_COMPILE_ERROR, /* error occured when try to compile shader but failed */
	};

	static const char* CudaErrorMessage[] = {
	    "an error occured",
	    "there is no point in cloud",
	    "no processing result, maybe the handler should be called first",
	    "empty vvc cuda parameters, might be a null pointer to PVVCCudaParam_t",
	    "object initialization error, should be initialized first",
	    "bad parameters",
	    "unmatched point cloud size",
	    "set/get an illegal timestamp",
	    "access out of range",
	    "subspace position must be an integer from 0 to 7",
	    "should make Octree before RAHT",
	    "color size is not equal to the point cloud",
	    "cannot encode a GoP with size zero",
	    "invalid slice type",
	    "no reference for predictive slice",
	    "cannot initialize glfw",
	    "cannot create window by glfw",
	    "cannot initialize glew",
	    "cannot compile shader",
	};

	class CudaException {
	  private:
		unsigned int line_;
		char*        file_;
		char*        func_;
		char*        message_;

	  public:
		CudaException(unsigned int _line, const char* _file, const char* _func, CudaErrorCode _message) : line_{_line} {
			strcpy(this->file_, _file);
			strcpy(this->func_, _func);
			strcpy(this->message_, CudaErrorMessage[_message]);
		}

		void Log() const {
			printf("\033[91m[Error]\033[0m in file \033[94m%s\033[0m, func \033[95m%s\033[0m, line \033[96m%d\033[0m, \033[91m%s\033[0m.\n", this->file_, this->func_, this->line_, this->message_);
		}
	};
}  // namespace common
}  // namespace vvc
#endif
