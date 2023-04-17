/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Definition of exception which used to handle the error of VVC.
 * Create Time   : 2022/12/06 19:06
 * Last Modified : 2022/12/27 14:06
 *
 */

#ifndef _PVVC_EXCEPTION_H_
#define _PVVC_EXCEPTION_H_

#include <iostream>
#include <string>

#define __EXCEPT__(msg) vvc::common::Exception(__LINE__, __FILE__, __FUNCTION__, vvc::common::ErrorMessage[vvc::common::msg])

namespace vvc {
namespace common {
	class Exception {
	  private:
		unsigned int line_;    /* line index where error occured  */
		std::string  file_;    /* file name where error occured */
		std::string  func_;    /* function name where error occured */
		std::string  message_; /* error message */

	  public:
		/*
		 * @description : constructor
		 * @param {unsigned int err_line} line index where error occured
		 * @param {std::string file_name} file name where error occured
		 * @param {std::string func_name} function name where error occured
		 * @param {std::string err_message} error message
		 * */
		Exception(unsigned int _err_line, std::string _file_name, std::string _func_name, std::string _err_message);

		/*
		 * @description : output log of error for debug
		 * */
		void Log() const;
	};

	enum ErrorType {
		ERROR_OCCURED,         /* an error occured */
		WRONG_FILE_FORMAT,     /* error occured when read/write a point cloud from/to a non-PLY file or this PLY file do not follow the ply standard */
		FILE_NOT_EXIST,        /* error occured when open a non-existed file to read data */
		PERMISSION_DENIED,     /* error occured when access a file but do not have permission */
		UNEXPECTED_FILE_ERROR, /* error occured when read/write a file, but is no clearly reasonable */
		FILE_READ_ERROR,       /* error occured when read a file, might be an unexpected EOF */
		FILE_WRITE_ERROR,      /* error occured when write a file with unclear reason */
		EMPTY_POINT_CLOUD,     /* error occured when read a ply file but the point number is zero */
		EMPTY_RESULT,          /* error occured when get result before processing function */
		EMPTY_PARAMS,          /* error occured when set vvc_param_t but with no instance */
		INVALID_PARAM_SEGMENT, /* error occured when then params of segmentation are invalid */
		INITIALIZER_ERROR,     /* error occured when a object member is used before initialization */
		BAD_PARAMETERS,        /* error occured when input an illegal parameter */
		UNMATCHED_CLOUD_SIZE,  /* error occured when try to concate two point cloud with different size */
		BAD_TIME_STAMP,        /* error occured when set or get a illegal timestamp */
		OUT_OF_RANGE,          /* error occured when try to access an out-of-bounds address location */
		BAD_POSITION,          /* error occured when set or get a subspace position not in {0,1,2,3,4,5,6,7} */
		EMPTY_OCTREE,          /* error occured when do RAHT before make an octree */
        UNMATCHED_COLOR_SIZE,  /* error occured when size of ColorYUV is not equal to size of point cloud */
	};

	inline static std::string ErrorMessage[100] = {
	    "an error occured",
	    "wrong file format",
	    "no such file",
	    "permission denied",
	    "file error occured with unexpected reason",
	    "error occured while reading file, might be an unexpected EOF",
	    "error occured while writing file, might be something wrong",
	    "there is no point in cloud",
	    "no processing result, maybe the handler should be called first",
	    "empty vvc parameters, might be a null pointer to vvc_param_t",
	    "parameters are invalid of segmentation",
	    "object initialization error, should be initialized first",
	    "bad parameters",
	    "unmatched point cloud size",
	    "set/get an illegal timestamp",
	    "access out of range",
	    "subspace position must be an integer from 0 to 7",
	    "should make Octree before RAHT",
        "color size is not equal to the point cloud",
	};

}  // namespace common
}  // namespace vvc
#endif
