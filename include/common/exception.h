/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2022/12/06 19:06
 * Last Modified : 2022/12/08 19:26
 *
 */

#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include <iostream>
#include <string>

#define __EXCEPT__(msg) exception(__LINE__, __FILE__, __FUNCTION__, vvc::common::error_message[vvc::msg])

namespace vvc {
namespace common {
	class exception {
	  private:
		unsigned int line;
		std::string  file;
		std::string  func;
		std::string  message;

	  public:
		/*
         * @description : constructor
         * @param {unsigned int err_line} line index where error occured
         * @param {std::string file_name} file name where error occured 
         * @param {std::string func_name} function name where error occured 
         * @param {std::string err_message} error message
         * */
        exception(unsigned int err_line, std::string file_name, std::string func_name, std::string err_message);
        
        /*
         * @description : output log of error for debug
         * */
        void log() const;
	};

    enum error_type {
        ERROR_OCCURED,      /* an error occured */
        WRONG_FILE_FORMAT,  /* error occured when read/write a point cloud from/to a non-PLY file or this PLY file do not follow the ply standard */
        FILE_NOT_EXIST,     /* error occured when open a non-existed file to read data */
        PERMISSION_DENIED,  /* error occured when access a file but do not have permission */
        UNEXPECTED_FILE_ERROR, /* error occured when read/write a file, but is no clearly reasonable */
        FILE_READ_ERROR,    /* error occured when read a file, might be an unexpected EOF */
        FILE_WRITE_ERROR,   /* error occured when write a file with unclear reason */
        EMPTY_POINT_CLOUD,  /* error occured when read a ply file but the point number is zero */
        EMPTY_RESULT,       /* error occured when get result before processing function */
    };

    inline static std::string error_message[100] = {
        "an error occured",
        "wrong file format",
        "no such file",
        "permission denied",
        "file error occured with unexpected reason",
        "error occured while reading file, might be an unexpected EOF",
        "error occured while writing file, might be something wrong",
        "there is no point in cloud",
        "no processing result, maybe the handler should be called first"
    };

}  // namespace common
}  // namespace vvc
#endif
