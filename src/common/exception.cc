/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07, All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   : Implementation of exception.
 * Create Time   : 2022/12/06 19:18
 * Last Modified : 2022/12/14 17:25
 *
 */

#include "common/exception.h"
#include <cstdio>

namespace vvc {
namespace common {
	Exception::Exception(unsigned int _err_line, std::string _file_name, std::string _func_name, std::string _err_message) {
		this->line_    = _err_line;
		this->file_    = _file_name;
		this->func_    = _func_name;
		this->message_ = _err_message;
	}

	void Exception::Log() const {
        printf("\033[91m[Error]\033[0m in file \033[94m%s\033[0m, func \033[95m%s\033[0m, line \033[96m%d\033[0m, \033[91m%s\033[0m.\n", this->file_.c_str(), this->func_.c_str(), this->line_, this->message_.c_str());
	}

}  // namespace common
}  // namespace vvc
