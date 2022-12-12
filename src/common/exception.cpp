/* Copyright Notice.
 * 
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 * 
 * Copyright: @ChenRP07, All Right Reserved.
 * 
 * Author        : ChenRP07
 * Description   : 
 * Create Time   : 2022/12/06 19:18
 * Last Modified : 2022/12/06 19:18
 * 
 */ 

#include "common/exception.h"
#include <cstdio>

using namespace vvc;

common::Exception::Exception(unsigned int _err_line, std::string _file_name, std::string _func_name, std::string _err_message) {
    this->line_ = _err_line;
    this->file_ = _file_name;
    this->func_ = _func_name;
    this->message_ = _err_message;
}

void common::Exception::Log() const {
    std::cout << "\033[91m[Error] \033[0min file \033[96m" << this->file_ << "\033[0m, func \033[96m" << this->func_ << "\033[0m, line \033[96m" << this->line_ << "\033[0m, \033[91m" << this->message_ << "\033[0m." << std::endl;
}


