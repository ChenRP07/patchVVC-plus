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

common::exception::exception(unsigned int err_line, std::string file_name, std::string func_name, std::string err_message) {
    this->line = err_line;
    this->file = file_name;
    this->func = func_name;
    this->message = err_message;
}

void common::exception::log() const {
    std::cout << "\033[91m[Error] \033[0min file \033[96m" << this->file << "\033[0m, func \033[96m" << this->func << "\033[0m, line \033[96m" << this->line << "\033[0m, \033[91m" << this->message << "\033[0m." << std::endl;
}


