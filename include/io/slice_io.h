/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/04/27 10:49
 * Last Modified : 2023/04/27 10:49
 *
 */

#ifndef _PVVC_SLICE_IO_H_
#define _PVVC_SLICE_IO_H_

#include "common/common.h"

#include <cstdio>
#include <regex>
#include <filesystem>

namespace vvc {
namespace io {
	/*
	 * @description : Save patchVVC slice.
	 * @param  : {const common::Slice& _slice}
	 * @param  : {const std::string& _name}
	 * @return : {}
	 * */
	[[nodiscard]] extern int SaveSlice(const common::Slice& _slice, const std::string& _name);

	/*
	 * @description : Save patchVVC slice.
	 * @param  : {const common::Slice& _slice}
	 * @param  : {const std::string& _name}
	 * @return : {}
	 * */
	extern void LoadSlice(common::Slice& _slice, const std::string& _name);

    /*
     * @description : Load slices from _input_dir and save a stream file in _ouput_name
     * @param  : {const std::string& _input_dir}
     * @param  : {const std::string& _output_name}
     * @return : {}
     * */
	extern void ChangeSliceToFrame(const std::filesystem::path& _input_dir, const std::string& _output_name);
}  // namespace io
}  // namespace vvc
#endif
