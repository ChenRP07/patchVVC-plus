/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @ChenRP07. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/01 13:54
 * Last Modified : 2023/05/01 13:54
 *
 */

#ifndef _PVVC_PATCH_IO_H_
#define _PVVC_PATCH_IO_H_

#include "common/common.h"

#include <cstdio>
#include <regex>

namespace vvc {
namespace io {
	/*
	 * @description : Save patch to binary file. timestamp index mv size points.
	 * @param  : {const common::Patch& _patch}
	 * @param  : {const std::string& _name}
	 * @return : {}
	 * */
	extern void SavePatch(const common::Patch& _patch, const std::string& _name);

	/*
	 * @description : Load patch from binary file. timestamp index mv size points.
	 * @param  : {common::Patch& _patch}
	 * @param  : {const std::string& _name}
	 * @return : {}
	 * */
	extern void LoadPatch(common::Patch& _patch, const std::string& _name);
}  // namespace io
}  // namespace vvc

#endif
