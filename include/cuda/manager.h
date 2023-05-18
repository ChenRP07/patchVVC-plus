/* Copyright Notice.
 *
 * Please read the LICENSE file in the project root directory for details
 * of the open source licenses referenced by this source code.
 *
 * Copyright: @SDUCS_IIC. All Right Reserved.
 *
 * Author        : ChenRP07
 * Description   :
 * Create Time   : 2023/05/18 14:41
 * Last Modified : 2023/05/18 14:41
 *
 */

#ifndef _PVVC_CUDA_MANAGER_H_
#define _PVVC_CUDA_MANAGER_H_

#include <mutex>
#include <queue>
#include <thread>

namespace vvc {
namespace client {
	constexpr int FRMAME_POINT_CNT{1'000'000};
	class Manager {
	  private:
		const static int MAX_VBO_SIZE;

	  private:
		Manager() {}
		~Manager() {}

	  public:
		/*
		 * @description : Get single instance of Manager.
		 * @param  : {}
		 * @return : {Manager&}
		 * */
		static Manager& Init() {
			static Manager instance{};
			return instance;
		}

		/* Start task */
		void Start();
	};
}  // namespace client
}  // namespace vvc
#endif
