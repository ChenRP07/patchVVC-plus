/*
 * @Author: lixin
 * @Date: 2023-05-05 16:01:48
 * @LastEditTime: 2023-05-05 17:36:41
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#ifndef _PVVC_CUDA_BASE_CUH_
#define _PVVC_CUDA_BASE_CUH_
namespace vvc {
namespace common {
        struct Points   //点云数据的结构体
        {
            float x, y, z;  //三维坐标值
            float r, g, b;  //颜色信息
        };
    }
}
#endif
