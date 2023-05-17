/*
 * @Author: lixin
 * @Date: 2023-05-16 11:47:17
 * @LastEditTime: 2023-05-16 22:04:30
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "device_launch_parameters.h"
#include <helper_cuda.h>
#include "cuda/octree.cuh"

struct Points
{
	float x, y, z;
	float r, g, b;
};

// 起始地址 + Slice的属性 + I帧的数组
__global__ void processCUDA(Points* cudaData, int timestamp, int index, uint8_t type, float* mv, uint32_t size, uint8_t qp, uint8_t* geometry, uint32_t geometry_size, uint8_t* color, uint32_t color_size, vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx == 0){
		vvc::client::common::Slice_t slice(timestamp, index, type, mv, size, qp, geometry, geometry_size, color, color_size);
		invertRAHTOctree_gpu[0].SetSlice(slice);
		for(int i=0; i<size; i++){
			cudaData[i].x = invertRAHTOctree_gpu[0].source_cloud_[i].x;
			cudaData[i].y = invertRAHTOctree_gpu[0].source_cloud_[i].y;
			cudaData[i].z = invertRAHTOctree_gpu[0].source_cloud_[i].z;
			cudaData[i].r = ((invertRAHTOctree_gpu[0].source_colors_[i].y + 1.4020f * (invertRAHTOctree_gpu[0].source_colors_[i].v - 128.0f))) / 255;
			cudaData[i].g = ((invertRAHTOctree_gpu[0].source_colors_[i].y - 0.3441f * (invertRAHTOctree_gpu[0].source_colors_[i].u - 128.0f) - 0.7141f * (invertRAHTOctree_gpu[0].source_colors_[i].v - 128.0f))) / 255;
			cudaData[i].b = ((invertRAHTOctree_gpu[0].source_colors_[i].y + 1.7720f * (invertRAHTOctree_gpu[0].source_colors_[i].u - 128.0f))) / 255;
		}
		// invertRAHTOctree_gpu[0].GetPatch();
	}
}

extern "C" void launch_cudaProcess(int grid, int block, Points * cudaData, int timestamp, int index, uint8_t type, float* mv, uint32_t size, uint8_t qp, uint8_t* geometry, uint32_t geometry_size, uint8_t* color, uint32_t color_size, vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu){
	processCUDA <<<grid, block >>> (cudaData, timestamp, index, type, mv, size, qp, geometry, geometry_size, color, color_size, invertRAHTOctree_gpu);
}
