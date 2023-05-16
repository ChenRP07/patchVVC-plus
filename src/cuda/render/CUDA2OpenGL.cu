/*
 * @Author: lixin
 * @Date: 2023-05-16 11:47:17
 * @LastEditTime: 2023-05-16 20:33:47
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "device_launch_parameters.h"
#include <helper_cuda.h>
// #include "cuda/render.cuh"

struct Points
{
	float x, y, z;
	float r, g, b;
};

__global__ void processCUDA(Points* cudaData, size_t numElements, int frame_number)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;

	if (idx < numElements){
		cudaData[idx].x = cudaData[idx].x + frame_number * 0.02;
	}
}

extern "C" void launch_cudaProcess(int grid, int block, Points * cudaData, size_t numBytes, int frame_number){
	processCUDA <<<grid, block >>> (cudaData, numBytes, frame_number);
}
