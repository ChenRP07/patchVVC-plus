/*
 * @Author: lixin
 * @Date: 2023-05-10 11:28:12
 * @LastEditTime: 2023-05-15 21:04:25
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include<iostream>
#include<cstdio>
#include <cuda_runtime.h>
#include "cuda/octree.cuh"
#include "cuda/entropy_codec.cuh"
#include <string>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

// __device__ vvc::client::octree::InvertRAHTOctree invertRAHTOctree;

__global__ void cudaMultDecodeFunction(int timestamp, int index, uint8_t type, float* mv, uint32_t size, uint8_t qp, uint8_t* geometry, uint32_t geometry_size, uint8_t* color, uint32_t color_size, vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu){
    vvc::client::common::Slice_t slice(timestamp, index, type, mv, size, qp, geometry, geometry_size, color, color_size);
    printf("GPU: slice.timestamp = %d\n", slice.timestamp);
    printf("GPU: slice.index = %d\n", slice.index);
    printf("GPU: slice.type = %02x\n", slice.type);
    printf("GPU: slice.size = %d\n", slice.size);
    printf("GPU: slice.qp = %d\n", slice.qp);
    printf("GPU: slice.geometry_size = %d\n", slice.geometry_size);
    printf("GPU: slice.color_size = %d\n", slice.color_size);

    printf("slice.color = %p\n", slice.color);
    for(int i=0; i<slice.color_size; i++){
        printf("%02x ", slice.color[i]);
    }
    printf("\n");

    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            printf("%.2f ",slice.mv.data[i*4+j]);
        }
        printf("\n");
    }

    printf("SetSlice\n");
    invertRAHTOctree_gpu[0].SetSlice(slice);

    printf("GetPatch\n");
    invertRAHTOctree_gpu[0].GetPatch();
}

int main()
{
    const int patch_size = 2;
    vvc::client::common::Slice_t slice[patch_size];
    vvc::client::io::LoadSlice(slice[0], "/home/lixin/vvc/test/data/result_1_100.slice");
    vvc::client::io::LoadSlice(slice[1], "/home/lixin/vvc/test/data/result_2_100.slice");

    // 全局内存
    vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu;
    cudaMalloc((void**)&(invertRAHTOctree_gpu), sizeof(vvc::client::octree::InvertRAHTOctree));

    float* mv_gpu;
    cudaMalloc((void**)&(mv_gpu), sizeof(float) * 16);
    cudaMemcpy(mv_gpu, slice[0].mv.data, sizeof(float) * 16, cudaMemcpyHostToDevice);

    uint8_t* geometry_gpu;
    cudaMalloc((void**)&(geometry_gpu), sizeof(uint8_t) * slice[0].geometry_size);
    cudaMemcpy(geometry_gpu, slice[0].geometry, sizeof(uint8_t) * slice[0].geometry_size, cudaMemcpyHostToDevice);

    uint8_t* color_gpu;
    cudaMalloc((void**)&(color_gpu), sizeof(uint8_t) * slice[0].color_size);
    cudaMemcpy(color_gpu, slice[0].color, sizeof(uint8_t) * slice[0].color_size, cudaMemcpyHostToDevice);

    cudaMultDecodeFunction<<<1,1>>>(slice[0].timestamp, slice[0].index, slice[0].type, mv_gpu, slice[0].size, slice[0].qp, geometry_gpu, slice[0].geometry_size, color_gpu, slice[0].color_size, invertRAHTOctree_gpu);
    cudaDeviceSynchronize();

    float* mv_gpu_2;
    cudaMalloc((void**)&(mv_gpu_2), sizeof(float) * 16);
    cudaMemcpy(mv_gpu_2, slice[1].mv.data, sizeof(float) * 16, cudaMemcpyHostToDevice);

    uint8_t* geometry_gpu_2;
    cudaMalloc((void**)&(geometry_gpu_2), sizeof(uint8_t) * slice[1].geometry_size);
    cudaMemcpy(geometry_gpu_2, slice[1].geometry, sizeof(uint8_t) * slice[1].geometry_size, cudaMemcpyHostToDevice);

    uint8_t* color_gpu_2;
    cudaMalloc((void**)&(color_gpu_2), sizeof(uint8_t) * slice[1].color_size);
    cudaMemcpy(color_gpu_2, slice[1].color, sizeof(uint8_t) * slice[1].color_size, cudaMemcpyHostToDevice);

    cudaMultDecodeFunction<<<1,1>>>(slice[1].timestamp, slice[1].index, slice[1].type, mv_gpu_2, slice[1].size, slice[1].qp, geometry_gpu_2, slice[1].geometry_size, color_gpu_2, slice[1].color_size, invertRAHTOctree_gpu);
    cudaDeviceSynchronize();

    return 0;
}