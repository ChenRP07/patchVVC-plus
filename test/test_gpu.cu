/*
 * @Author: lixin
 * @Date: 2023-05-10 11:28:12
 * @LastEditTime: 2023-05-15 21:06:50
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

__global__ void cudaSigDecodeFunction(int timestamp, int index, uint8_t type, float* mv, uint32_t size, uint8_t qp, uint8_t* geometry, uint32_t geometry_size, uint8_t* color, uint32_t color_size){
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

   vvc::client::octree::InvertRAHTOctree invertRAHTOctree_gpu;
   printf("SetSlice\n");
   invertRAHTOctree_gpu.SetSlice(slice);

   printf("GetPatch\n");
   invertRAHTOctree_gpu.GetPatch();
}


int main()
{
   vvc::client::common::Slice_t slice;
   vvc::client::io::LoadSlice(slice, "/home/lixin/vvc/test/data/result_1_100.slice");

   float* mv_gpu;
   cudaMalloc((void**)&(mv_gpu), sizeof(float) * 16);
   cudaMemcpy(mv_gpu, slice.mv.data, sizeof(float) * 16, cudaMemcpyHostToDevice);

   uint8_t* geometry_gpu;
   cudaMalloc((void**)&(geometry_gpu), sizeof(uint8_t) * slice.geometry_size);
   cudaMemcpy(geometry_gpu, slice.geometry, sizeof(uint8_t) * slice.geometry_size, cudaMemcpyHostToDevice);

   uint8_t* color_gpu;
   cudaMalloc((void**)&(color_gpu), sizeof(uint8_t) * slice.color_size);
   cudaMemcpy(color_gpu, slice.color, sizeof(uint8_t) * slice.color_size, cudaMemcpyHostToDevice);

   cudaSigDecodeFunction<<<1,1>>>(slice.timestamp, slice.index, slice.type, mv_gpu, slice.size, slice.qp, geometry_gpu, slice.geometry_size, color_gpu, slice.color_size);
   cudaDeviceSynchronize();

    return 0;
}