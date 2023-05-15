/*
 * @Author: lixin
 * @Date: 2023-05-10 11:28:12
 * @LastEditTime: 2023-05-15 10:51:19
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include<iostream>
#include<cstdio>
#include <cuda_runtime.h>
#include "io/slice_io.h"
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

__global__ void func(uint8_t* data, int e_size, int d_size){
   vvc::client::common::RLGRDecoder dec;
   dec.Decode(data, e_size, d_size);
   vvc::client::common::FIX_DATA_INT* res = dec.GetResult();
   for(int i=0; i<d_size; i++){
      printf("%ld,", res[i]);
   }
   printf("\ne_size = %d, d_size = %d\n", e_size, d_size);
}

int main()
{
   vvc::common::Slice slice;
   vvc::io::LoadSlice(slice, "/home/lixin/vvc/test/data/result_1_100.slice");

   std::cout<< slice.timestamp <<std::endl;
   std::cout<< slice.index <<std::endl;
   printf("%02x\n", slice.type);
   std::cout<< slice.mv <<std::endl;
   std::cout<< slice.size <<std::endl;
   printf("%d\n", slice.qp );
   std::cout<< slice.geometry->size() <<std::endl;
   std::cout<< slice.color->size() <<std::endl;

   // for(int i=0; i<4; i++){
   //    for(int j=0; j<4; j++){
   //       std::cout<<slice.mv(i)(j)<<" ";
   //    }
   //    std::cout<<std::endl;
   // }

   std::string str ="123";
   std::cout<<str<<std::endl;

   // int e_size = 16;
   // uint8_t cpu_data[16] = {0x06, 0x91, 0x87, 0xfd, 0x10, 0xbf, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x32, 0x00, 0x25, 0x70};
   // uint8_t *gpu_data;
   // cudaMalloc((void **)&gpu_data, sizeof(uint8_t)*e_size);
   // cudaMemcpy(gpu_data, cpu_data, sizeof(uint8_t)*e_size, cudaMemcpyHostToDevice);
   // int d_size = 20;
   // func<<<1,1>>>(gpu_data, e_size, d_size);
   // cudaDeviceSynchronize();

   // // int size = 10;
   // // // scanf("%d", &size);
   // // int* size_GPU;
   // // cudaMalloc((void **)&size_GPU, sizeof(int) );
   // // cudaMemcpy(size_GPU, &size, sizeof(int),cudaMemcpyHostToDevice);
   // // func<<<1,1>>>(size_GPU);
   // // cudaDeviceSynchronize();


    return 0;
}