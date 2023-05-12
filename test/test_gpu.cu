/*
 * @Author: lixin
 * @Date: 2023-05-10 11:28:12
 * @LastEditTime: 2023-05-12 17:16:55
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include<iostream>
#include<cstdio>
#include <cuda_runtime.h>
#include "cuda/octree.cuh"

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

__global__ void func(int* size){

   // int *a = (int *)malloc(sizeof(int) * size[0]);
   // for(int i=0; i<size[0]; i++){
   //    a[i] = i*10;
   // }
   // for(int i=0; i<size[0]; i++){
   //    printf("%d \n", a[i]);
   // }

   // dvector.push_back(10);
   // dvector.push_back(20);
   // printf("%d %d\n", dvector[0], dvector[1]);
    vvc::octree::InvertRAHTOctree Iro;
   printf("hello CUDA\n");
}

int main()
{
   int size = 10;
   // scanf("%d", &size);
   int* size_GPU;
   cudaMalloc((void **)&size_GPU, sizeof(int) );
   cudaMemcpy(size_GPU, &size, sizeof(int),cudaMemcpyHostToDevice);
   func<<<1,1>>>(size_GPU);
   cudaDeviceSynchronize();


    return 0;
}