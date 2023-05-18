/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-18 16:09:48
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "device_launch_parameters.h"
#include <helper_cuda.h>
#include "cuda/render.cuh"
#include "cuda/base.cuh"
#include <fstream>//文件流

#define VBOSIZE BUFFER_SIZE                 // GPU的缓冲区大小 (VBO缓冲区大小)
#define BUFFSIZE (BUFFER_SIZE*2)            // CPU的缓冲区大小 (将CPU的数据依次拷贝到VBO中进行渲染)

int frame_size = 0;

// 将 slice 拷贝到对应指针中
void frameCpu2Gpu(int *inner_offset_gpu, int *index_gpu, uint8_t *type_gpu, uint32_t *size_gpu, uint8_t *qp_gpu, uint32_t *geometry_size_gpu, uint32_t *color_size_gpu, float **mv_gpu, uint8_t **geometry_gpu, uint8_t **color_gpu,float **mid_mv, uint8_t **mid_geometry, uint8_t **mid_color, vvc::client::common::Frame_t& _frame, int patch_size){
    
    cudaMemcpy(index_gpu,           _frame.index,          sizeof(int) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(type_gpu,            _frame.type,           sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(size_gpu,            _frame.size,           sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(qp_gpu,              _frame.qp,             sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_size_gpu,   _frame.geometry_size,  sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_size_gpu,      _frame.color_size,     sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);

    int *inner_offset_cpu    = (int *)       malloc (sizeof(int)     * patch_size);
    int inner_offset = 0;
    for(int i=0; i<patch_size; i++){
        cudaMemcpy(mid_mv[i],       _frame.mv[i],       sizeof(float) * 16,                          cudaMemcpyHostToDevice);
        cudaMemcpy(mid_geometry[i], _frame.geometry[i], sizeof(uint8_t)  * _frame.geometry_size[i],  cudaMemcpyHostToDevice);
        cudaMemcpy(mid_color[i],    _frame.color[i],    sizeof(uint8_t)  * _frame.color_size[i],     cudaMemcpyHostToDevice);
        inner_offset_cpu[i] = inner_offset;
        inner_offset += _frame.size[i];
    }
    frame_size = inner_offset;
    // 最后统一的将多个 xxx 的地址拷贝到 mv_gpu 中
    cudaMemcpy(mv_gpu, mid_mv, sizeof(float *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_gpu, mid_geometry, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_gpu, mid_color, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);

    // 将一维数组进行拷贝
    cudaMemcpy(inner_offset_gpu,    inner_offset_cpu,   sizeof(int) * patch_size, cudaMemcpyHostToDevice);
}

int main()
{
    std::vector<std::vector<vvc::client::common::Points>> vertices(BUFFSIZE);//点数据容器对象
    std::vector<int> offset_list(BUFFSIZE);
    std::vector<int> size_list(BUFFSIZE);
    std::ifstream ifs;//文件流对象
    int offset = 0;
    
    // 读取两个压缩的数据
    vvc::client::common::Frame_t frame;
    if(vvc::client::io::LoadFrame(frame, "/home/lixin/vvc/test/data/result.frame") == 1){
        printf("LoadFrame Error\n");
        exit(1);
    }
    frame.index[1] += 1;
    const int patch_size = frame.slice_cnt;
    
    // 全局内存 因为 P 帧参考 I 帧
    vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu;
    cudaMalloc((void**)&(invertRAHTOctree_gpu), sizeof(vvc::client::octree::InvertRAHTOctree) * patch_size);

    int         *inner_offset_gpu;
    int         *index_gpu;
    uint8_t     *type_gpu;
    uint32_t    *size_gpu;
    uint8_t     *qp_gpu;
    uint32_t    *geometry_size_gpu;
    uint32_t    *color_size_gpu;

    float       **mv_gpu;
    uint8_t     **geometry_gpu;
    uint8_t     **color_gpu;

    // GPU 申请一维数组空间
    cudaMalloc((void**)&(inner_offset_gpu),     sizeof(int)     * patch_size);
    cudaMalloc((void**)&(index_gpu),            sizeof(int)     * patch_size);
    cudaMalloc((void**)&(type_gpu),             sizeof(uint8_t) * patch_size);
    cudaMalloc((void**)&(size_gpu),             sizeof(uint32_t)* patch_size);
    cudaMalloc((void**)&(qp_gpu),               sizeof(uint8_t) * patch_size);
    cudaMalloc((void**)&(geometry_size_gpu),    sizeof(uint32_t)* patch_size);
    cudaMalloc((void**)&(color_size_gpu),       sizeof(uint32_t)* patch_size);

    // GPU 申请二维数组空间
    cudaMalloc((void***)&(mv_gpu), sizeof(float *) * patch_size);
    cudaMalloc((void***)&(geometry_gpu), sizeof(uint8_t *) * patch_size);
    cudaMalloc((void***)&(color_gpu), sizeof(uint8_t *) * patch_size);
    
    // 申请二维数组
    float **mid_mv;
    mid_mv = (float**)malloc(sizeof(float *) * patch_size);
    uint8_t **mid_geometry;
    mid_geometry = (uint8_t**)malloc(sizeof(uint8_t *) * patch_size);
    uint8_t **mid_color;
    mid_color = (uint8_t**)malloc(sizeof(uint8_t *) * patch_size);
    // 申请相应的 GPU 内存空间
    for(int i=0; i<patch_size; i++){
        // 相当于 申请了 名为 xxx 的一块 GPU 内存空间  xxx 这个名字被存储在 mid_mv[i] 中
        cudaMalloc((void**)&(mid_mv[i]), sizeof(float) * 16);
        cudaMalloc((void**)&(mid_geometry[i]), sizeof(uint8_t)  * frame.geometry_size[i]);
        cudaMalloc((void**)&(mid_color[i]), sizeof(uint8_t)  * frame.color_size[i]);
    }

    // 声明对象
    vvc::client::render::Render renderer;
    renderer.InitWindow();
    std::cout<<"InitWindow"<<std::endl;

    renderer.InitOpenGL();
    std::cout<<"InitOpenGL"<<std::endl;

    int i=0;
    while (!glfwWindowShouldClose(renderer.window)){
        if( i==0 ){
            for(int j=0; j<100; j++){
                frameCpu2Gpu(inner_offset_gpu, index_gpu, type_gpu, size_gpu, qp_gpu, geometry_size_gpu, color_size_gpu, mv_gpu, geometry_gpu, color_gpu, mid_mv, mid_geometry, mid_color, frame, frame.slice_cnt); 

                renderer.CUDADecode(0, frame.timestamp, inner_offset_gpu, index_gpu, type_gpu, mv_gpu, size_gpu, qp_gpu, geometry_gpu, geometry_size_gpu, color_gpu, color_size_gpu, invertRAHTOctree_gpu, patch_size);

                renderer.Rendering(0, frame_size);
            }
        }
    }
    return 0;

}