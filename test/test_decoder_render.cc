/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-17 16:16:32
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

// 将 slice 拷贝到对应指针中
void sliceCpu2Gpu(){
    
}

int main()
{
    std::vector<std::vector<vvc::client::common::Points>> vertices(BUFFSIZE);//点数据容器对象
    std::vector<int> offset_list(BUFFSIZE);
    std::vector<int> size_list(BUFFSIZE);
    std::ifstream ifs;//文件流对象
    int offset = 0;
    
    // 读取两个压缩的数据
    const int patch_size = 2;
    vvc::client::common::Slice_t slice[patch_size];
    vvc::client::io::LoadSlice(slice[0], "/home/lixin/vvc/test/data/result_1_100.slice");
    vvc::client::io::LoadSlice(slice[1], "/home/lixin/vvc/test/data/result_1_100.slice");

    // 全局内存 因为 P 帧参考 I 帧
    vvc::client::octree::InvertRAHTOctree* invertRAHTOctree_gpu;
    cudaMalloc((void**)&(invertRAHTOctree_gpu), sizeof(vvc::client::octree::InvertRAHTOctree) * patch_size);

    int         *inner_offset_cpu,  *inner_offset_gpu;
    int         *index_cpu,         *index_gpu;
    uint8_t     *type_cpu,          *type_gpu;
    uint32_t    *size_cpu,          *size_gpu;
    uint8_t     *qp_cpu,            *qp_gpu;
    uint32_t    *geometry_size_cpu, *geometry_size_gpu;
    uint32_t    *color_size_cpu,    *color_size_gpu;

    inner_offset_cpu    = (int *)       malloc (sizeof(int)     * patch_size);
    index_cpu           = (int *)       malloc (sizeof(int)     * patch_size);
    type_cpu            = (uint8_t *)   malloc (sizeof(uint8_t) * patch_size);
    size_cpu            = (uint32_t *)  malloc (sizeof(uint32_t)* patch_size);
    qp_cpu              = (uint8_t *)   malloc (sizeof(uint8_t) * patch_size);
    geometry_size_cpu   = (uint32_t *)  malloc (sizeof(uint32_t)* patch_size);
    color_size_cpu      = (uint32_t *)  malloc (sizeof(uint32_t)* patch_size);

    // 填充 cpu 数组
    int inner_offset = 0;
    for(int i=0; i<patch_size; i++){
        inner_offset_cpu[i]     = inner_offset;
        index_cpu[i]            = slice[i].index;
        type_cpu[i]             = slice[i].type;
        size_cpu[i]             = slice[i].size;
        qp_cpu[i]               = slice[i].qp;
        geometry_size_cpu[i]    = slice[i].geometry_size;
        color_size_cpu[i]       = slice[i].color_size;
        inner_offset            += slice[i].size;
    }

    float **mv_cpu, **mv_gpu;
    uint8_t **geometry_cpu, **geometry_gpu;
    uint8_t **color_cpu, **color_gpu;

    // 填充 cpu 数组
    mv_cpu          = (float **)    malloc (sizeof(float *) * patch_size);
    geometry_cpu    = (uint8_t **)  malloc (sizeof(uint8_t *) * patch_size);
    color_cpu       = (uint8_t **)  malloc (sizeof(uint8_t *) * patch_size);

    for(int i=0; i<patch_size; i++){
        mv_cpu[i]       = (float *)    malloc (sizeof(float)    * 16);
        geometry_cpu[i] = (uint8_t *)  malloc (sizeof(uint8_t)  * slice[i].geometry_size);
        color_cpu[i]    = (uint8_t *)  malloc (sizeof(uint8_t)  * slice[i].color_size);

        memcpy(mv_cpu[i],       slice[i].mv.data,   sizeof(float) * 16);
        memcpy(geometry_cpu[i], slice[i].geometry,  sizeof(uint8_t) * slice[i].geometry_size);
        memcpy(color_cpu[i],    slice[i].color,     sizeof(uint8_t) * slice[i].color_size);
    }

    // 一维数组的拷贝
    cudaMalloc((void**)&(inner_offset_gpu),     sizeof(int)     * patch_size);
    cudaMalloc((void**)&(index_gpu),            sizeof(int)     * patch_size);
    cudaMalloc((void**)&(type_gpu),             sizeof(uint8_t) * patch_size);
    cudaMalloc((void**)&(size_gpu),             sizeof(uint32_t)* patch_size);
    cudaMalloc((void**)&(qp_gpu),               sizeof(uint8_t) * patch_size);
    cudaMalloc((void**)&(geometry_size_gpu),    sizeof(uint32_t)* patch_size);
    cudaMalloc((void**)&(color_size_gpu),       sizeof(uint32_t)* patch_size);

    cudaMemcpy(inner_offset_gpu,    inner_offset_cpu,   sizeof(int) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(index_gpu,           index_cpu,          sizeof(int) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(type_gpu,            type_cpu,           sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(size_gpu,            size_cpu,           sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(qp_gpu,              qp_cpu,             sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_size_gpu,   geometry_size_cpu,  sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_size_gpu,      color_size_cpu,     sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);

    // 二维数组的拷贝
    cudaMalloc((void***)&(mv_gpu), sizeof(float *) * patch_size);
    cudaMalloc((void***)&(geometry_gpu), sizeof(uint8_t *) * patch_size);
    cudaMalloc((void***)&(color_gpu), sizeof(uint8_t *) * patch_size);
    
    float **mid_mv;
    mid_mv = (float**)malloc(sizeof(float *) * patch_size);
    uint8_t **mid_geometry;
    mid_geometry = (uint8_t**)malloc(sizeof(uint8_t *) * patch_size);
    uint8_t **mid_color;
    mid_color = (uint8_t**)malloc(sizeof(uint8_t *) * patch_size);

    for(int i=0; i<patch_size; i++){
        cudaMalloc((void**)&(mid_mv[i]), sizeof(float) * 16);
        cudaMemcpy(mid_mv[i], mv_cpu[i], sizeof(float) * 16, cudaMemcpyHostToDevice);

        cudaMalloc((void**)&(mid_geometry[i]), sizeof(uint8_t)  * slice[i].geometry_size);
        cudaMemcpy(mid_geometry[i], geometry_cpu[i], sizeof(uint8_t)  * slice[i].geometry_size, cudaMemcpyHostToDevice);

        cudaMalloc((void**)&(mid_color[i]), sizeof(uint8_t)  * slice[i].color_size);
        cudaMemcpy(mid_color[i], color_cpu[i], sizeof(uint8_t)  * slice[i].color_size, cudaMemcpyHostToDevice);
    }
    cudaMemcpy(mv_gpu, mid_mv, sizeof(float *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_gpu, mid_geometry, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_gpu, mid_color, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);

    
    // 声明对象
    vvc::client::render::Render renderer;
    renderer.InitWindow();
    std::cout<<"InitWindow"<<std::endl;

    renderer.InitOpenGL();
    std::cout<<"InitOpenGL"<<std::endl;

    int i=0;
    while (!glfwWindowShouldClose(renderer.window)){
        renderer.CUDADecode(0, slice[i].timestamp, inner_offset_gpu, index_gpu, type_gpu, mv_gpu, size_gpu, qp_gpu, geometry_gpu, geometry_size_gpu, color_gpu, color_size_gpu, invertRAHTOctree_gpu, patch_size);
        renderer.Rendering(0, slice[0].size + slice[1].size);
    }

    return 0;

}
