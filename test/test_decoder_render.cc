/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-18 15:15:10
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
void sliceCpu2Gpu(int *inner_offset_gpu, int *index_gpu, uint8_t *type_gpu, uint32_t *size_gpu, uint8_t *qp_gpu, uint32_t *geometry_size_gpu, uint32_t *color_size_gpu, float **mv_gpu, uint8_t **geometry_gpu, uint8_t **color_gpu,float **mid_mv, uint8_t **mid_geometry, uint8_t **mid_color, vvc::client::common::Slice_t slice[], int patch_size){
    // 声明一维 CPU 变量
    int         *inner_offset_cpu;
    int         *index_cpu;
    uint8_t     *type_cpu;
    uint32_t    *size_cpu;
    uint8_t     *qp_cpu;
    uint32_t    *geometry_size_cpu;
    uint32_t    *color_size_cpu;

    // 申请相应空间
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
    index_cpu[1]++;

    // 声明二维 CPU 变量
    float **mv_cpu;
    uint8_t **geometry_cpu;
    uint8_t **color_cpu;

    // 申请相应空间
    mv_cpu          = (float **)    malloc (sizeof(float *) * patch_size);
    geometry_cpu    = (uint8_t **)  malloc (sizeof(uint8_t *) * patch_size);
    color_cpu       = (uint8_t **)  malloc (sizeof(uint8_t *) * patch_size);

    // 填充 cpu 数组
    for(int i=0; i<patch_size; i++){
        mv_cpu[i]       = (float *)    malloc (sizeof(float)    * 16);
        geometry_cpu[i] = (uint8_t *)  malloc (sizeof(uint8_t)  * slice[i].geometry_size);
        color_cpu[i]    = (uint8_t *)  malloc (sizeof(uint8_t)  * slice[i].color_size);

        memcpy(mv_cpu[i],       slice[i].mv.data,   sizeof(float) * 16);
        memcpy(geometry_cpu[i], slice[i].geometry,  sizeof(uint8_t) * slice[i].geometry_size);
        memcpy(color_cpu[i],    slice[i].color,     sizeof(uint8_t) * slice[i].color_size);
    }

    // 将一维数组进行拷贝
    cudaMemcpy(inner_offset_gpu,    inner_offset_cpu,   sizeof(int) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(index_gpu,           index_cpu,          sizeof(int) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(type_gpu,            type_cpu,           sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(size_gpu,            size_cpu,           sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(qp_gpu,              qp_cpu,             sizeof(uint8_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_size_gpu,   geometry_size_cpu,  sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_size_gpu,      color_size_cpu,     sizeof(uint32_t) * patch_size, cudaMemcpyHostToDevice);

    for(int i=0; i<patch_size; i++){
        cudaMemcpy(mid_mv[i], mv_cpu[i], sizeof(float) * 16, cudaMemcpyHostToDevice);
        cudaMemcpy(mid_geometry[i], geometry_cpu[i], sizeof(uint8_t)  * slice[i].geometry_size, cudaMemcpyHostToDevice);
        cudaMemcpy(mid_color[i], color_cpu[i], sizeof(uint8_t)  * slice[i].color_size, cudaMemcpyHostToDevice);
    }

    // 最后统一的将多个 xxx 的地址拷贝到 mv_gpu 中
    cudaMemcpy(mv_gpu, mid_mv, sizeof(float *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(geometry_gpu, mid_geometry, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(color_gpu, mid_color, sizeof(uint8_t *) * patch_size, cudaMemcpyHostToDevice);

    free(inner_offset_cpu);
    free(index_cpu);
    free(type_cpu);
    free(size_cpu);
    free(qp_cpu);
    free(geometry_size_cpu);
    free(color_size_cpu);
    for(int i=0; i<patch_size; i++){
        free(mv_cpu[i]);
        free(geometry_cpu[i]);
        free(color_cpu[i]);
    }
    free(mv_cpu);
    free(geometry_cpu);
    free(color_cpu);
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

    vvc::client::common::Slice_t slice_p[patch_size];
    vvc::client::io::LoadSlice(slice_p[0], "/home/lixin/vvc/test/data/result_2_100.slice");
    vvc::client::io::LoadSlice(slice_p[1], "/home/lixin/vvc/test/data/result_2_100.slice");

    slice_p[0].mv.data[7] += 100.0f;
    
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
        cudaMalloc((void**)&(mid_geometry[i]), sizeof(uint8_t)  * slice[i].geometry_size);
        cudaMalloc((void**)&(mid_color[i]), sizeof(uint8_t)  * slice[i].color_size);
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
                sliceCpu2Gpu(inner_offset_gpu, index_gpu, type_gpu, size_gpu, qp_gpu, geometry_size_gpu, color_size_gpu, mv_gpu, geometry_gpu, color_gpu, mid_mv, mid_geometry, mid_color, slice, patch_size); 
                renderer.CUDADecode(0, slice[0].timestamp, inner_offset_gpu, index_gpu, type_gpu, mv_gpu, size_gpu, qp_gpu, geometry_gpu, geometry_size_gpu, color_gpu, color_size_gpu, invertRAHTOctree_gpu, patch_size);
                renderer.Rendering(0, slice[0].size + slice[1].size);
            }
            i = 1;
        }
        if( i==1 ){
            for(int j=0; j<100; j++){
                sliceCpu2Gpu(inner_offset_gpu, index_gpu, type_gpu, size_gpu, qp_gpu, geometry_size_gpu, color_size_gpu, mv_gpu, geometry_gpu, color_gpu, mid_mv, mid_geometry, mid_color, slice_p, patch_size);
                renderer.CUDADecode(0, slice_p[i].timestamp, inner_offset_gpu, index_gpu, type_gpu, mv_gpu, size_gpu, qp_gpu, geometry_gpu, geometry_size_gpu, color_gpu, color_size_gpu, invertRAHTOctree_gpu, patch_size);
                renderer.Rendering(0, slice_p[0].size + slice_p[1].size);
            }
            i = 0;
        }
    }
    return 0;

}
