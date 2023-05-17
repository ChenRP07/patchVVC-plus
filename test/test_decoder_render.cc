/*
 * @Author: lixin
 * @Date: 2023-05-06 12:50:36
 * @LastEditTime: 2023-05-16 21:59:56
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
/* 
 * Author: lixin
 * Date: 2023-05-06 12:50:36
 * LastEditTime: 2023-05-06 14:58:56
 * Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "device_launch_parameters.h"
#include <helper_cuda.h>
#include "cuda/render.cuh"
#include "cuda/base.cuh"
#include <fstream>//文件流

#define VBOSIZE BUFFER_SIZE                 // GPU的缓冲区大小 (VBO缓冲区大小)
#define BUFFSIZE (BUFFER_SIZE*2)            // CPU的缓冲区大小 (将CPU的数据依次拷贝到VBO中进行渲染)
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
    vvc::client::io::LoadSlice(slice[1], "/home/lixin/vvc/test/data/result_2_100.slice");

    // 全局内存 因为 P 帧参考 I 帧
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

    float* mv_gpu_2;
    cudaMalloc((void**)&(mv_gpu_2), sizeof(float) * 16);
    cudaMemcpy(mv_gpu_2, slice[1].mv.data, sizeof(float) * 16, cudaMemcpyHostToDevice);

    uint8_t* geometry_gpu_2;
    cudaMalloc((void**)&(geometry_gpu_2), sizeof(uint8_t) * slice[1].geometry_size);
    cudaMemcpy(geometry_gpu_2, slice[1].geometry, sizeof(uint8_t) * slice[1].geometry_size, cudaMemcpyHostToDevice);

    uint8_t* color_gpu_2;
    cudaMalloc((void**)&(color_gpu_2), sizeof(uint8_t) * slice[1].color_size);
    cudaMemcpy(color_gpu_2, slice[1].color, sizeof(uint8_t) * slice[1].color_size, cudaMemcpyHostToDevice);

    // 声明对象
    vvc::client::render::Render renderer;
    renderer.InitWindow();
    std::cout<<"InitWindow"<<std::endl;

    renderer.InitOpenGL();
    std::cout<<"InitOpenGL"<<std::endl;

   
    int inner_offset = 0;
    renderer.CUDADecode(inner_offset, slice[0].timestamp, slice[0].index, slice[0].type, mv_gpu, slice[0].size, slice[0].qp, geometry_gpu, slice[0].geometry_size, color_gpu, slice[0].color_size, invertRAHTOctree_gpu);

    while (!glfwWindowShouldClose(renderer.window)){ 
        renderer.Rendering(inner_offset, slice[0].size);
    }

    // int start = 0;
    // int end = start + BUFFER_SIZE;
    // while (!glfwWindowShouldClose(renderer.window)){
    //     renderer.InputData(vertices, start, end);
    //     int inner_offset = 0;
    //     for(int i=start; i<end; i++ ){
    //         printf("第 %d 帧 offset = %d size = %d\n", i, inner_offset, size_list[i]);
    //         for(int j=0; j<100; j++){
    //             renderer.CUDADecode(inner_offset, size_list[i], j);  
    //             renderer.Rendering(inner_offset, size_list[i]);
    //             if(glfwWindowShouldClose(renderer.window)){
    //                 break;
    //             }
    //         }
    //         inner_offset += size_list[i];
    //     }
    //     start = end;
    //     end = start + BUFFER_SIZE;
    //     if(end > BUFFSIZE){
    //         start = 0;
    //         end = start + BUFFER_SIZE;
    //     }
    // }

    return 0;

}
