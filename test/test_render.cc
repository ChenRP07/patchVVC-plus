/*
 * @Author: lixin
 * @Date: 2023-05-06 12:50:36
 * @LastEditTime: 2023-05-16 20:33:41
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
    for(int index=0; index<BUFFSIZE; index++){
        int order = 1051 + index;
        ifs.open("/mnt/data/pcdataset/8idataset/longdress/longdress_vox10_"+ std::to_string(order) +".ply", std::ios::in);
        
        if (!ifs.is_open()){
            std::cout << "文件打开失败！" << std::endl;
            return -1;
        }

        for (int i = 0; i < 14; ++i) {
            ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        struct vvc::client::common::Points tempPoint, tempOtherData;//临时存储数据对象
        while (ifs >> tempPoint.x >> tempPoint.y >> tempPoint.z >> tempPoint.r >> tempPoint.g >> tempPoint.b)//从流中输入数据到结构体_点中，注意后两列数据输入到结构体_其他
        {
            vertices[index].push_back(tempPoint);//将结构体_点的值添加至vector容器的最后面，循环直到文件被输入完毕
        }
        int frame_size = vertices[index].size();
        std::cout << frame_size << std::endl;

        for (int i = 0; i < vertices[index].size(); i++)
        {
            vertices[index][i].r = vertices[index][i].r * 1.0 / 255;
            vertices[index][i].g = vertices[index][i].g * 1.0 / 255;
            vertices[index][i].b = vertices[index][i].b * 1.0 / 255;
        }

        size_list[index] = vertices[index].size();
        offset_list[index] = offset;
        offset += size_list[index];
        std::cout << vertices[index].size() << "数据已导入" << std::endl;
        ifs.close();
    }

    // 声明对象
    vvc::client::render::Render renderer;
    renderer.InitWindow();
    std::cout<<"InitWindow"<<std::endl;

    renderer.InitOpenGL();
    std::cout<<"InitOpenGL"<<std::endl;

    int start = 0;
    int end = start + BUFFER_SIZE;
    while (!glfwWindowShouldClose(renderer.window)){
        renderer.InputData(vertices, start, end);
        int inner_offset = 0;
        for(int i=start; i<end; i++ ){
            printf("第 %d 帧 offset = %d size = %d\n", i, inner_offset, size_list[i]);
            for(int j=0; j<100; j++){
                renderer.CUDADecode(inner_offset, size_list[i], j);  
                renderer.Rendering(inner_offset, size_list[i]);
                if(glfwWindowShouldClose(renderer.window)){
                    break;
                }
            }
            inner_offset += size_list[i];
        }
        start = end;
        end = start + BUFFER_SIZE;
        if(end > BUFFSIZE){
            start = 0;
            end = start + BUFFER_SIZE;
        }
    }

    return 0;

}
