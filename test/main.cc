/*
 * @Author: lixin
 * @Date: 2023-05-06 12:50:36
 * @LastEditTime: 2023-05-06 15:37:03
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
#include "cuda/render/render.cuh"
#include "cuda/common/base.cuh"
#include <fstream>//文件流

#define BUFFSIZE 1
int main()
{
    std::vector<vvc::common::Points> vertices[BUFFSIZE];//点数据容器对象
    // std::vector<std::string> fileName ={"longdress_1055.txt", "longdress_1100.txt", "longdress_1150.txt", "longdress_1200.txt"};
    std::ifstream ifs;//文件流对象
    for(int index=0; index<BUFFSIZE; index++){
        //读取文件
        // std::cout<<"../"+fileName[index]<<std::endl;
        // ifs.open("../"+fileName[index], std::ios::in);//1、文件所在路径(使用了相对路径)；2、文件以输入方式打开(文件数据输入到内存)

        int order = 1051 + index;
        ifs.open("/mnt/data/pcdataset/8idataset/longdress/longdress_vox10_"+ std::to_string(order) +".ply", std::ios::in);
        
        if (!ifs.is_open())
        {
            std::cout << "文件打开失败！" << std::endl;
            return -1;
        }

        for (int i = 0; i < 15; ++i) {
            ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        struct vvc::common::Points tempPoint, tempOtherData;//临时存储数据对象
        int a, b, c;
        while (ifs >> tempPoint.x >> tempPoint.y >> tempPoint.z >> tempPoint.r >> tempPoint.g >> tempPoint.b)//从流中输入数据到结构体_点中，注意后两列数据输入到结构体_其他
        {
            vertices[index].push_back(tempPoint);//将结构体_点的值添加至vector容器的最后面，循环直到文件被输入完毕
        }
        int frame_size = vertices[index].size();
        std::cout << frame_size << std::endl;

        for (int i = 0; i < vertices[index].size(); i++)
        {
            // ///坐标变换参数
            // vertices[index][i].x = vertices[index][i].x * 1.0 / 1000;
            // vertices[index][i].y = vertices[index][i].y * 1.0 / 1000;
            // vertices[index][i].z = vertices[index][i].z * 1.0 / 1000;

            vertices[index][i].r = vertices[index][i].r * 1.0 / 255;
            vertices[index][i].g = vertices[index][i].g * 1.0 / 255;
            vertices[index][i].b = vertices[index][i].b * 1.0 / 255;
        }

        std::cout << vertices[index].size() << "数据已导入" << std::endl;
        ifs.close();
    }

    // 声明对象
    vvc::render::Render renderer;
    renderer.InitWindow();
    std::cout<<"InitWindow"<<std::endl;

    renderer.InitOpenGL();
    std::cout<<"InitOpenGL"<<std::endl;

    int index = 0;
    while (!glfwWindowShouldClose(renderer.window)){
        renderer.InputData(vertices[index % BUFFSIZE]);
        std::cout<<"InputData"<<std::endl;
        renderer.Rendering();
        index ++;
    }

    return 0;

}
