/*
 * @Author: lixin
 * @Date: 2023-05-22 20:21:44
 * @LastEditTime: 2023-05-23 16:38:57
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "cuda/manager.h"
using namespace vvc::client;
int main()
{

    size_t size{1024 * 1024 * 1024};
    int patch_size = 383;
    cudaDeviceSetLimit(cudaLimitMallocHeapSize, size);
    octree::InvertRAHTOctree* tmpDecoders;
    octree::InvertRAHTOctree* temp_Decoders = new octree::InvertRAHTOctree[patch_size];
    // printf("Malloc GPU memory ......\n");
    gpuErrchk(cudaMalloc((void**)&(tmpDecoders), sizeof(octree::InvertRAHTOctree) *patch_size));
    gpuErrchk(cudaMemcpy(tmpDecoders, temp_Decoders, sizeof(octree::InvertRAHTOctree) *patch_size, cudaMemcpyHostToDevice));
    delete[] temp_Decoders;
    for (int ttt = 0; ttt < 30; ++ttt) {
        
        common::Points* tmpCudaData;
        
        CudaFrame_t tmpCUDAFrame;
        gpuErrchk(cudaMalloc((void**)&(tmpCudaData), sizeof(common::Points) * 1000000));

        /* GPU 申请一维数组空间 */
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.inner_offset_gpu), sizeof(int) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.index_gpu), sizeof(int) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.type_gpu), sizeof(uint8_t) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.size_gpu), sizeof(uint32_t) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.qp_gpu), sizeof(uint8_t) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.geometry_size_gpu), sizeof(uint32_t) *patch_size));
        gpuErrchk(cudaMalloc((void**)&(tmpCUDAFrame.color_size_gpu), sizeof(uint32_t) *patch_size));

        /* GPU 申请二维数组空间 */
        gpuErrchk(cudaMalloc((void***)&(tmpCUDAFrame.mv_gpu), sizeof(float*) *patch_size));
        gpuErrchk(cudaMalloc((void***)&(tmpCUDAFrame.geometry_gpu), sizeof(uint8_t*) *patch_size));
        gpuErrchk(cudaMalloc((void***)&(tmpCUDAFrame.color_gpu), sizeof(uint8_t*) *patch_size));

        float** tmp_mv    = (float**)malloc(sizeof(float*) *patch_size);
        uint8_t** tmp_geo   = (uint8_t**)malloc(sizeof(uint8_t*) *patch_size);
        uint8_t** tmp_color = (uint8_t**)malloc(sizeof(uint8_t*) *patch_size);

        for (int i = 0; i <patch_size; ++i) {
            gpuErrchk(cudaMalloc((void**)&(tmp_mv[i]), sizeof(float) * 16));
            gpuErrchk(cudaMalloc((void**)&(tmp_geo[i]), sizeof(uint8_t) * MAX_SLICE_SIZE));
            gpuErrchk(cudaMalloc((void**)&(tmp_color[i]), sizeof(uint8_t) * MAX_SLICE_SIZE));
        }

        common::Frame_t frame_p;
        io::LoadFrame(frame_p, "/mnt/data/pvvc_data/loot/frame/loot_"+std::to_string(ttt)+".frame");
        int frame_point_cnt{};
        for (int idx = 0; idx < frame_p.slice_cnt; ++idx) {
            frame_point_cnt += frame_p.size[idx];
        }


        auto& _frame = frame_p;
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.index_gpu, _frame.index, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.type_gpu, _frame.type, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.size_gpu, _frame.size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.qp_gpu, _frame.qp, sizeof(uint8_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.geometry_size_gpu, _frame.geometry_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.color_size_gpu, _frame.color_size, sizeof(uint32_t) * _frame.slice_cnt, cudaMemcpyHostToDevice));

        int* inner_offset_cpu = (int*)malloc(sizeof(int) * _frame.slice_cnt);
        int  inner_offset     = 0;
        int  point_num = 0;
        for (int i = 0; i < _frame.slice_cnt; i++) {
            gpuErrchk(cudaMemcpy(tmp_mv[i], _frame.mv[i], sizeof(float) * 16, cudaMemcpyHostToDevice));
            gpuErrchk(cudaMemcpy(tmp_geo[i], _frame.geometry[i], sizeof(uint8_t) * _frame.geometry_size[i], cudaMemcpyHostToDevice));
            gpuErrchk(cudaMemcpy(tmp_color[i], _frame.color[i], sizeof(uint8_t) * _frame.color_size[i], cudaMemcpyHostToDevice));
            inner_offset_cpu[i] = inner_offset;
            inner_offset += _frame.size[i];
        }
        point_num = inner_offset;

        // frame_size = inner_offset;
        // 最后统一的将多个 xxx 的地址拷贝到 mv_gpu 中
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.mv_gpu, tmp_mv, sizeof(float*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.geometry_gpu, tmp_geo, sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.color_gpu, tmp_color, sizeof(uint8_t*) * _frame.slice_cnt, cudaMemcpyHostToDevice));

        // 将一维数组进行拷贝
        gpuErrchk(cudaMemcpy(tmpCUDAFrame.inner_offset_gpu, inner_offset_cpu, sizeof(int) * _frame.slice_cnt, cudaMemcpyHostToDevice));
        free(inner_offset_cpu);
        


        int numElements =patch_size;
        int blockSize   = 1;
        int numBlocks   = (numElements + blockSize - 1) / blockSize;

        vvc::client::render::launch_cudaProcess(numBlocks, blockSize, tmpCudaData, 0, 
                            tmpCUDAFrame.inner_offset_gpu,
                            tmpCUDAFrame.index_gpu,
                            tmpCUDAFrame.type_gpu,
                            tmpCUDAFrame.mv_gpu,
                            tmpCUDAFrame.size_gpu,
                            tmpCUDAFrame.qp_gpu,
                            tmpCUDAFrame.geometry_gpu,
                            tmpCUDAFrame.geometry_size_gpu,
                            tmpCUDAFrame.color_gpu,
                            tmpCUDAFrame.color_size_gpu, 
                            tmpDecoders, 
                            patch_size);

        gpuErrchk(cudaDeviceSynchronize());

        // 将数据拷贝到 CPU
        common::Points* tmpCudaData_cpu = (common::Points *)malloc(sizeof(common::Points) * 1000000);
        gpuErrchk(cudaMemcpy(tmpCudaData_cpu, tmpCudaData, sizeof(common::Points)* point_num, cudaMemcpyDeviceToHost));

        printf("frame = %d num = %d\n",ttt, point_num);
        for(int i=0; i<point_num; i++){
            printf("%.2f %.2f %.2f %.0f %.0f %.0f\n",tmpCudaData_cpu[i].x, tmpCudaData_cpu[i].y, tmpCudaData_cpu[i].z ,tmpCudaData_cpu[i].r*255, tmpCudaData_cpu[i].g*255, tmpCudaData_cpu[i].b*255);
        }

    }
}