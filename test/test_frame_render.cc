/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-19 10:25:48
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "cuda/manager.h"
int main()
{
    auto &instance  = vvc::client::Manager::Init();
    instance.Start(1, "/home/lixin/vvc/test/data/result_1.txt");
    return 0;
}
