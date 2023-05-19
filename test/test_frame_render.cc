/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-19 14:56:35
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "cuda/manager.h"
int main()
{
    auto &instance  = vvc::client::Manager::Init();
    instance.Start(5, "/home/lixin/vvc/test/data/test_");
    return 0;
}
