/*
 * @Author: lixin
 * @Date: 2023-05-16 21:12:20
 * @LastEditTime: 2023-05-22 22:10:17
 * @Description: 
 * Copyright (c) @lixin, All Rights Reserved.
 */
#include "cuda/manager.h"
int main()
{
    auto &instance  = vvc::client::Manager::Init();
    instance.Start(383, "/mnt/data/pvvc_data/loot/frame/loot_");
    return 0;
}
