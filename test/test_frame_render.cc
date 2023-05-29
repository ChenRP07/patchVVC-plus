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
    // instance.Start(1000, "/mnt/data/pvvc_data/loot/frame/loot_");
    // instance.Start(1000, "/mnt/data/pvvc_data/soldier/frame/soldier_");
    // instance.Start(1000, "/mnt/data/pvvc_data/longdress/frame/longdress_");
    instance.Start(1000, "/mnt/data/pvvc_data/redandblack/frame/redandblack_");
    return 0;
}
