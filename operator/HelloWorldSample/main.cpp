/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "acl/acl.h"
extern void hello_world_do(uint32_t coreDim, void* stream);

int32_t main(int argc, char const *argv[]) {
    aclInit(nullptr);
    aclrtContext context;
    int32_t deviceId = 0;
    aclrtSetDevice(deviceId);
    aclrtCreateContext(&context,deviceId);
    aclrtStream stream = nullptr;
    aclrtCreateStream(&stream);
    
    constexpr uint32_t blockDim = 8;
    hello_world_do(blockDim, stream);
    aclrtSynchronizeStream(stream);

    aclrtDestroyStream(stream);
    aclrtDestroyContext(context);
    aclrtResetDevice(deviceId);
    aclFinalize();
    return 0;
}
