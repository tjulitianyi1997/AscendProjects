/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
extern void leaky_relu_custom_do(uint32_t coreDim, void* l2ctrl, void* stream, uint8_t* x, uint8_t* y);
#else
#include "tikicpulib.h"
extern "C" __global__ __aicore__ void leaky_relu_custom(GM_ADDR x, GM_ADDR y);
#endif

int32_t main(int32_t argc, char* argv[])
{
    uint32_t blockDim = 8;
    size_t inputByteSize = 8 * 200 * 1024 * sizeof(float);
    size_t outputByteSize = 8 * 200 * 1024 * sizeof(float);

#ifdef __CCE_KT_TEST__
    uint8_t* x = (uint8_t*)AscendC::GmAlloc(inputByteSize);
    uint8_t* y = (uint8_t*)AscendC::GmAlloc(outputByteSize);

    ReadFile("./input/input_x.bin", inputByteSize, x, inputByteSize);
    AscendC::SetKernelMode(KernelMode::AIV_MODE);
    ICPU_RUN_KF(leaky_relu_custom, blockDim, x, y); // use this macro for cpu debug
    WriteFile("./output/output_y.bin", y, outputByteSize);
    AscendC::GmFree((void *)x);
    AscendC::GmFree((void *)y);
#else
    CHECK_ACL(aclInit(nullptr));
    aclrtContext context;
    int32_t deviceId = 0;
    CHECK_ACL(aclrtSetDevice(deviceId));
    CHECK_ACL(aclrtCreateContext(&context, deviceId));
    aclrtStream stream = nullptr;
    CHECK_ACL(aclrtCreateStream(&stream));

    uint8_t *xHost, *yHost;
    uint8_t *xDevice, *yDevice;
    CHECK_ACL(aclrtMallocHost((void**)(&xHost), inputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&yHost), outputByteSize));
    CHECK_ACL(aclrtMalloc((void**)&xDevice, inputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&yDevice, outputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));

    ReadFile("./input/input_x.bin", inputByteSize, xHost, inputByteSize);
    CHECK_ACL(aclrtMemcpy(xDevice, inputByteSize, xHost, inputByteSize, ACL_MEMCPY_HOST_TO_DEVICE));

    leaky_relu_custom_do(blockDim, nullptr, stream, xDevice, yDevice);
    CHECK_ACL(aclrtSynchronizeStream(stream));

    CHECK_ACL(aclrtMemcpy(yHost, outputByteSize, yDevice, outputByteSize, ACL_MEMCPY_DEVICE_TO_HOST));
    WriteFile("./output/output_y.bin", yHost, outputByteSize);

    CHECK_ACL(aclrtFree(xDevice));
    CHECK_ACL(aclrtFree(yDevice));
    CHECK_ACL(aclrtFreeHost(xHost));
    CHECK_ACL(aclrtFreeHost(yHost));

    CHECK_ACL(aclrtDestroyStream(stream));
    CHECK_ACL(aclrtDestroyContext(context));
    CHECK_ACL(aclrtResetDevice(deviceId));
    CHECK_ACL(aclFinalize());
#endif
    return 0;
}
