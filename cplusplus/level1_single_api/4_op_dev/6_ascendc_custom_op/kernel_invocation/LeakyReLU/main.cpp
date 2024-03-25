/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */
#include "data_utils.h"
#include "leakyrelu_custom_tiling.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
extern void leakyrelu_custom_do(uint32_t coreDim, void* l2ctrl, void* stream,
    uint8_t *x, uint8_t *y, uint8_t *workspace, uint8_t *tiling);
#else
#include "tikicpulib.h"
extern "C" __global__ __aicore__ void leakyrelu_custom(GM_ADDR x, GM_ADDR y, GM_ADDR workspace, GM_ADDR tiling);
#endif

int32_t main(int32_t argc, char* argv[])
{
    size_t tilingSize = sizeof(LeakyReluCustomTilingData);
    size_t usrWorkSpaceSize = 4096;
    size_t sysWorkSpaceSize = 16 * 1024 * 1024;
    uint32_t blockDim = 8;

#ifdef __CCE_KT_TEST__
    uint8_t *usrWorkSpace = (uint8_t *)AscendC::GmAlloc(usrWorkSpaceSize);
    uint8_t *tiling = (uint8_t *)AscendC::GmAlloc(tilingSize);
    ReadFile("./input/tiling.bin", tilingSize, tiling, tilingSize);

    size_t inputByteSize = blockDim * 200 *1024 * sizeof(u_int16_t);
    size_t outputByteSize = blockDim * 200 *1024 * sizeof(u_int16_t);

    uint8_t *x = (uint8_t *)AscendC::GmAlloc(inputByteSize);
    uint8_t *y = (uint8_t *)AscendC::GmAlloc(outputByteSize);

    ReadFile("./input/input_x.bin", inputByteSize, x, inputByteSize);

    AscendC::SetKernelMode(KernelMode::AIV_MODE);


    ICPU_RUN_KF(leakyrelu_custom, blockDim, x, y, usrWorkSpace, tiling);

    WriteFile("./output/output_y.bin", y, outputByteSize);

    AscendC::GmFree((void *)x);
    AscendC::GmFree((void *)y);
    AscendC::GmFree((void *)usrWorkSpace);
    AscendC::GmFree((void *)tiling);
#else
    CHECK_ACL(aclInit(nullptr));
    aclrtContext context;
    int32_t deviceId = 0;
    CHECK_ACL(aclrtSetDevice(deviceId));
    CHECK_ACL(aclrtCreateContext(&context, deviceId));
    aclrtStream stream = nullptr;
    CHECK_ACL(aclrtCreateStream(&stream));

    uint8_t *xHost, *yHost, *tilingHost, *workspaceHost;
    uint8_t *xDevice, *yDevice, *tilingDevice, *workspaceDevice;


    CHECK_ACL(aclrtMallocHost((void**)(&tilingHost), tilingSize));
    ReadFile("./input/tiling.bin", tilingSize, tilingHost, tilingSize);
    CHECK_ACL(aclrtMallocHost((void**)(&workspaceHost), tilingSize));
    
    size_t inputByteSize = blockDim * 200 *1024 * sizeof(u_int16_t);
    size_t outputByteSize = blockDim * 200 *1024 * sizeof(u_int16_t);
    size_t workSpaceByteSize = sysWorkSpaceSize + usrWorkSpaceSize;

    CHECK_ACL(aclrtMallocHost((void**)(&xHost), inputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&yHost), inputByteSize));
    CHECK_ACL(aclrtMallocHost((void**)(&workspaceHost), workSpaceByteSize));
    CHECK_ACL(aclrtMalloc((void**)&xDevice, inputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&yDevice, inputByteSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&tilingDevice, tilingSize, ACL_MEM_MALLOC_HUGE_FIRST));
    CHECK_ACL(aclrtMalloc((void**)&workspaceDevice, workSpaceByteSize, ACL_MEM_MALLOC_HUGE_FIRST));

    ReadFile("./input/input_x.bin", inputByteSize, xHost, inputByteSize);
    CHECK_ACL(aclrtMemcpy(xDevice, inputByteSize, xHost, inputByteSize, ACL_MEMCPY_HOST_TO_DEVICE));
    CHECK_ACL(aclrtMemcpy(tilingDevice, tilingSize, tilingHost, tilingSize, ACL_MEMCPY_HOST_TO_DEVICE));

    leakyrelu_custom_do(blockDim, nullptr, stream,xDevice,yDevice,workspaceDevice,tilingDevice);
    CHECK_ACL(aclrtSynchronizeStream(stream));
    CHECK_ACL(aclrtMemcpy(yHost, outputByteSize, yDevice, outputByteSize, ACL_MEMCPY_DEVICE_TO_HOST));
    WriteFile("./output/output_y.bin", yHost, outputByteSize);

    CHECK_ACL(aclrtFree(xDevice));
    CHECK_ACL(aclrtFree(yDevice));
    CHECK_ACL(aclrtFree(workspaceDevice));
    CHECK_ACL(aclrtFree(tilingDevice));
    CHECK_ACL(aclrtFreeHost(xHost));
    CHECK_ACL(aclrtFreeHost(yHost));
    CHECK_ACL(aclrtFreeHost(workspaceHost));
    CHECK_ACL(aclrtFreeHost(tilingHost));

    CHECK_ACL(aclrtDestroyStream(stream));
    CHECK_ACL(aclrtDestroyContext(context));
    CHECK_ACL(aclrtResetDevice(deviceId));
    CHECK_ACL(aclFinalize());
#endif
    return 0;
}