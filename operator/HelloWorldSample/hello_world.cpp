/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 */
#include "kernel_operator.h"
using namespace AscendC;
extern "C" __global__ __aicore__ void hello_world() {
    PRINTF("Hello World!!!\n");
}

void hello_world_do(uint32_t blockDim, void* stream) {
    hello_world<<<blockDim, nullptr, stream>>>();
}