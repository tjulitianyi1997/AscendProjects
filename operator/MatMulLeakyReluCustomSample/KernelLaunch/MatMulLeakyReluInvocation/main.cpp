/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */

#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
#include "aclrtlaunch_matmul_leakyrelu_custom.h"
#else
#include "tikicpulib.h"
extern "C" void matmul_leakyrelu_custom(uint8_t *, uint8_t *, uint8_t *,
                                        uint8_t *, uint8_t *, uint8_t *);
#endif
extern uint8_t *GenerateTiling();

int32_t main(int32_t argc, char *argv[]) {
  size_t aFileSize = 262144 * sizeof(int16_t);
  size_t bFileSize = 163840 * sizeof(int16_t);
  size_t cFileSize = 655360 * sizeof(float);
  size_t biasFileSize = 640 * sizeof(float);
  size_t tilingFileSize = 48 * sizeof(int32_t);
  size_t workspaceFileSize = 100663296 * sizeof(float);
#ifdef CUSTOM_ASCEND310P
  uint32_t blockDim = 2;
#else
  uint32_t blockDim = 1;
#endif

#ifdef __CCE_KT_TEST__
  uint8_t *a = (uint8_t *)AscendC::GmAlloc(aFileSize);
  uint8_t *b = (uint8_t *)AscendC::GmAlloc(bFileSize);
  uint8_t *bias = (uint8_t *)AscendC::GmAlloc(biasFileSize);
  uint8_t *c = (uint8_t *)AscendC::GmAlloc(cFileSize);
  uint8_t *tiling = (uint8_t *)AscendC::GmAlloc(tilingFileSize);
  uint8_t *workspace = (uint8_t *)AscendC::GmAlloc(workspaceFileSize);

  ReadFile("./input/x1_gm.bin", aFileSize, a, aFileSize);
  // PrintData(a, 262144, printDataType::HALF);
  ReadFile("./input/x2_gm.bin", bFileSize, b, bFileSize);
  // PrintData(b, 163840, printDataType::HALF);
  ReadFile("./input/bias.bin", biasFileSize, bias, biasFileSize);
  // PrintData(c, 640, printDataType::FLOAT);
  memcpy_s(tiling, tilingFileSize, GenerateTiling(), tilingFileSize);
  // PrintData(tiling, 32, printDataType::INT32_T);
  ReadFile("./input/workspace.bin", workspaceFileSize, workspace,
           workspaceFileSize);
  // PrintData(workspace, 100663296, printDataType::FLOAT);
  ICPU_RUN_KF(matmul_leakyrelu_custom, blockDim, a, b, bias, c, workspace,
              tiling);

  // PrintData(bias, 655360, printDataType::FLOAT);
  WriteFile("./output/cpp_cpu_output.bin", c, cFileSize);
  AscendC::GmFree((void *)a);
  AscendC::GmFree((void *)b);
  AscendC::GmFree((void *)bias);
  AscendC::GmFree((void *)c);
  AscendC::GmFree((void *)tiling);
  AscendC::GmFree((void *)workspace);
#else
  CHECK_ACL(aclInit(nullptr));
  aclrtContext context;
  int32_t deviceId = 0;
  CHECK_ACL(aclrtSetDevice(deviceId));
  CHECK_ACL(aclrtCreateContext(&context, deviceId));
  aclrtStream stream = nullptr;
  CHECK_ACL(aclrtCreateStream(&stream));

  uint8_t *inputAHost;
  uint8_t *inputADevice;
  CHECK_ACL(aclrtMallocHost((void **)(&inputAHost), aFileSize));
  CHECK_ACL(aclrtMalloc((void **)&inputADevice, aFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/x1_gm.bin", aFileSize, inputAHost, aFileSize);
  // PrintData(inputAHost, 262144, printDataType::HALF);
  CHECK_ACL(aclrtMemcpy(inputADevice, aFileSize, inputAHost, aFileSize,
                        ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *inputBHost;
  uint8_t *inputBDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&inputBHost), bFileSize));
  CHECK_ACL(aclrtMalloc((void **)&inputBDevice, bFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/x2_gm.bin", bFileSize, inputBHost, bFileSize);
  // PrintData(inputBHost, 163840, printDataType::HALF);
  CHECK_ACL(aclrtMemcpy(inputBDevice, bFileSize, inputBHost, bFileSize,
                        ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *outputCHost;
  uint8_t *outputCDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&outputCHost), cFileSize));
  CHECK_ACL(aclrtMalloc((void **)&outputCDevice, cFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));

  uint8_t *inputBiasHost;
  uint8_t *inputBiasDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&inputBiasHost), biasFileSize));
  CHECK_ACL(aclrtMalloc((void **)&inputBiasDevice, biasFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/bias.bin", biasFileSize, inputBiasHost, biasFileSize);
  // PrintData(inputBiasHost, 640, printDataType::FLOAT);
  CHECK_ACL(aclrtMemcpy(inputBiasDevice, biasFileSize, inputBiasHost,
                        biasFileSize, ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *tilingHost;
  uint8_t *tilingDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&tilingHost), tilingFileSize));
  CHECK_ACL(aclrtMalloc((void **)&tilingDevice, tilingFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  CHECK_ACL(aclrtMemcpy(tilingHost, tilingFileSize, GenerateTiling(),
                        tilingFileSize, ACL_MEMCPY_HOST_TO_HOST));
  // PrintData(tilingHost, 48, printDataType::INT32_T);
  CHECK_ACL(aclrtMemcpy(tilingDevice, tilingFileSize, tilingHost,
                        tilingFileSize, ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *workspaceHost;
  uint8_t *workspaceDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&workspaceHost), workspaceFileSize));
  CHECK_ACL(aclrtMalloc((void **)&workspaceDevice, workspaceFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/workspace.bin", workspaceFileSize, workspaceHost,
           workspaceFileSize);
  // PrintData(workspaceHost, 100663296, printDataType::FLOAT);
  CHECK_ACL(aclrtMemcpy(workspaceDevice, workspaceFileSize, workspaceHost,
                        workspaceFileSize, ACL_MEMCPY_HOST_TO_DEVICE));
  ACLRT_LAUNCH_KERNEL(matmul_leakyrelu_custom)
  (blockDim, stream, inputADevice, inputBDevice, inputBiasDevice, outputCDevice,
   workspaceDevice, tilingDevice);

  CHECK_ACL(aclrtSynchronizeStream(stream));

  CHECK_ACL(aclrtFree(inputADevice));
  CHECK_ACL(aclrtFreeHost(inputAHost));
  CHECK_ACL(aclrtFree(inputBDevice));
  CHECK_ACL(aclrtFreeHost(inputBHost));
  CHECK_ACL(aclrtMemcpy(outputCHost, cFileSize, outputCDevice, cFileSize,
                        ACL_MEMCPY_DEVICE_TO_HOST));
  // PrintData(outputCHost, 655360, printDataType::FLOAT);
  WriteFile("./output/cpp_cpu_output.bin", outputCHost, cFileSize);
  CHECK_ACL(aclrtFree(outputCDevice));
  CHECK_ACL(aclrtFreeHost(outputCHost));
  CHECK_ACL(aclrtFree(inputBiasDevice));
  CHECK_ACL(aclrtFreeHost(inputBiasHost));
  CHECK_ACL(aclrtFree(tilingDevice));
  CHECK_ACL(aclrtFreeHost(tilingHost));
  CHECK_ACL(aclrtFree(workspaceDevice));
  CHECK_ACL(aclrtFreeHost(workspaceHost));

  CHECK_ACL(aclrtDestroyStream(stream));
  CHECK_ACL(aclrtDestroyContext(context));
  CHECK_ACL(aclrtResetDevice(deviceId));
  CHECK_ACL(aclFinalize());
#endif
  return 0;
}