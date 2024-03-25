/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 * This file constains code of cpu debug and npu code.We read data from bin file
 * and write result to file.
 */

#include "data_utils.h"
#ifndef __CCE_KT_TEST__
#include "acl/acl.h"
#include "aclrtlaunch_matmul_custom.h"
#else
#include "tikicpulib.h"
extern "C" void matmul_custom(uint8_t *a, uint8_t *b, uint8_t *c,
                              uint8_t *workspace, uint8_t *tiling);
#endif
extern uint8_t *GenerateTiling();

int32_t main(int32_t argc, char *argv[]) {
  size_t aFileSize = 512 * 512 * sizeof(uint16_t);   // uint16_t represent half
  size_t bFileSize = 512 * 1024 * sizeof(uint16_t);  // uint16_t represent half
  size_t cFileSize = 512 * 1024 * sizeof(float);
  uint32_t workspaceSize = 16 * 1024 * 1024;
  size_t tilingFileSize = 48 * sizeof(uint32_t);
#ifdef CUSTOM_ASCEND310P
  uint32_t blockDim = 2;
#else
  uint32_t blockDim = 1;
#endif

#ifdef __CCE_KT_TEST__
  uint8_t *a = (uint8_t *)AscendC::GmAlloc(aFileSize);
  uint8_t *b = (uint8_t *)AscendC::GmAlloc(bFileSize);
  uint8_t *c = (uint8_t *)AscendC::GmAlloc(cFileSize);
  uint8_t *workspace = (uint8_t *)AscendC::GmAlloc(workspaceSize);
  uint8_t *tiling = (uint8_t *)AscendC::GmAlloc(tilingFileSize);

  ReadFile("./input/x1_gm.bin", aFileSize, a, aFileSize);
  // PrintData(a, 16, printDataType::HALF);
  ReadFile("./input/x2_gm.bin", bFileSize, b, bFileSize);
  // PrintData(b, 16, printDataType::HALF);
  memcpy_s(tiling, tilingFileSize, GenerateTiling(), tilingFileSize);
  // PrintData(tiling, 16, printDataType::UINT32_T);

  ICPU_RUN_KF(matmul_custom, blockDim, a, b, c, workspace, tiling);

  // PrintData(c, 16, printDataType::FLOAT);
  WriteFile("./output/output.bin", c, cFileSize);

  AscendC::GmFree((void *)a);
  AscendC::GmFree((void *)b);
  AscendC::GmFree((void *)c);
  AscendC::GmFree((void *)workspace);
  AscendC::GmFree((void *)tiling);
#else
  CHECK_ACL(aclInit(nullptr));
  aclrtContext context;
  int32_t deviceId = 0;
  CHECK_ACL(aclrtSetDevice(deviceId));
  CHECK_ACL(aclrtCreateContext(&context, deviceId));
  aclrtStream stream = nullptr;
  CHECK_ACL(aclrtCreateStream(&stream));

  uint8_t *aHost;
  uint8_t *aDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&aHost), aFileSize));
  CHECK_ACL(
      aclrtMalloc((void **)&aDevice, aFileSize, ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/x1_gm.bin", aFileSize, aHost, aFileSize);
  // PrintData(aHost, 16, printDataType::HALF);
  CHECK_ACL(aclrtMemcpy(aDevice, aFileSize, aHost, aFileSize,
                        ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *bHost;
  uint8_t *bDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&bHost), bFileSize));
  CHECK_ACL(
      aclrtMalloc((void **)&bDevice, bFileSize, ACL_MEM_MALLOC_HUGE_FIRST));
  ReadFile("./input/x2_gm.bin", bFileSize, bHost, bFileSize);
  // PrintData(bHost, 16, printDataType::HALF);
  CHECK_ACL(aclrtMemcpy(bDevice, bFileSize, bHost, bFileSize,
                        ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *workspaceHost;
  uint8_t *workspaceDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&workspaceHost), workspaceSize));
  CHECK_ACL(aclrtMalloc((void **)&workspaceDevice, workspaceSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));

  uint8_t *tilingHost;
  uint8_t *tilingDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&tilingHost), tilingFileSize));
  CHECK_ACL(aclrtMalloc((void **)&tilingDevice, tilingFileSize,
                        ACL_MEM_MALLOC_HUGE_FIRST));
  CHECK_ACL(aclrtMemcpy(tilingHost, tilingFileSize, GenerateTiling(),
                        tilingFileSize, ACL_MEMCPY_HOST_TO_HOST));
  // PrintData(tilingHost, 16, printDataType::UINT32_T);
  CHECK_ACL(aclrtMemcpy(tilingDevice, tilingFileSize, tilingHost,
                        tilingFileSize, ACL_MEMCPY_HOST_TO_DEVICE));

  uint8_t *cHost;
  uint8_t *cDevice;
  CHECK_ACL(aclrtMallocHost((void **)(&cHost), cFileSize));
  CHECK_ACL(
      aclrtMalloc((void **)&cDevice, cFileSize, ACL_MEM_MALLOC_HUGE_FIRST));

  ACLRT_LAUNCH_KERNEL(matmul_custom)
  (blockDim, stream, aDevice, bDevice, cDevice, workspaceDevice, tilingDevice);
  CHECK_ACL(aclrtSynchronizeStream(stream));

  CHECK_ACL(aclrtMemcpy(cHost, cFileSize, cDevice, cFileSize,
                        ACL_MEMCPY_DEVICE_TO_HOST));
  // PrintData(cHost, 16, printDataType::FLOAT);
  WriteFile("./output/output.bin", cHost, cFileSize);
  CHECK_ACL(aclrtFree(cDevice));
  CHECK_ACL(aclrtFreeHost(cHost));

  CHECK_ACL(aclrtDestroyStream(stream));
  CHECK_ACL(aclrtDestroyContext(context));
  CHECK_ACL(aclrtResetDevice(deviceId));
  CHECK_ACL(aclFinalize());
#endif
  return 0;
}