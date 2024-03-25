/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2022-2023. All rights reserved.
 */
#include "kernel_operator.h"
using namespace AscendC;

constexpr int32_t TOTAL_LENGTH = 8 * 200 * 1024;                            // total length of data
constexpr int32_t USE_CORE_NUM = 8;                                   // num of core used
constexpr int32_t BLOCK_LENGTH = TOTAL_LENGTH / USE_CORE_NUM;         // length computed of each core
constexpr int32_t TILE_NUM = 16;                                       // split data into 8 tiles for each core
constexpr int32_t BUFFER_NUM = 2;                                     // tensor num for each queue
constexpr int32_t TILE_LENGTH = BLOCK_LENGTH / TILE_NUM / BUFFER_NUM; // seperate to 2 parts, due to double buffer
constexpr float NEGATIVESLOPE = 0.0;

class KernelLeakyRelu {
public:
    __aicore__ inline KernelLeakyRelu() {}
    __aicore__ inline void Init(GM_ADDR x, GM_ADDR y)
    {
        ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");
        // get start index for current core, core parallel
        xGm.SetGlobalBuffer((__gm__ float*)x + BLOCK_LENGTH * GetBlockIdx(), BLOCK_LENGTH);
        yGm.SetGlobalBuffer((__gm__ float*)y + BLOCK_LENGTH * GetBlockIdx(), BLOCK_LENGTH);
        // pipe alloc memory to queue, the unit is Bytes
        pipe.InitBuffer(inQueueX, BUFFER_NUM, TILE_LENGTH * sizeof(float));
        pipe.InitBuffer(outQueueY, BUFFER_NUM, TILE_LENGTH * sizeof(float));
        pipe.InitBuffer(tmpBuffer1, TILE_LENGTH * sizeof(float));
        pipe.InitBuffer(tmpBuffer2, TILE_LENGTH * sizeof(float));
    }
    __aicore__ inline void Process()
    {
        // loop count need to be doubled, due to double buffer
        constexpr int32_t loopCount = TILE_NUM * BUFFER_NUM;
        // tiling strategy, pipeline parallel
        for (int32_t i = 0; i < loopCount; i++) {
            CopyIn(i);
            Compute(i);
            CopyOut(i);
        }
    }

private:
    __aicore__ inline void CopyIn(int32_t progress)
    {
        // alloc tensor from queue memory
        LocalTensor<float> xLocal = inQueueX.AllocTensor<float>();
        // copy progress_th tile from global tensor to local tensor
        DataCopy(xLocal, xGm[progress * TILE_LENGTH], TILE_LENGTH);
        // enque input tensors to VECIN queue
        inQueueX.EnQue(xLocal);
    }
    __aicore__ inline void Compute(int32_t progress)
    {
        // deque input tensors from VECIN queue
        LocalTensor<float> xLocal = inQueueX.DeQue<float>();
        LocalTensor<float> yLocal = outQueueY.AllocTensor<float>();
        LocalTensor<float> tmpTensor1 = tmpBuffer1.Get<float>();
        LocalTensor<float> tmpTensor2 = tmpBuffer2.Get<float>();
        float inputVal = 0.0;
        Maxs(tmpTensor1, xLocal, inputVal, TILE_LENGTH);
        Mins(tmpTensor2, xLocal, inputVal, TILE_LENGTH);
        Muls(tmpTensor2, tmpTensor2, NEGATIVESLOPE, TILE_LENGTH);
        Add(yLocal, tmpTensor1, tmpTensor2, TILE_LENGTH);
        // enque the output tensor to VECOUT queue
        outQueueY.EnQue<float>(yLocal);
        // free input tensors for reuse
        inQueueX.FreeTensor(xLocal);
    }
    __aicore__ inline void CopyOut(int32_t progress)
    {
        // deque output tensor from VECOUT queue
        LocalTensor<float> yLocal = outQueueY.DeQue<float>();
        // copy progress_th tile from local tensor to global tensor
        DataCopy(yGm[progress * TILE_LENGTH], yLocal, TILE_LENGTH);
        // free output tensor for reuse
        outQueueY.FreeTensor(yLocal);
    }

private:
    TPipe pipe;
    TBuf<QuePosition::VECCALC> tmpBuffer1, tmpBuffer2;
    // create queues for input, in this case depth is equal to buffer num
    TQue<QuePosition::VECIN, BUFFER_NUM> inQueueX;
    // create queue for output, in this case depth is equal to buffer num
    TQue<QuePosition::VECOUT, BUFFER_NUM> outQueueY;
    GlobalTensor<float> xGm, yGm;
};

// implementation of kernel function
extern "C" __global__ __aicore__ void leaky_relu_custom(GM_ADDR x, GM_ADDR y)
{
    KernelLeakyRelu op;
    op.Init(x, y);
    op.Process();
}

#ifndef __CCE_KT_TEST__
// call of kernel function
void leaky_relu_custom_do(uint32_t blockDim, void* l2ctrl, void* stream, uint8_t* x, uint8_t* y)
{
    leaky_relu_custom<<<blockDim, l2ctrl, stream>>>(x, y);
}
#endif
