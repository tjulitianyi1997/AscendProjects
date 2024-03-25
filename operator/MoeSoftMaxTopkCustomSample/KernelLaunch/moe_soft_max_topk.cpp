#include "kernel_operator.h"

using namespace AscendC;
constexpr uint32_t REDUCE_SUM_ONE_REPEAT = 256;
constexpr uint32_t EACH_BLOCK_SIZE = 32;
constexpr uint32_t BUFFER_NUM = 1;
constexpr uint32_t TILE_NUM = 1;
constexpr uint32_t USE_CORE_NUM = 32;

class KernelSoftmaxTopK {
 public:
  __aicore__ inline KernelSoftmaxTopK() {}
  __aicore__ inline void Init(GM_ADDR x, GM_ADDR y, GM_ADDR indices,
                              uint32_t k) {
    ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");

    if (k == 4) {
      this->totalLength = 1024 * 16;
      this->lastDim = 16;

    } else {
      this->totalLength = 2048 * 32;
      this->lastDim = 32;
    }
    this->k = k;
    this->tileNum = TILE_NUM;
    this->blockLength = this->totalLength / USE_CORE_NUM;
    this->outputLength =
        this->totalLength / this->lastDim / USE_CORE_NUM * this->k;
    ASSERT(tileNum != 0 && "tileNum can not be zero!");
    this->tileLength = this->blockLength / this->tileNum / BUFFER_NUM;
    this->outputTileLength = this->outputLength / this->tileNum / BUFFER_NUM;

    xGm.SetGlobalBuffer((__gm__ float *)x + this->blockLength * GetBlockIdx(),
                        this->blockLength);
    yGm.SetGlobalBuffer((__gm__ float *)y + this->outputLength * GetBlockIdx(),
                        this->outputLength);
    indicesGm.SetGlobalBuffer(
        (__gm__ float *)indices + this->outputLength * GetBlockIdx(),
        this->outputLength);

    pipe.InitBuffer(inQueueX, BUFFER_NUM, this->tileLength * sizeof(float));
    pipe.InitBuffer(outQueueY, BUFFER_NUM,
                    this->outputTileLength * sizeof(float));
    pipe.InitBuffer(outQueueIndices, BUFFER_NUM,
                    this->outputTileLength * sizeof(float));

    pipe.InitBuffer(reduceRes,
                    this->tileLength / this->lastDim * sizeof(float));
    pipe.InitBuffer(
        topKRes, this->tileLength * (32 / this->lastDim) * 2 * sizeof(float));
    pipe.InitBuffer(
        workLocal, this->tileLength * (32 / this->lastDim) * 2 * sizeof(float));
    pipe.InitBuffer(topKIndices, this->lastDim * 3 * sizeof(uint32_t));

    LocalTensor<uint32_t> indicesInitLocal = topKIndices.Get<uint32_t>();
    GenerateInitIndex(indicesInitLocal);
  }
  __aicore__ inline void Process() {
    if (GetBlockIdx() >= USE_CORE_NUM) {
      return;
    }
    int32_t loopCount = this->tileNum * BUFFER_NUM;
    for (int32_t i = 0; i < loopCount; i++) {
      CopyIn(i);
      Compute(i);
      CopyOut(i);
    }
  }

 private:
  __aicore__ inline void CopyIn(int32_t progress) {
    LocalTensor<float> xLocal = inQueueX.AllocTensor<float>();
    DataCopy(xLocal, xGm[progress * this->tileLength], this->tileLength);
    inQueueX.EnQue(xLocal);
  }

  __aicore__ inline void Compute(int32_t progress) {
    LocalTensor<float> xLocal = inQueueX.DeQue<float>();
    LocalTensor<float> yLocal = outQueueY.AllocTensor<float>();
    LocalTensor<float> indicesLocal = outQueueIndices.AllocTensor<float>();
    DoSoftMax(xLocal, progress);
    DoTopK<float>(xLocal, yLocal, indicesLocal);
    inQueueX.FreeTensor(xLocal);
  }

  __aicore__ inline void CopyOut(int32_t progress) {
    LocalTensor<float> yLocal = outQueueY.DeQue<float>();
    DataCopy(yGm[progress * this->outputTileLength], yLocal,
             this->outputTileLength);
    outQueueY.FreeTensor(yLocal);

    LocalTensor<float> indicesLocalTensor = outQueueIndices.DeQue<float>();
    DataCopy(indicesGm[progress * this->outputTileLength], indicesLocalTensor,
             this->outputTileLength);
    outQueueIndices.FreeTensor(indicesLocalTensor);
  }

  template <typename T>
  __aicore__ inline void DoSoftMax(const LocalTensor<T> &srcLocalTensor,
                                   int32_t progress) {
    Exp(srcLocalTensor, srcLocalTensor, this->tileLength);

    LocalTensor<float> reduceTensor = reduceRes.Get<float>();
    DoReduceSum(srcLocalTensor, reduceTensor);
    LocalTensor<float> reduceBroadcastTensor = workLocal.Get<float>();
    for (uint32_t i = 0; i < this->tileLength / this->lastDim; i++) {
      Duplicate(reduceBroadcastTensor, reduceTensor.GetValue(i), this->lastDim);
      Div(srcLocalTensor[i * this->lastDim], srcLocalTensor[i * this->lastDim],
          reduceBroadcastTensor, this->lastDim);
    }
  }

  template <typename T>
  __aicore__ inline void DoReduceSum(const LocalTensor<T> &srcLocalTensor,
                                     const LocalTensor<T> &reduceTensor) {
    uint32_t repeatTimes = this->tileLength / this->lastDim;
    constexpr uint32_t one_step_time = REDUCE_SUM_ONE_REPEAT / sizeof(T);
    if (this->lastDim >= one_step_time) {
      uint64_t mask = one_step_time;
      uint32_t dstRepStride = 1;
      uint32_t srcBlkStride = 1;
      uint32_t srcRepStride = one_step_time * sizeof(T) / EACH_BLOCK_SIZE;
      WholeReduceSum(reduceTensor, srcLocalTensor, mask, repeatTimes,
                     dstRepStride, srcBlkStride, srcRepStride);
    } else {
      uint64_t blockStride = this->lastDim * sizeof(T) / EACH_BLOCK_SIZE;
      if (blockStride == 0) {
        for (uint32_t i = 0; i < repeatTimes; ++i) {
          reduceTensor.SetValue(i, srcLocalTensor.GetValue(i * this->lastDim));
          for (int j = 1; j < this->lastDim; ++j) {
            reduceTensor.SetValue(
                i, reduceTensor.GetValue(i) +
                       srcLocalTensor.GetValue(i * this->lastDim + j));
          }
        }
      } else {
        uint64_t mask = this->lastDim;
        uint32_t dstRepStride = 1;
        uint32_t srcBlkStride = 1;
        uint32_t srcRepStride = this->lastDim * sizeof(T) / EACH_BLOCK_SIZE;
        WholeReduceSum(reduceTensor, srcLocalTensor, mask, repeatTimes,
                       dstRepStride, srcBlkStride, srcRepStride);
      }
    }
  }

  template <typename T>
  __aicore__ inline void DoTopK(const LocalTensor<T> &srcLocalTensor,
                                const LocalTensor<T> &yLocalTensor,
                                const LocalTensor<T> &indicesLocalTensor) {
    LocalTensor<uint32_t> indicesInitLocal = topKIndices.Get<uint32_t>();
    LocalTensor<uint32_t> topkInitTensor = indicesInitLocal;
    LocalTensor<uint32_t> gatherScoreTensor = indicesInitLocal[this->lastDim];
    LocalTensor<uint32_t> gatherIndicesTensor =
        indicesInitLocal[this->lastDim * 2];

    LocalTensor<T> topKResTensor = topKRes.Get<T>();
    LocalTensor<float> workLocalTensor = workLocal.Get<float>();
    T ZERO(0);
    if (this->lastDim < 32) {
      Duplicate(workLocalTensor, ZERO, this->tileLength * 2);
      uint32_t blockBase = EACH_BLOCK_SIZE / sizeof(T);
      uint16_t blockStride = 1;
      uint16_t dstRepeatSize = 32 / blockBase;
      uint16_t srcRepeatSize = this->lastDim / blockBase;
      uint32_t repeat = this->tileLength / this->lastDim;
      Copy(workLocalTensor, srcLocalTensor, this->lastDim, repeat,
           {1, 1, dstRepeatSize, srcRepeatSize});
      DoSort32(topKResTensor, workLocalTensor, topkInitTensor);
    } else {
      DoSort32(topKResTensor, srcLocalTensor, topkInitTensor);
    }
    if (this->lastDim < 32) {
      Duplicate(workLocalTensor, ZERO, this->tileLength * 2);
      DoGatherMask(workLocalTensor, topKResTensor, gatherScoreTensor);
      DoCopyToRes(yLocalTensor, workLocalTensor);

      Duplicate(workLocalTensor, ZERO, this->tileLength * 2);
      DoGatherMask(workLocalTensor, topKResTensor, gatherIndicesTensor);
      DoCopyToRes(indicesLocalTensor, workLocalTensor);
    } else {
      DoGatherMask(workLocalTensor, topKResTensor, gatherScoreTensor);
      DoCopyToRes(yLocalTensor, workLocalTensor);

      Duplicate(workLocalTensor, ZERO, this->tileLength * 2);
      DoGatherMask(workLocalTensor, topKResTensor, gatherIndicesTensor);
      DoCopyToRes(indicesLocalTensor, workLocalTensor);
    }

    outQueueY.EnQue<float>(yLocalTensor);
    outQueueIndices.EnQue<float>(indicesLocalTensor);
  }

  template <typename T>
  __aicore__ inline void DoSort32(const LocalTensor<T> &topKResTensor,
                                  const LocalTensor<T> &srcLocalTensor,
                                  const LocalTensor<uint32_t> &indicesTensor) {
    for (uint32_t i = 0; i < this->tileLength / this->lastDim; ++i) {
      Sort32(topKResTensor[i * 32 * 2], srcLocalTensor[i * 32], indicesTensor,
             1);
    }
  }

  template <typename T>
  __aicore__ inline void DoGatherMask(
      const LocalTensor<T> &workLocalTensor,
      const LocalTensor<T> &topKResTensor,
      const LocalTensor<uint32_t> &gatherMaskTensor) {
    uint8_t src0BlockStride = 1;
    uint16_t gatherRepeatTimes = this->tileLength / this->lastDim;
    uint16_t src0RepeatStride = 8;
    uint8_t src1RepeatStride = 0;
    uint32_t mask = REDUCE_SUM_ONE_REPEAT / sizeof(T);
    uint64_t rsvdCnt = 0;
    GatherMaskParams gatherMaskParams = {src0BlockStride, gatherRepeatTimes,
                                         src0RepeatStride, src1RepeatStride};
    GatherMask(workLocalTensor, topKResTensor, gatherMaskTensor, true, mask,
               gatherMaskParams, rsvdCnt);
  }

  template <typename T>
  __aicore__ inline void DoCopyToRes(const LocalTensor<T> &dstTensor,
                                     const LocalTensor<T> &srcTensor) {
    uint32_t mask = REDUCE_SUM_ONE_REPEAT / sizeof(T);
    if (this->outputTileLength >= mask) {
      Copy(dstTensor, srcTensor, mask, this->outputTileLength / mask,
           {1, 1, 8, 8});
      if (this->outputTileLength % mask != 0) {
        uint32_t offset = this->outputTileLength - mask;
        Copy(dstTensor[offset], srcTensor[offset], mask, 1, {1, 1, 8, 8});
      }

    } else {
      Copy(dstTensor, srcTensor, this->outputTileLength, 1, {1, 1, 8, 8});
    }
  }

  __aicore__ inline void GenerateInitIndex(
      const LocalTensor<uint32_t> &initTensor) {
    uint32_t zero(0);
    Duplicate(initTensor, zero, this->lastDim * 3);
    for (uint32_t i = 0; i < this->lastDim; ++i) {
      initTensor.SetValue(i, i);
    }
    int n = this->k;
    int indices_sum = 0;
    int score_sum = 0;
    while (n) {
      indices_sum += (1 << (2 * n - 1));
      score_sum += (1 << (2 * n - 2));
      n--;
    }
    initTensor.SetValue(this->lastDim, score_sum);
    initTensor.SetValue(this->lastDim * 2, indices_sum);
  }

 private:
  uint32_t totalLength;
  uint32_t lastDim;
  uint32_t tileNum;
  uint32_t blockLength;
  uint32_t tileLength;

  uint32_t outputLength;
  uint32_t outputTileLength;

  uint32_t k;

  GlobalTensor<float> xGm;
  GlobalTensor<float> yGm;
  GlobalTensor<float> indicesGm;

  TPipe pipe;
  TQue<QuePosition::VECIN, 1> inQueueX;
  TQue<QuePosition::VECOUT, 1> outQueueY, outQueueIndices;
  TBuf<QuePosition::VECCALC> reduceRes, workLocal;
  TBuf<QuePosition::VECCALC> topKRes;
  TBuf<QuePosition::VECCALC> topKIndices;
};

extern "C" __global__ __aicore__ void moe_soft_max_topk(GM_ADDR x, GM_ADDR y,
                                                        GM_ADDR indices,
                                                        uint32_t k) {
  KernelSoftmaxTopK op;
  op.Init(x, y, indices, k);
  op.Process();
}

#ifndef __CCE_KT_TEST__
// call of kernel function
void moe_soft_max_topk_do(uint32_t blockDim, void *l2ctrl, void *stream,
                          uint8_t *x, uint8_t *y, uint8_t *indices,
                          uint32_t k) {
  moe_soft_max_topk<<<blockDim, l2ctrl, stream>>>(x, y, indices, k);
}
#endif