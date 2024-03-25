#include "kernel_operator.h"

using namespace AscendC;
constexpr uint32_t REDUCE_SUM_ONE_REPEAT = 256;
constexpr uint32_t EACH_BLOCK_SIZE = 32;
constexpr uint32_t BUFFER_NUM = 1;

class KernelSoftmaxTopK {
 public:
  __aicore__ inline KernelSoftmaxTopK() {}
  __aicore__ inline void Init(GM_ADDR x, GM_ADDR y, GM_ADDR indices,
                              uint32_t totalLength, uint32_t lastDim,
                              uint32_t tileNum, uint32_t k, int32_t scoreSum,
                              int32_t indicesSum) {
    ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");
    this->totalLength = totalLength;
    this->lastDim = lastDim;
    this->tileNum = tileNum;
    this->k = k;
    this->blockLength = this->totalLength / GetBlockNum();
    this->outputLength =
        this->totalLength / this->lastDim / GetBlockNum() * this->k;
    ASSERT(tileNum != 0 && "tileNum can not be zero!");
    this->tileLength = this->blockLength / this->tileNum / BUFFER_NUM;
    this->outputTileLength = this->outputLength / this->tileNum / BUFFER_NUM;

    this->blockReduceStride = this->lastDim * 4 / EACH_BLOCK_SIZE;
    this->normalMask = REDUCE_SUM_ONE_REPEAT / 4;
    this->topKRepeatTimes = this->tileLength / this->lastDim;
    this->padTilingLength = this->tileLength * (32 / this->lastDim);

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

    pipe.InitBuffer(reduceRes, topKRepeatTimes * sizeof(float));
    pipe.InitBuffer(topKRes, this->padTilingLength * 2 * sizeof(float));
    pipe.InitBuffer(workLocal, this->padTilingLength * 2 * sizeof(float));
    pipe.InitBuffer(topKIndices, this->lastDim * 3 * sizeof(uint32_t));

    LocalTensor<uint32_t> indicesInitLocal = topKIndices.Get<uint32_t>();
    GenerateInitIndex(indicesInitLocal, scoreSum, indicesSum);
  }
  __aicore__ inline void Process() {
    if (GetBlockIdx() >= GetBlockNum()) {
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
    LocalTensor<float> reduceTensor = reduceRes.Get<float>();
    DoSoftMax(xLocal, reduceTensor, progress);
    DoTopK(xLocal, yLocal, indicesLocal);

    outQueueY.EnQue(yLocal);
    outQueueIndices.EnQue(indicesLocal);
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
                                   const LocalTensor<T> &reduceTensor,
                                   int32_t progress) {
    Exp(srcLocalTensor, srcLocalTensor, this->tileLength);
    DoReduceSum(srcLocalTensor, reduceTensor);
    //  LocalTensor<float> reduceBroadcastTensor = workLocal.Get<float>();
    for (uint32_t i = 0; i < this->topKRepeatTimes; i++) {
      uint32_t offset = i * this->lastDim;
      Muls(srcLocalTensor[offset], srcLocalTensor[offset],
           1 / reduceTensor.GetValue(i), this->lastDim);
    }
  }

  template <typename T>
  __aicore__ inline void DoReduceSum(const LocalTensor<T> &srcLocalTensor,
                                     const LocalTensor<T> &reduceTensor) {
    uint32_t repeatTimes = topKRepeatTimes;
    if (this->lastDim >= this->normalMask) {
      const uint32_t dstRepStride = 1;
      const uint32_t srcBlkStride = 1;
      uint32_t srcRepStride = this->normalMask * sizeof(T) / EACH_BLOCK_SIZE;
      WholeReduceSum(reduceTensor, srcLocalTensor, this->normalMask,
                     repeatTimes, dstRepStride, srcBlkStride, srcRepStride);
    } else {
      if (this->blockReduceStride == 0) {
        for (uint32_t i = 0; i < repeatTimes; ++i) {
          reduceTensor.SetValue(i, srcLocalTensor.GetValue(i * this->lastDim));
          for (uint32_t j = 1; j < this->lastDim; ++j) {
            reduceTensor.SetValue(
                i, reduceTensor.GetValue(i) +
                       srcLocalTensor.GetValue(i * this->lastDim + j));
          }
        }
      } else {
        uint64_t mask = this->lastDim;
        const uint32_t dstRepStride = 1;
        const uint32_t srcBlkStride = 1;
        uint64_t srcRepStride = this->blockReduceStride;
        WholeReduceSum(reduceTensor, srcLocalTensor, mask, repeatTimes,
                       dstRepStride, srcBlkStride, srcRepStride);
      }
    }
  }

  __aicore__ inline void DoTopK(const LocalTensor<float> &srcLocalTensor,
                                const LocalTensor<float> &yLocalTensor,
                                const LocalTensor<float> &indicesLocalTensor) {
    LocalTensor<uint32_t> indicesInitLocal = topKIndices.Get<uint32_t>();
    LocalTensor<uint32_t> topkInitTensor = indicesInitLocal;
    LocalTensor<uint32_t> gatherScoreTensor = indicesInitLocal[this->lastDim];
    LocalTensor<uint32_t> gatherIndicesTensor =
        indicesInitLocal[this->lastDim * 2];

    LocalTensor<float> topKResTensor = topKRes.Get<float>();
    LocalTensor<float> workLocalTensor = workLocal.Get<float>();
    float ZERO(0);
    if (this->lastDim < 32) {
      Duplicate(workLocalTensor, ZERO, this->padTilingLength);
      uint16_t repeat = this->lastDim / 8;
      Copy(workLocalTensor, srcLocalTensor, this->lastDim,
           this->topKRepeatTimes, {1, 1, 4, repeat});
      DoSort32(topKResTensor, workLocalTensor, topkInitTensor);
    } else {
      DoSort32(topKResTensor, srcLocalTensor, topkInitTensor);
    }
    if (this->lastDim < 32) {
      Duplicate(workLocalTensor, ZERO, this->padTilingLength);
      DoGatherMask(workLocalTensor, topKResTensor, gatherScoreTensor);
      DoCopyToRes(yLocalTensor, workLocalTensor);

      Duplicate(workLocalTensor, ZERO, this->padTilingLength);
      DoGatherMask(workLocalTensor, topKResTensor, gatherIndicesTensor);
      DoCopyToRes(indicesLocalTensor, workLocalTensor);
    } else {
      DoGatherMask(workLocalTensor, topKResTensor, gatherScoreTensor);
      DoCopyToRes(yLocalTensor, workLocalTensor);

      Duplicate(workLocalTensor, ZERO, this->padTilingLength);
      DoGatherMask(workLocalTensor, topKResTensor, gatherIndicesTensor);
      DoCopyToRes(indicesLocalTensor, workLocalTensor);
    }
  }

  template <typename T>
  __aicore__ inline void DoSort32(const LocalTensor<T> &topKResTensor,
                                  const LocalTensor<T> &srcLocalTensor,
                                  const LocalTensor<uint32_t> &indicesTensor) {
    for (uint32_t i = 0; i < this->topKRepeatTimes; ++i) {
      Sort32(topKResTensor[i * 64], srcLocalTensor[i * 32], indicesTensor, 1);
    }
  }

  template <typename T>
  __aicore__ inline void DoGatherMask(
      const LocalTensor<T> &workLocalTensor,
      const LocalTensor<T> &topKResTensor,
      const LocalTensor<uint32_t> &gatherMaskTensor) {
    uint8_t src0BlockStride = 1;
    uint16_t src0RepeatStride = 8;
    uint8_t src1RepeatStride = 0;
    uint64_t rsvdCnt = 0;
    GatherMaskParams gatherMaskParams = {src0BlockStride, this->topKRepeatTimes,
                                         src0RepeatStride, src1RepeatStride};
    GatherMask(workLocalTensor, topKResTensor, gatherMaskTensor, true,
               this->normalMask, gatherMaskParams, rsvdCnt);
  }

  template <typename T>
  __aicore__ inline void DoCopyToRes(const LocalTensor<T> &dstTensor,
                                     const LocalTensor<T> &srcTensor) {
    if (this->outputTileLength >= this->normalMask) {
      Copy(dstTensor, srcTensor, this->normalMask,
           this->outputTileLength / this->normalMask, {1, 1, 8, 8});
      if (this->outputTileLength % this->normalMask != 0) {
        uint32_t offset = this->outputTileLength - this->normalMask;
        Copy(dstTensor[offset], srcTensor[offset], this->normalMask, 1,
             {1, 1, 8, 8});
      }
    } else {
      Copy(dstTensor, srcTensor, this->outputTileLength, 1, {1, 1, 8, 8});
    }
  }

  __aicore__ inline void GenerateInitIndex(
      const LocalTensor<uint32_t> &initTensor, int32_t scoreSum,
      int32_t indicesSum) {
    uint32_t zero(0);
    Duplicate(initTensor, zero, this->lastDim * 3);
    for (uint32_t i = 0; i < this->lastDim; ++i) {
      initTensor.SetValue(i, i);
    }
    initTensor.SetValue(this->lastDim, scoreSum);
    initTensor.SetValue(this->lastDim * 2, indicesSum);
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
  uint32_t blockReduceStride;
  uint32_t normalMask;
  uint16_t topKRepeatTimes;
  uint32_t padTilingLength;

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
                                                        GM_ADDR workspace,
                                                        GM_ADDR tiling) {
  GET_TILING_DATA(tiling_data, tiling);
  KernelSoftmaxTopK op;
  op.Init(x, y, indices, tiling_data.totalLength, tiling_data.lastDim,
          tiling_data.tileNum, tiling_data.k, tiling_data.scoreSum,
          tiling_data.indicesSum);
  if (TILING_KEY_IS(1)) {
    op.Process();
  }
}