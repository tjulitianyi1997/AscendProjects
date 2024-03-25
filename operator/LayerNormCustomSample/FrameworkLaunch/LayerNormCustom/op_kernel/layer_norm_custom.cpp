#include "kernel_operator.h"
using namespace AscendC;
constexpr int32_t BUFFER_NUM = 1;

class KernelLayerNorm {
 public:
  __aicore__ inline KernelLayerNorm() {}
  __aicore__ inline void InitTiling(GM_ADDR tiling) {
    GET_TILING_DATA(tiling_data, tiling);
    rowNum = tiling_data.rowNum;
    rowNumSp = tiling_data.rowNumSp;
    rowLength = tiling_data.rowLength;
    blockPivot = tiling_data.blockPivot;
    tileLoop = tiling_data.tileLoop;
    tileLength = tiling_data.tileLength;
    loopCount = tiling_data.loopCount;
    factor = tiling_data.factor;
    mfactor = tiling_data.mfactor;
    eps = tiling_data.eps;
  }
  __aicore__ inline void Init(GM_ADDR x, GM_ADDR gamma, GM_ADDR beta, GM_ADDR z,
                              GM_ADDR tiling) {
    InitTiling(tiling);

    ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");

    this->leftRow = this->rowNum % this->tileLoop;
    if (GetBlockIdx() < this->blockPivot) {
      this->rowNum = this->rowNumSp;
      this->leftRow += 1;
    }

    this->blockLength = this->rowNum * this->rowLength;
    uint32_t offset = 0;
    if (GetBlockIdx() < this->blockPivot) {
      offset = this->blockLength * GetBlockIdx();
    } else {
      offset = this->blockLength * GetBlockIdx() +
               this->rowLength * this->blockPivot;
    }

    xGm.SetGlobalBuffer((__gm__ float *)x + offset, this->blockLength);
    zGm.SetGlobalBuffer((__gm__ float *)z + offset, this->blockLength);

    gammaGm.SetGlobalBuffer((__gm__ float *)gamma, this->rowLength);
    betaGm.SetGlobalBuffer((__gm__ float *)beta, this->rowLength);

    pipe.InitBuffer(queueX, BUFFER_NUM, this->tileLength * sizeof(float));
    pipe.InitBuffer(queueZ, BUFFER_NUM, this->tileLength * sizeof(float));

    pipe.InitBuffer(tmpBuffer1, 64 * sizeof(float));
    pipe.InitBuffer(tmpBuffer2, 64 * sizeof(float));
    pipe.InitBuffer(onesBuffer, 64 * sizeof(float));

    pipe.InitBuffer(queueGamma, 1, this->rowLength * sizeof(float));
    pipe.InitBuffer(queueBeta, 1, this->rowLength * sizeof(float));
  }
  __aicore__ inline void Process() {
    for (int32_t i = 0; i < this->loopCount; i++) {
      CopyIn(i, this->tileLoop);
      Compute(i, this->tileLoop);
      CopyOut(i, this->tileLoop);
    }
    if (this->leftRow > 0) {
      CopyIn(this->loopCount, this->leftRow);
      Compute(this->loopCount, this->leftRow);
      CopyOut(this->loopCount, this->leftRow);
    }
  }

 private:
  __aicore__ inline void CopyIn(int32_t progress, int32_t rowNum) {
    LocalTensor<float> xLocal = queueX.AllocTensor<float>();
    LocalTensor<float> gammaLocal = queueGamma.AllocTensor<float>();
    LocalTensor<float> betaLocal = queueBeta.AllocTensor<float>();
    DataCopy(xLocal, xGm[progress * this->tileLength],
             this->rowLength * rowNum);
    DataCopy(gammaLocal, gammaGm[0], this->rowLength);
    DataCopy(betaLocal, betaGm[0], this->rowLength);
    queueX.EnQue(xLocal);
    queueGamma.EnQue(gammaLocal);
    queueBeta.EnQue(betaLocal);
  }

  __aicore__ inline void Compute(int32_t progress, int32_t rowNum) {
    LocalTensor<float> xLocal = queueX.DeQue<float>();
    LocalTensor<float> gammaLocal = queueGamma.DeQue<float>();
    LocalTensor<float> betaLocal = queueBeta.DeQue<float>();

    LocalTensor<float> tmpTensor1 = tmpBuffer1.Get<float>();
    LocalTensor<float> tmpTensor2 = tmpBuffer2.Get<float>();
    LocalTensor<float> onesLocal = onesBuffer.Get<float>();
    LocalTensor<float> zLocal = queueZ.AllocTensor<float>();
    Duplicate<float>(onesLocal, 1.0f, this->tileLoop);

    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      ReduceSum<float>(tmpTensor2[j], xLocal[buffIndex], tmpTensor1,
                       this->rowLength);
    }

    Muls(zLocal, tmpTensor2, this->mfactor, rowNum);

    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      Adds(xLocal[buffIndex], xLocal[buffIndex], zLocal.GetValue(j),
           this->rowLength);
    }

    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      Mul(zLocal[buffIndex], xLocal[buffIndex], xLocal[buffIndex],
          this->rowLength);
    }
    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      ReduceSum<float>(tmpTensor2[j], zLocal[buffIndex], tmpTensor1,
                       this->rowLength);
    }
    Muls(tmpTensor2, tmpTensor2, this->factor, rowNum);
    Adds(tmpTensor2, tmpTensor2, this->eps, rowNum);
    Sqrt(tmpTensor2, tmpTensor2, rowNum);
    Div(tmpTensor2, onesLocal, tmpTensor2, rowNum);

    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      Muls(zLocal[buffIndex], xLocal[buffIndex], tmpTensor2.GetValue(j),
           this->rowLength);
    }

    for (uint32_t j = 0; j < rowNum; ++j) {
      uint32_t buffIndex = j * this->rowLength;
      Mul(zLocal[buffIndex], zLocal[buffIndex], gammaLocal, this->rowLength);
      Add(zLocal[buffIndex], zLocal[buffIndex], betaLocal, this->rowLength);
    }

    queueZ.EnQue<float>(zLocal);
    queueGamma.FreeTensor(gammaLocal);
    queueBeta.FreeTensor(betaLocal);
    queueX.FreeTensor(xLocal);
  }

  __aicore__ inline void CopyOut(int32_t progress, int32_t rowNum) {
    LocalTensor<float> zLocal = queueZ.DeQue<float>();

    DataCopy(zGm[progress * this->tileLength], zLocal,
             rowNum * this->rowLength);

    queueZ.FreeTensor(zLocal);
  }

 private:
  TPipe pipe;
  TBuf<QuePosition::VECCALC> tmpBuffer1, tmpBuffer2, onesBuffer;
  TQue<QuePosition::VECIN, BUFFER_NUM> queueX;
  TQue<QuePosition::VECIN, 1> queueGamma, queueBeta;
  TQue<QuePosition::VECOUT, BUFFER_NUM> queueZ;
  GlobalTensor<float> xGm;
  GlobalTensor<float> gammaGm;
  GlobalTensor<float> betaGm;
  GlobalTensor<float> zGm;

  uint32_t blockLength = 0;
  uint32_t leftRow = 0;
  uint32_t rowNum = 341;
  uint32_t rowNumSp = 342;
  uint32_t rowLength = 1024;
  uint32_t blockPivot = 16;
  uint32_t tileLoop = 8;
  uint32_t tileLength = 8 * 1024;
  uint32_t loopCount = 42;
  float factor = 0.0009765625;
  float mfactor = -0.0009765625;
  float eps = 1e-5;
};
extern "C" __global__ __aicore__ void layer_norm_custom(
    GM_ADDR x, GM_ADDR gamma, GM_ADDR beta, GM_ADDR res_out, GM_ADDR workspace,
    GM_ADDR tiling) {
  KernelLayerNorm op;
  op.Init(x, gamma, beta, res_out, tiling);
  if (TILING_KEY_IS(1)) {
    op.Process();
  }
}