#include "kernel_operator.h"
using namespace AscendC;

constexpr int32_t BUFFER_NUM = 2;

class KernelLayerNorm {
 public:
  __aicore__ inline KernelLayerNorm() {}
  __aicore__ inline void Init(GM_ADDR x, GM_ADDR gamma, GM_ADDR beta,
                              GM_ADDR z) {
    ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");
    if (GetBlockIdx() < this->blockPivot) {
      this->rowNum = this->rowNumSp;
    }
    this->leftRow = this->rowNum % this->tileLoop;
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

    pipe.InitBuffer(queueGamma, 1, this->rowLength * sizeof(float));
    pipe.InitBuffer(queueBeta, 1, this->rowLength * sizeof(float));

    pipe.InitBuffer(tmpBuffer1, this->tileLength * sizeof(float));
    pipe.InitBuffer(tmpBuffer2, this->tileLength * sizeof(float));
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
    LocalTensor<float> zLocal = queueZ.AllocTensor<float>();

    for (size_t j = 0; j < rowNum; j++) {
      LocalTensor<float> xLocalj = xLocal[j * this->rowLength];
      LocalTensor<float> zLocalj = zLocal[j * this->rowLength];
      LocalTensor<float> tmpTensor1j = tmpTensor1[j * this->rowLength];
      LocalTensor<float> tmpTensor2j = tmpTensor2[j * this->rowLength];

      ReduceSum<float>(tmpTensor2j, xLocalj, tmpTensor1j, this->rowLength);
      Muls(tmpTensor1j, tmpTensor2j, this->mfactor, 1);
      Adds(tmpTensor2j, xLocalj, tmpTensor1j.GetValue(0), this->rowLength);
      Mul(xLocalj, tmpTensor2j, tmpTensor2j, this->rowLength);
      Muls(tmpTensor1j, xLocalj, this->factor, this->rowLength);
      ReduceSum<float>(xLocalj, tmpTensor1j, zLocalj, this->rowLength);
      Adds(tmpTensor1j, xLocalj, this->eps, 1);
      Ln(zLocalj, tmpTensor1j, 1);
      Muls(tmpTensor1j, zLocalj, -0.5f, 1);
      Exp(xLocalj, tmpTensor1j, 1);
      Muls(tmpTensor1j, tmpTensor2j, xLocalj.GetValue(0), this->rowLength);

      Mul(tmpTensor2j, tmpTensor1j, gammaLocal, this->rowLength);
      Add(zLocalj, tmpTensor2j, betaLocal, this->rowLength);
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
  TBuf<QuePosition::VECCALC> tmpBuffer1, tmpBuffer2;
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
  uint32_t tileLoop = 5;
  uint32_t tileLength = 5 * 1024;
  uint32_t loopCount = 68;
  float factor = 0.0009765625;
  float mfactor = -0.0009765625;
  float eps = 1e-5;
};

extern "C" __global__ __aicore__ void layer_norm_custom(GM_ADDR x,
                                                        GM_ADDR gamma,
                                                        GM_ADDR beta,
                                                        GM_ADDR res_out) {
  KernelLayerNorm op;
  op.Init(x, gamma, beta, res_out);
  op.Process();
}

#ifndef __CCE_KT_TEST__
// call of kernel function
void layer_norm_custom_do(uint32_t blockDim, void *l2ctrl, void *stream,
                          uint8_t *x, uint8_t *gamma, uint8_t *beta,
                          uint8_t *res_out) {
  layer_norm_custom<<<blockDim, l2ctrl, stream>>>(x, gamma, beta, res_out);
}
#endif