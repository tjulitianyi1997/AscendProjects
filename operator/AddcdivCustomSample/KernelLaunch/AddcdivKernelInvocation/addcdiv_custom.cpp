/*
 *
 *
 * Function :
 *
 */
#include "kernel_operator.h"
using namespace AscendC;

// constexpr int32_t TOTAL_LENGTH_IN = 8 * 2048;// 8 * 2048 // total length of
// data
constexpr int32_t TOTAL_LENGTH = 8 * 2048;
constexpr int32_t USE_CORE_NUM = 8;  // num of core used
constexpr int32_t BLOCK_LENGTH =
    TOTAL_LENGTH / USE_CORE_NUM;   // length computed of each core
constexpr int32_t TILE_NUM = 16;   // split data into 8 tiles for each core
constexpr int32_t BUFFER_NUM = 1;  // tensor num for each queue
constexpr int32_t TILE_LENGTH =
    BLOCK_LENGTH / TILE_NUM /
    BUFFER_NUM;  // seperate to 2 parts, due to double buffer

class KernelAddcdiv {
 public:
  __aicore__ inline KernelAddcdiv() {}
  __aicore__ inline void Init(GM_ADDR x, GM_ADDR y, GM_ADDR z, GM_ADDR out) {
    this->blockLength = BLOCK_LENGTH;
    this->tileNum = TILE_NUM;
    ASSERT(tileNum != 0 && "tile num can not be zero!");
    this->tileLength = TILE_LENGTH;
    this->value = (half)1.0;  // 与gen_data.py内value保存一致

    xGm.SetGlobalBuffer((__gm__ half*)x + this->blockLength * GetBlockIdx(),
                        this->blockLength);
    yGm.SetGlobalBuffer((__gm__ half*)y + this->blockLength * GetBlockIdx(),
                        this->blockLength);
    zGm.SetGlobalBuffer((__gm__ half*)z + this->blockLength * GetBlockIdx(),
                        this->blockLength);
    outGm.SetGlobalBuffer((__gm__ half*)out + this->blockLength * GetBlockIdx(),
                          this->blockLength);
    pipe.InitBuffer(inQueueX, BUFFER_NUM, this->tileLength * sizeof(half));
    pipe.InitBuffer(inQueueY, BUFFER_NUM, this->tileLength * sizeof(half));
    pipe.InitBuffer(inQueueZ, BUFFER_NUM, this->tileLength * sizeof(half));
    pipe.InitBuffer(outQueueOUT, BUFFER_NUM, this->tileLength * sizeof(half));
  }
  __aicore__ inline void Process() {
    int32_t loopCount = this->tileNum * BUFFER_NUM;
    for (int32_t i = 0; i < loopCount; i++) {
      CopyIn(i);
      Compute(i);
      CopyOut(i);
    }
  }

 private:
  __aicore__ inline void CopyIn(int32_t progress) {
    LocalTensor<half> xLocal = inQueueX.AllocTensor<half>();
    LocalTensor<half> yLocal = inQueueY.AllocTensor<half>();
    LocalTensor<half> zLocal = inQueueZ.AllocTensor<half>();
    DataCopy(xLocal, xGm[progress * this->tileLength], this->tileLength);
    DataCopy(yLocal, yGm[progress * this->tileLength], this->tileLength);
    DataCopy(zLocal, zGm[progress * this->tileLength], this->tileLength);
    inQueueX.EnQue(xLocal);
    inQueueY.EnQue(yLocal);
    inQueueZ.EnQue(zLocal);
  }
  __aicore__ inline void Compute(int32_t progress) {
    LocalTensor<half> xLocal = inQueueX.DeQue<half>();
    LocalTensor<half> yLocal = inQueueY.DeQue<half>();
    LocalTensor<half> zLocal = inQueueZ.DeQue<half>();
    LocalTensor<half> outLocal = outQueueOUT.AllocTensor<half>();
    // compute
    Div(outLocal, yLocal, zLocal, this->tileLength);
    Muls(outLocal, outLocal, this->value, this->tileLength);
    Add(outLocal, xLocal, outLocal, this->tileLength);
    outQueueOUT.EnQue<half>(outLocal);
    inQueueX.FreeTensor(xLocal);
    inQueueY.FreeTensor(yLocal);
    inQueueZ.FreeTensor(zLocal);
  }
  __aicore__ inline void CopyOut(int32_t progress) {
    LocalTensor<half> outLocal = outQueueOUT.DeQue<half>();
    DataCopy(outGm[progress * this->tileLength], outLocal, this->tileLength);
    outQueueOUT.FreeTensor(outLocal);
  }

 private:
  TPipe pipe;
  TQue<QuePosition::VECIN, BUFFER_NUM> inQueueX, inQueueY, inQueueZ;
  TQue<QuePosition::VECOUT, BUFFER_NUM> outQueueOUT;
  GlobalTensor<half> xGm;
  GlobalTensor<half> yGm;
  GlobalTensor<half> zGm;
  GlobalTensor<half> outGm;
  half value;
  uint32_t blockLength;
  uint32_t tileNum;
  uint32_t tileLength;
};

extern "C" __global__ __aicore__ void addcdiv_custom(GM_ADDR x, GM_ADDR y,
                                                     GM_ADDR z, GM_ADDR out) {
  KernelAddcdiv op;
  op.Init(x, y, z, out);
  op.Process();
}

#ifndef __CCE_KT_TEST__
// call of kernel function
void addcdiv_custom_do(uint32_t blockDim, void* l2ctrl, void* stream,
                       uint8_t* x, uint8_t* y, uint8_t* z, uint8_t* out) {
  addcdiv_custom<<<blockDim, l2ctrl, stream>>>(x, y, z, out);
}
#endif