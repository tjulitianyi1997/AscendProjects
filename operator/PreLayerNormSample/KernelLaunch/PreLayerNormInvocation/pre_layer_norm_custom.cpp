#include "kernel_operator.h"
using namespace AscendC;

constexpr int32_t TOTAL_LENGTH = 4980 * 4 * 2048;
constexpr int32_t ONCE_CAL_NUM = 2048;
constexpr int32_t TILE_NUM = 415;
constexpr float epsilon = 1e-5f;
constexpr int32_t BUFFER_NUM = 1;

class KernelLayerNorm
{
public:
    __aicore__ inline KernelLayerNorm() {}
    __aicore__ inline void Init(GM_ADDR x, GM_ADDR y, GM_ADDR gamma, GM_ADDR beta, GM_ADDR z)
    {
        gemmaGm.SetGlobalBuffer((__gm__ float *)gamma);
        betaGm.SetGlobalBuffer((__gm__ float *)beta);
        xGm.SetGlobalBuffer((__gm__ float *)x + TILE_NUM * ONCE_CAL_NUM * GetBlockIdx(), TILE_NUM * ONCE_CAL_NUM);
        yGm.SetGlobalBuffer((__gm__ float *)y + TILE_NUM * ONCE_CAL_NUM * GetBlockIdx(), TILE_NUM * ONCE_CAL_NUM);
        zGm.SetGlobalBuffer((__gm__ float *)z + TILE_NUM * ONCE_CAL_NUM * GetBlockIdx(), TILE_NUM * ONCE_CAL_NUM);
        pipe.InitBuffer(queueX, BUFFER_NUM, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(queueY, BUFFER_NUM, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(queueGamma, 1, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(queueBeta, 1, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(queueZ, BUFFER_NUM, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(tempBuf1, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(tempBuf2, ONCE_CAL_NUM * sizeof(float));
        pipe.InitBuffer(tempBuf3, ONCE_CAL_NUM * sizeof(float));
    }
    __aicore__ inline void Process()
    {
        for (int32_t i = 0; i < TILE_NUM; i++)
        {
            CopyIn(i);
            Compute(i);
            CopyOut(i);
        }
    }

private:
    __aicore__ inline void CopyIn(int32_t progress)
    {
        LocalTensor<float> xLocal = queueX.AllocTensor<float>();
        LocalTensor<float> yLocal = queueY.AllocTensor<float>();
        LocalTensor<float> gammaLocal = queueGamma.AllocTensor<float>();
        LocalTensor<float> betaLocal = queueBeta.AllocTensor<float>();
        DataCopy(xLocal, xGm[progress * ONCE_CAL_NUM], ONCE_CAL_NUM);
        DataCopy(yLocal, yGm[progress * ONCE_CAL_NUM], ONCE_CAL_NUM);
        DataCopy(gammaLocal, gemmaGm, ONCE_CAL_NUM);
        DataCopy(betaLocal, betaGm, ONCE_CAL_NUM);
        queueX.EnQue(xLocal);
        queueY.EnQue(yLocal);
        queueGamma.EnQue(gammaLocal);
        queueBeta.EnQue(betaLocal);
    }

    __aicore__ inline void Compute(int32_t progress)
    {
        LocalTensor<float> xLocal = queueX.DeQue<float>();
        LocalTensor<float> yLocal = queueY.DeQue<float>();
        LocalTensor<float> gammaLocal = queueGamma.DeQue<float>();
        LocalTensor<float> betaLocal = queueBeta.DeQue<float>();
        LocalTensor<float> zLocal = queueZ.AllocTensor<float>();

        LocalTensor<float> tempOneLocal = tempBuf1.Get<float>();
        LocalTensor<float> tempTwoLocal = tempBuf2.Get<float>();
        LocalTensor<float> tempThreeLocal = tempBuf3.Get<float>();
        Add(xLocal, xLocal, yLocal, ONCE_CAL_NUM);
        ReduceSum<float>(tempOneLocal, xLocal, tempTwoLocal, ONCE_CAL_NUM);
        Muls(tempOneLocal, tempOneLocal, 1.0f / ONCE_CAL_NUM, 1);
        event_t eventIdVToS = static_cast<event_t>(GetTPipePtr()->FetchEventID(HardEvent::V_S));
        set_flag(PIPE_V, PIPE_S, eventIdVToS);
        wait_flag(PIPE_V, PIPE_S, eventIdVToS);

        float average = tempOneLocal.GetValue(0) * -1.0f;
        event_t eventIdSToV = static_cast<event_t>(GetTPipePtr()->FetchEventID(HardEvent::S_V));
        set_flag(PIPE_S, PIPE_V, eventIdSToV);
        wait_flag(PIPE_S, PIPE_V, eventIdSToV);
        Adds(xLocal, xLocal, average, ONCE_CAL_NUM);
        tempOneLocal = xLocal * xLocal;
        ReduceSum<float>(tempTwoLocal, tempOneLocal, tempThreeLocal, ONCE_CAL_NUM);

        Muls(tempTwoLocal, tempTwoLocal, 1.0f / ONCE_CAL_NUM, 1);
        Adds(tempTwoLocal, tempTwoLocal, epsilon, 1);
        Sqrt(tempTwoLocal, tempTwoLocal, 1);
        tempOneLocal = xLocal * gammaLocal;
        eventIdVToS = static_cast<event_t>(GetTPipePtr()->FetchEventID(HardEvent::V_S));
        set_flag(PIPE_V, PIPE_S, eventIdVToS);
        wait_flag(PIPE_V, PIPE_S, eventIdVToS);

        float temp = 1.0f / tempTwoLocal.GetValue(0);
        eventIdSToV = static_cast<event_t>(GetTPipePtr()->FetchEventID(HardEvent::S_V));
        set_flag(PIPE_S, PIPE_V, eventIdSToV);
        wait_flag(PIPE_S, PIPE_V, eventIdSToV);

        Muls(tempOneLocal, tempOneLocal, temp, ONCE_CAL_NUM);
        zLocal = tempOneLocal + betaLocal;
        queueZ.EnQue<float>(zLocal);
        queueGamma.FreeTensor(gammaLocal);
        queueBeta.FreeTensor(betaLocal);

        queueY.FreeTensor(yLocal);
        queueX.FreeTensor(xLocal);
    }

    __aicore__ inline void CopyOut(int32_t progress)
    {
        LocalTensor<float> zLocal = queueZ.DeQue<float>();
        DataCopy(zGm[progress * ONCE_CAL_NUM], zLocal, ONCE_CAL_NUM);
        queueZ.FreeTensor(zLocal);
    }

private:
    TPipe pipe;
    TQue<QuePosition::VECIN, BUFFER_NUM> queueX, queueY, queueGamma, queueBeta;
    TQue<QuePosition::VECOUT, BUFFER_NUM> queueZ;
    GlobalTensor<float> xGm;
    GlobalTensor<float> yGm;
    GlobalTensor<float> gemmaGm;
    GlobalTensor<float> betaGm;
    GlobalTensor<float> zGm;
    TBuf<> tempBuf1;
    TBuf<> tempBuf2;
    TBuf<> tempBuf3;
};

extern "C" __global__ __aicore__ void pre_layer_norm_custom(GM_ADDR x, GM_ADDR y, GM_ADDR gamma, GM_ADDR beta, GM_ADDR z)
{
    KernelLayerNorm op;
    op.Init(x, y, gamma, beta, z);
    op.Process();
}

#ifndef __CCE_KT_TEST__
void pre_layer_norm_custom_do(uint32_t blockDim, void *l2ctrl, void *stream, uint8_t *x, uint8_t *y, uint8_t *gamma, uint8_t *beta, uint8_t *z)
{
    pre_layer_norm_custom<<<blockDim, l2ctrl, stream>>>(x, y, gamma, beta, z);
}
#endif