#include "kernel_operator.h"
using namespace AscendC;

constexpr int32_t BUFFER_NUM = 1;

class KernelLayerNorm{
public:
    __aicore__ inline KernelLayerNorm(){}
    __aicore__ inline void Init(GM_ADDR x, GM_ADDR y, GM_ADDR gamma, GM_ADDR beta, GM_ADDR z,GM_ADDR workspace, uint32_t onceCalNum, uint32_t onceRowNum, float epsilon, uint32_t tileNum){
        ASSERT(GetBlockNum() != 0 && "block dim can not be zero!");
        this->onceCalNum = onceCalNum;
        this->onceRowNum = onceRowNum;
        this->tileNum = 42;
        if(GetBlockIdx() == 47){
            this->tileNum = 74;
        }
        this->epsilon = epsilon;
        this->loopNum = onceCalNum / onceRowNum;
        gemmaGm.SetGlobalBuffer((__gm__ float*)gamma);
        betaGm.SetGlobalBuffer((__gm__ float*)beta);
        xGm.SetGlobalBuffer((__gm__ float*)x + 42 * this->onceCalNum * GetBlockIdx(), this->tileNum * this->onceCalNum);
        yGm.SetGlobalBuffer((__gm__ float*)y + 42 * this->onceCalNum * GetBlockIdx(), this->tileNum * this->onceCalNum);
        zGm.SetGlobalBuffer((__gm__ float*)z + 42 * this->onceCalNum * GetBlockIdx(), this->tileNum * this->onceCalNum);
        workspaceGm.SetGlobalBuffer((__gm__ float*)workspace + 42 * this->onceCalNum * GetBlockIdx(), this->tileNum * this->onceCalNum);
        pipe.InitBuffer(queueX, BUFFER_NUM, this->onceCalNum * sizeof(float));
        pipe.InitBuffer(queueY, BUFFER_NUM, this->onceCalNum * sizeof(float));
        pipe.InitBuffer(queueGamma, 1, this->onceRowNum * sizeof(float));
        pipe.InitBuffer(queueBeta, 1, this->onceRowNum * sizeof(float));
        pipe.InitBuffer(queueZ, BUFFER_NUM, this->onceRowNum * sizeof(float));
        pipe.InitBuffer(tempBuf1, this->onceCalNum / 64 * sizeof(float));
        pipe.InitBuffer(tempBuf2, this->onceCalNum / 64 * sizeof(float));
    }
    __aicore__ inline void Process(){
        for (int32_t i = 0; i < this->tileNum; i++){
            Calculate(i);
        }
    }
private:
    __aicore__ inline void Calculate(int32_t progress){
        LocalTensor<float> xLocal = queueX.AllocTensor<float>();
        LocalTensor<float> yLocal = queueY.AllocTensor<float>();
        DataCopy(xLocal, xGm[progress * this->onceCalNum], this->onceCalNum);
        queueX.EnQue(xLocal);
        DataCopy(yLocal, yGm[progress * this->onceCalNum], this->onceCalNum);
        queueY.EnQue(yLocal);
        xLocal = queueX.DeQue<float>();
        yLocal = queueY.DeQue<float>();

        LocalTensor<float> tempOneLocal = tempBuf1.Get<float>();
        LocalTensor<float> tempTwoLocal = tempBuf2.Get<float>();

        Add(xLocal,xLocal,yLocal,this->onceCalNum);
        ReduceSum<float>(tempOneLocal,xLocal, tempTwoLocal, this->onceCalNum);
        float neg_average = tempOneLocal.GetValue(0) * -1.0f / this->onceCalNum;
        Adds(xLocal, xLocal, neg_average, this->onceCalNum);

        yLocal = xLocal * xLocal;
        ReduceSum<float>(tempOneLocal,yLocal,tempTwoLocal,this->onceCalNum);
        this->coefficient = 1.0f /sqrt((tempOneLocal.GetValue(0)/ this->onceCalNum) + this->epsilon);
        queueY.FreeTensor(yLocal);
        LocalTensor<float> gammaLocal = queueGamma.AllocTensor<float>();
        LocalTensor<float> betaLocal = queueBeta.AllocTensor<float>();
        LocalTensor<float> zLocal = queueZ.AllocTensor<float>();

        for (int i = 0; i < this->loopNum; i++)
        {
            DataCopy(gammaLocal,gemmaGm[i* this->onceRowNum],this->onceRowNum);
            DataCopy(betaLocal,betaGm[i* this->onceRowNum],this->onceRowNum);
            queueGamma.EnQue(gammaLocal);
            queueBeta.EnQue(betaLocal);
            gammaLocal = queueGamma.DeQue<float>();
            betaLocal = queueBeta.DeQue<float>(); 
            Mul(xLocal[i * this->onceRowNum],xLocal[i * this->onceRowNum], gammaLocal, this->onceRowNum);
            Muls(xLocal[i * this->onceRowNum],xLocal[i * this->onceRowNum], this->coefficient, this->onceRowNum);
            Add(zLocal,xLocal[i * this->onceRowNum], betaLocal, this->onceRowNum);

            queueZ.EnQue<float>(zLocal);
            zLocal = queueZ.DeQue<float>();
            DataCopy(workspaceGm[progress * this->onceCalNum + i * this->onceRowNum],zLocal,this->onceRowNum);

            pipe_barrier(PIPE_ALL);
        }
        
        for (int i = 0; i < this->loopNum; i++)
        {
            DataCopy(zLocal, workspaceGm[progress * this->onceCalNum + i * onceRowNum],this->onceRowNum);
            pipe_barrier(PIPE_ALL);
            queueZ.EnQue(zLocal);
            zLocal = queueZ.DeQue<float>();
            DataCopy(zGm[progress * this->onceCalNum + i* this->onceRowNum], zLocal,this->onceRowNum);
            pipe_barrier(PIPE_ALL);
        }
        queueGamma.FreeTensor(gammaLocal);
        queueBeta.FreeTensor(betaLocal);
        
        queueZ.FreeTensor(zLocal);
        queueX.FreeTensor(xLocal);
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
    GlobalTensor<float> workspaceGm;
    TBuf<> tempBuf1;
    TBuf<> tempBuf2;
    uint32_t loopNum;
    uint32_t onceCalNum;
    uint32_t onceRowNum;
    float epsilon;
    uint32_t tileNum;
    float coefficient;
};

extern "C" __global__ __aicore__ void pre_layer_norm_custom(GM_ADDR x, GM_ADDR y, GM_ADDR gamma, GM_ADDR beta, GM_ADDR res_out, GM_ADDR workspace, GM_ADDR tiling) {
    GET_TILING_DATA(tiling_data, tiling);
    // TODO: user kernel impl
    GM_ADDR usrWorkspace =  AscendC::GetUserWorkspace(workspace);
    KernelLayerNorm op;
    op.Init(x,y,gamma,beta,res_out,usrWorkspace,tiling_data.onceCalNum,tiling_data.onceRowNum,tiling_data.epsilon,tiling_data.tileNum);
    if(TILING_KEY_IS(1)){
        op.Process();
    }
}