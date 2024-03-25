#include "kernel_operator.h"
#include "lib/matmul_intf.h"

using namespace AscendC;
using namespace matmul;

__aicore__ inline void CalcGMOffset(int blockIdx, int usedCoreNum,
                                    const TCubeTiling& param, int& offsetA,
                                    int& offsetB, int& offsetC,
                                    int& curSingleCoreM, int& curSingleCoreN, 
                                    int& curSingleCoreK) {
  ASSERT(blockIdx < usedCoreNum);
  uint32_t mIterSize = Ceil(param.M, param.singleCoreM);
  uint32_t mCoreIndx = blockIdx % mIterSize;
  uint32_t nCoreIndx = blockIdx / mIterSize;

  offsetA = mCoreIndx * param.Ka * param.singleCoreM;
  offsetB = nCoreIndx * param.singleCoreN;
  offsetC =
      mCoreIndx * param.N * param.singleCoreM + nCoreIndx * param.singleCoreN;

  // tail M
  int gmUseM = param.M - mCoreIndx * param.singleCoreM;
  curSingleCoreM = gmUseM < param.singleCoreM ? gmUseM : param.singleCoreM;

  // tail N
  int gmUseN = param.N - nCoreIndx * param.singleCoreN;
  curSingleCoreN = gmUseN < param.singleCoreN ? gmUseN : param.singleCoreN;

  // tail K
  int gmUseK = param.Ka;
  curSingleCoreK = gmUseK < param.singleCoreK ? gmUseK : param.singleCoreK;
}

__aicore__ inline void CopyTiling(TCubeTiling* tiling, GM_ADDR tilingGM) {
  uint32_t* ptr = reinterpret_cast<uint32_t*>(tiling);
  auto tiling32 = reinterpret_cast<__gm__ uint32_t*>(tilingGM);

  for (int i = 0; i < sizeof(TCubeTiling) / sizeof(uint32_t); i++, ptr++) {
    *ptr = *(tiling32 + i);
  }
  return;
}

extern "C" __global__ __aicore__ void matmul_custom(GM_ADDR a, GM_ADDR b,
                                                    GM_ADDR c,
                                                    GM_ADDR workspace,
                                                    GM_ADDR tilingGm) {
  using A_T = half;
  using B_T = half;
  using C_T = float;
  using BiasT = float;

  TPipe que;
  TCubeTiling tiling;
  CopyTiling(&tiling, tilingGm);

  GlobalTensor<A_T> aGlobal;
  GlobalTensor<B_T> bGlobal;
  GlobalTensor<C_T> cGlobal;
  aGlobal.SetGlobalBuffer(reinterpret_cast<__gm__ A_T*>(a),
                          tiling.M * tiling.Ka);
  bGlobal.SetGlobalBuffer(reinterpret_cast<__gm__ B_T*>(b),
                          tiling.Ka * tiling.N);
  cGlobal.SetGlobalBuffer(reinterpret_cast<__gm__ C_T*>(c),
                          tiling.M * tiling.N);

  int offsetA = 0;
  int offsetB = 0;
  int offsetC = 0;
  int curSingleCoreM = 0;
  int curSingleCoreN = 0;
  int curSingleCoreK = 0;

  CalcGMOffset(GetBlockIdx(), tiling.usedCoreNum, tiling, offsetA, offsetB,
               offsetC, curSingleCoreM, curSingleCoreN, curSingleCoreK);
  auto gmA = aGlobal[offsetA];
  auto gmB = bGlobal[offsetB];
  auto gmC = cGlobal[offsetC];
  Matmul<MatmulType<TPosition::GM, CubeFormat::ND, A_T>,
         MatmulType<TPosition::GM, CubeFormat::ND, B_T>,
         MatmulType<TPosition::GM, CubeFormat::ND, C_T>,
         MatmulType<TPosition::GM, CubeFormat::ND, BiasT>>
      mm;
  REGIST_MATMUL_OBJ(&que, GetSysWorkSpacePtr(), mm, &tiling);
#ifdef CUSTOM_ASCEND310P
  TBuf<> tmpMMFormatUb;
  LocalTensor<uint8_t> mmFormatUb;
  que.InitBuffer(tmpMMFormatUb, TOTAL_VEC_LOCAL_SIZE);
  mmFormatUb = tmpMMFormatUb.Get<uint8_t>(TOTAL_VEC_LOCAL_SIZE);
  mm.SetLocalWorkspace(mmFormatUb);
#endif
  mm.SetTensorA(gmA);
  mm.SetTensorB(gmB);
  mm.SetTail(curSingleCoreM, curSingleCoreN, curSingleCoreK);
  mm.IterateAll(gmC);
  mm.End();
}
