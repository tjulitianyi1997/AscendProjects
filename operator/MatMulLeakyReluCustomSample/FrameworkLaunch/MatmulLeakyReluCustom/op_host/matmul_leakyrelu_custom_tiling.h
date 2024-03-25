#ifndef MATMUL_LEAKYRELU_TILING_H
#define MATMUL_LEAKYRELU_TILING_H

#include "register/tilingdata_base.h"
#include "tiling/tiling_api.h"

namespace optiling {
BEGIN_TILING_DATA_DEF(MatmulLeakyreluCustomTilingData)
  TILING_DATA_FIELD_DEF(float, alpha);
  TILING_DATA_FIELD_DEF_STRUCT(TCubeTiling, cubeTilingData);
END_TILING_DATA_DEF;

REGISTER_TILING_DATA_CLASS(MatmulLeakyreluCustom, MatmulLeakyreluCustomTilingData)
}

#endif