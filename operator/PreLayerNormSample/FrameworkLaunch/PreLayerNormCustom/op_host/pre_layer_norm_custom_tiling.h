
#include "register/tilingdata_base.h"

namespace optiling {
BEGIN_TILING_DATA_DEF(PreLayerNormCustomTilingData)
  TILING_DATA_FIELD_DEF(uint32_t, onceRowNum);
  TILING_DATA_FIELD_DEF(uint32_t, onceCalNum);
  TILING_DATA_FIELD_DEF(uint32_t, tileNum);
  TILING_DATA_FIELD_DEF(float, epsilon);
END_TILING_DATA_DEF;

REGISTER_TILING_DATA_CLASS(PreLayerNormCustom, PreLayerNormCustomTilingData)
}
