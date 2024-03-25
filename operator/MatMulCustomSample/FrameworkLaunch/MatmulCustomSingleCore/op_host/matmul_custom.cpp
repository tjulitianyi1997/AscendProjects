#include "matmul_custom_tiling.h"
#include "register/op_def_registry.h"
#include "tiling/platform/platform_ascendc.h"
#include "tiling/tiling_api.h"

using namespace matmul_tiling;

namespace optiling {
static ge::graphStatus TilingFunc(gert::TilingContext* context)
{
  auto ascendcPlatform = platform_ascendc::PlatformAscendC(context->GetPlatformInfo());
  
  auto shape_a = context->GetInputTensor(0)->GetOriginShape();
  auto shape_b = context->GetInputTensor(1)->GetOriginShape();
  int32_t M = shape_a.GetDim(0);
  int32_t N = shape_b.GetDim(1);
  int32_t K = shape_a.GetDim(1);
  int32_t baseM = 128;
  int32_t baseN = 128;
  MatmulApiTiling cubeTiling(ascendcPlatform); 
  cubeTiling.SetAType(TPosition::GM, CubeFormat::ND, matmul_tiling::DataType::DT_FLOAT16);
  cubeTiling.SetBType(TPosition::GM, CubeFormat::ND, matmul_tiling::DataType::DT_FLOAT16);
  cubeTiling.SetCType(TPosition::LCM, CubeFormat::ND, matmul_tiling::DataType::DT_FLOAT);
  cubeTiling.SetBiasType(TPosition::GM, CubeFormat::ND, matmul_tiling::DataType::DT_FLOAT);
  cubeTiling.SetShape(M, N, K);
  cubeTiling.SetOrgShape(M, N, K);
  cubeTiling.SetFixSplit(baseM, baseN, -1);
  cubeTiling.SetBias(true);
  cubeTiling.SetBufferSpace(-1, -1, -1);
  MatmulCustomTilingData tiling;
  if (cubeTiling.GetTiling(tiling.cubeTilingData) == -1){
      return ge::GRAPH_FAILED;
  }

  if (ascendcPlatform.GetSocVersion() == platform_ascendc::SocVersion::ASCEND310P) {
      context->SetTilingKey(2);
  }else {
      context->SetTilingKey(1);
  }
  context->SetBlockDim(1);
  tiling.SaveToBuffer(context->GetRawTilingData()->GetData(), context->GetRawTilingData()->GetCapacity());
  context->GetRawTilingData()->SetDataSize(tiling.GetDataSize());
  size_t userWorkspaceSize = 0;
  size_t systemWorkspaceSize = ascendcPlatform.GetLibApiWorkSpaceSize();
  size_t *currentWorkspace = context->GetWorkspaceSizes(1);
  currentWorkspace[0] = userWorkspaceSize + systemWorkspaceSize;

  return ge::GRAPH_SUCCESS;
}
}

namespace ops {
class MatmulCustom : public OpDef {
public:
    explicit MatmulCustom(const char* name) : OpDef(name)
    {
        this->Input("a")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT16})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("b")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT16})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("bias")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Output("c")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});


        this->AICore()
            .SetTiling(optiling::TilingFunc);
        this->AICore().AddConfig("ascend910b");
        this->AICore().AddConfig("ascend310p");

    }
};

OP_ADD(MatmulCustom);
}
