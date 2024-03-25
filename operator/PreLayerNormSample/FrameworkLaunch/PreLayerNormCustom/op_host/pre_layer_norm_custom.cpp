
#include "pre_layer_norm_custom_tiling.h"
#include "register/op_def_registry.h"


namespace optiling {
const uint32_t BLOCK_DIM = 48;
const uint32_t ONCE_ROW_NUM = 2048;
const uint32_t ONCE_CAL_NUM = 20480;

static ge::graphStatus TilingFunc(gert::TilingContext* context)
{

  PreLayerNormCustomTilingData tiling;
  const float* epsilonAttr = context->GetAttrs()->GetAttrPointer<float>(0);
  tiling.set_epsilon(*epsilonAttr);
  uint32_t totalLength = context->GetInputTensor(0)->GetShapeSize();
  gert::Shape oriShape = context->GetInputShape(0)->GetOriginShape();
  context->SetBlockDim(BLOCK_DIM);
  tiling.set_onceCalNum(ONCE_CAL_NUM);
  tiling.set_onceRowNum(ONCE_ROW_NUM);
  uint32_t tile_num = totalLength / ONCE_CAL_NUM /BLOCK_DIM;
  tiling.set_tileNum(tile_num);

  tiling.SaveToBuffer(context->GetRawTilingData()->GetData(), context->GetRawTilingData()->GetCapacity());
  context->GetRawTilingData()->SetDataSize(tiling.GetDataSize());
  context->SetTilingKey(1);
  size_t usrSize = 512 * 4 * 20480 * 1024;
  size_t sysWorksapceSize = 16 *1024 * 1024;
  size_t *currentWorkSpace = context->GetWorkspaceSizes(1);
  currentWorkSpace[0] = usrSize + sysWorksapceSize;
  return ge::GRAPH_SUCCESS;
}
}


namespace ge {
static ge::graphStatus InferShape(gert::InferShapeContext* context)
{
    const gert::Shape* x1_shape = context->GetInputShape(0);
    gert::Shape* y_shape = context->GetOutputShape(0);
    *y_shape = *x1_shape;
    return GRAPH_SUCCESS;
}
}


namespace ops {
class PreLayerNormCustom : public OpDef {
public:
    explicit PreLayerNormCustom(const char* name) : OpDef(name)
    {
        this->Input("x")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("y")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("gamma")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Input("beta")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Output("res_out")
            .ParamType(REQUIRED)
            .DataType({ge::DT_FLOAT})
            .Format({ge::FORMAT_ND})
            .UnknownShapeFormat({ge::FORMAT_ND});
        this->Attr("epsilon").Float();

        this->SetInferShape(ge::InferShape);

        this->AICore()
            .SetTiling(optiling::TilingFunc);
        this->AICore().AddConfig("ascend910b");

    }
};

OP_ADD(PreLayerNormCustom);
}
