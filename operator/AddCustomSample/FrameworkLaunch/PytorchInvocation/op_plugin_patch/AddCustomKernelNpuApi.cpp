#include "op_plugin/OpApiInterface.h"
#include "op_plugin/AclOpsInterface.h"
#include "op_plugin/utils/op_api_common.h"

namespace op_api {
using npu_preparation = at_npu::native::OpPreparation;

at::Tensor npu_add_custom(const at::Tensor& x, const at::Tensor& y) {
    at::Tensor result = npu_preparation::apply_tensor_without_format(x); // 创建输出内存

    // calculate the output result of the NPU
    EXEC_NPU_CMD(aclnnAddCustom, x, y, result);
    return result;
}
} // namespace at_npu
