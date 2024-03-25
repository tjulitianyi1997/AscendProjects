#include <pybind11/pybind11.h>
#include "aclrtlaunch_matmul_leakyrelu_custom.h"
#include <torch/extension.h>
#include "torch_npu/csrc/core/npu/NPUStream.h"
#include "acl/acl.h"

extern uint8_t *GenerateTiling();

namespace my_matmul_leakyrelu {
at::Tensor run_matmul_leakyrelu_custom(const at::Tensor& a, const at::Tensor& b, const at::Tensor& bias) {
    auto acl_stream = c10_npu::getCurrentNPUStream().stream(false);
    auto c = at::empty({a.sizes()[0], b.sizes()[1]}, at::TensorOptions().dtype(at::kFloat).device(a.options().device()));
    auto workspace_size = 96 * 1024 * 1024 * sizeof(float);
    auto workspace_tensor = at::empty({workspace_size}, at::TensorOptions().dtype(at::kByte).device(a.options().device()));

    size_t tilingFileSize = 48 * sizeof(int32_t);
    uint8_t *tilingHost;
    uint8_t *tilingDevice;

    aclrtMallocHost((void**)(&tilingHost), tilingFileSize);
    aclrtMalloc((void**)&tilingDevice, tilingFileSize, ACL_MEM_MALLOC_HUGE_FIRST);
    aclrtMemcpy(tilingHost, tilingFileSize, GenerateTiling(), tilingFileSize, ACL_MEMCPY_HOST_TO_HOST);
    aclrtMemcpy(tilingDevice, tilingFileSize, tilingHost, tilingFileSize, ACL_MEMCPY_HOST_TO_DEVICE);

#ifdef CUSTOM_ASCEND310P
    uint32_t blockDim = 2;
#else
    uint32_t blockDim = 1;
#endif
    ACLRT_LAUNCH_KERNEL(matmul_leakyrelu_custom)(blockDim, acl_stream, a.storage().data(), b.storage().data(), bias.storage().data(), c.storage().data(), workspace_tensor.storage().data(), tilingDevice);
    return c;
}
}

PYBIND11_MODULE(matmul_leakyrelu_custom, m) {
    m.doc() = "matmul_leakyrelu_custom pybind11 interfaces"; // optional module docstring
    m.def("run_matmul_leakyrelu_custom", &my_matmul_leakyrelu::run_matmul_leakyrelu_custom, "");
}
