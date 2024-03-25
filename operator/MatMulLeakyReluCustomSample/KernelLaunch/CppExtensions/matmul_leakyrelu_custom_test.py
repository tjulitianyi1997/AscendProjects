import torch
import torch.nn as nn
import torch_npu
import sys, os

sys.path.append(os.getcwd())
import matmul_leakyrelu_custom

a = torch.randn([1024, 256], dtype=torch.float16).npu()
print("a is", a)
b = torch.randn([256, 640], dtype=torch.float16).npu()
print("b is ", b)
bias = torch.randn([640], dtype=torch.float32).npu()
print("bias is ", bias)

c = matmul_leakyrelu_custom.run_matmul_leakyrelu_custom(a, b, bias)
print("c is ", c)
m = nn.LeakyReLU(0.001)
golden = m(torch.matmul(a.cpu().type(c.dtype), b.cpu().type(c.dtype)) + bias.cpu())
print("golden is ", golden)
