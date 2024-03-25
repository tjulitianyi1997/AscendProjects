import torch
import torch_npu
import sys,os
sys.path.append(os.getcwd())
import add_custom

x = torch.randn([1024,2048], dtype=torch.float16).npu()
print(x)
y = torch.randn([1024,2048], dtype=torch.float16).npu()
print(y)
z = add_custom.run_add_custom(x, y)
print(z)
