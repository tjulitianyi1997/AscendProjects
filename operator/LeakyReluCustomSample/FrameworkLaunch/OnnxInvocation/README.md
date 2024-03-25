AscendC 自定义算子入Onnx网络示例教程:
以leakyrelu单算子离线推理为例
推理平台：Ascend310P3

一、自定义算子准备
1.在LeakyReluCustom目录下执行编译操作，编译出算子run包
2.安装在LeakyReluCustom/build_out/目录下生成的自定义算子run包

二、离线推理验证流程
1.获取单算子onnx模型, 该模型参考leaky_relu.py生成
    ```
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/AscendC/leaky_relu.onnx
    ```

2.onnx模型转换为om模型
For Ascend310P3:
atc --model=./leaky_relu.onnx --framework=5 --soc_version=Ascend310P3 --output=./leaky_relu --input_shape="X:8,16,1024" --input_format=ND


若出现:
start compile Ascend C operator LeakyReluCustom. kernel name is leaky_relu_custom
compile Ascend C operator: LeakyReluCustom success!
打印，表明进入了AscendC算子编译

出现ATC run success, welcome to the next use 表明离线om模型转换成功

3.执行离线推理
可使用https://gitee.com/ascend/tools/tree/master/msame 工具进行推理验证
