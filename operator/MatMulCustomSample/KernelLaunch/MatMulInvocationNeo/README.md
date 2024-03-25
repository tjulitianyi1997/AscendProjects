## 样例介绍

演示如何通过Kernel Launch调用的方式调用MatMul自定义算子。

样例支持的产品型号为：
- Atlas 推理系列产品（Ascend 310P处理器）
- Atlas A2训练系列产品

## 获取源码包
    
 可以使用以下两种方式下载，请选择其中一种进行源码准备。

 - 命令行方式下载（下载时间较长，但步骤简单）。

   ```    
   # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
   cd ${HOME}     
   git clone https://gitee.com/ascend/samples.git
   ```
   **注：如果需要切换到其它tag版本，以v0.5.0为例，可执行以下命令。**
   ```
   git checkout v0.5.0
   ```   
 - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
   **注：如果需要下载其它版本代码，请先请根据前置条件说明进行samples仓分支切换。**   
   ``` 
   # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
   # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
   # 3. 开发环境中，执行以下命令，解压zip包。     
   cd ${HOME}    
   unzip ascend-samples-master.zip
   ```

## 样例运行

  - 打开样例目录
        
    ```    
    cd $HOME/samples/operator/MatMulCustomSample/MatMulInvocationNeo
    ```
  - 配置环境变量
    
    这里的\$HOME需要替换为CANN包的安装路径。
    ```
    export ASCEND_HOME_DIR=$HOME/Ascend/ascend-toolkit/latest
    ```

    配置仿真模式日志文件目录，默认为sim_log。
    ```
    export CAMODEL_LOG_PATH=./sim_log
    ```

  - 样例执行

    ```
    bash run.sh -r [RUN_MODE] -v [SOC_VERSION]
    ```
    - RUN_MODE：编译方式，可选择CPU调试，NPU仿真，NPU上板。支持参数为[cpu / sim/ npu]
    - SOC_VERSION：昇腾AI处理器型号，如果无法确定具体的[SOC_VERSION]，则在安装昇腾AI处理器的服务器执行npu-smi info命令进行查询，在查询到的“Name”前增加Ascend信息，例如“Name”对应取值为xxxyy，实际配置的[SOC_VERSION]值为Ascendxxxyy。支持以下参数取值（xxx请替换为具体取值）：
      - Atlas 推理系列产品（Ascend 310P处理器）参数值：Ascend310P1、Ascend310P3
      - Atlas A2训练系列产品参数值：AscendxxxB1、AscendxxxB2、AscendxxxB3、AscendxxxB4
    
    示例如下。
    ```
    bash run.sh -r cpu -v Ascend310P1
    ```
  

## 更新说明
  | 时间 | 更新事项 | 注意事项 |
|----|------|------|
| 2024/01/26 | 新增Kernel Launch调用算子样例 |需要基于社区CANN包7.0.0.alpha003及之后版本运行|
  

## 已知issue

  暂无
