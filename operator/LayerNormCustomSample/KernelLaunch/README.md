## 样例介绍

演示如何通过核函数调用的方式调用LayerNorm自定义算子，该样例为固定shape，主要演示核函数的调用。

该算子的算子原型如下：
<table>
<tr><td rowspan="1" align="center">算子类型(OpType)</td><td colspan="4" align="center">LayerNorm</td></tr>
</tr>
<tr><td rowspan="4" align="center">算子输入</td><td align="center">name</td><td align="center">shape</td><td align="center">data type</td><td align="center">format</td></tr>
<tr><td align="center">x</td><td align="center">4980 * 4 * 2048</td><td align="center">float</td><td align="center">ND</td></tr>
<tr><td align="center">gamma</td><td align="center">2048</td><td align="center">float</td><td align="center">ND</td></tr>
<tr><td align="center">beta</td><td align="center">2048</td><td align="center">float</td><td align="center">ND</td></tr>
</tr>
</tr>
<tr><td rowspan="1" align="center">算子输出</td><td align="center">z</td><td align="center">4980 * 4 * 2048</td><td align="center">float</td><td align="center">ND</td></tr>
</tr>
<tr><td rowspan="1" align="center">核函数名</td><td colspan="4" align="center">layer_norm_custom</td></tr>
</table>

## 样例支持的产品型号为：
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
    cd $HOME/samples/operator/LayerNormCustomSample/KernelLaunch
    ```
  - 配置环境变量
    
    这里的\$HOME需要替换为CANN包的安装路径。
    ```
    export ASCEND_HOME_DIR=$HOME/Ascend/ascend-toolkit/latest
    ```

  - 样例执行

    ```
    bash run.sh [RUN_MODE] 
    ```
    - RUN_MODE：编译方式，可选择CPU调试，NPU仿真，NPU上板。支持参数为[cpu / sim / npu]。



    示例如下。
    ```
    bash run.sh cpu
    ```
