## 样例介绍

功能：演示使用<<<>>>内核调用符完成算子核函数NPU侧运行验证的基础流程和PRINTF宏打印；

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
    cd $HOME/samples/operator/HelloWorldSample
    ```
  - 配置环境变量
    
    这里的\$HOME需要替换为CANN包的安装路径。
    ```
    export ASCEND_HOME_DIR=$HOME/Ascend/ascend-toolkit/latest
    ```
  
  - 配置修改
    * 修改CMakeLists.txt内SOC_VERSION为所需产品型号
    * 修改CMakeLists.txt内ASCEND_CANN_PACKAGE_PATH为CANN包的安装路径

  - 样例执行

    执行以下命令会完成可执行程序编译及运行
    ```
    bash run.sh
    ```

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2024/3/7 | 修改样例编译方式，并添加PRINTF宏使用方式展示 |
| 2023/10/23 | 新增HelloWorldSample样例 |
  

## 已知issue

  暂无
