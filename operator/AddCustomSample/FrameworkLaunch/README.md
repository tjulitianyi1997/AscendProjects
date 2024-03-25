## 目录结构
| 目录                  | 描述                   |
|---------------------|----------------------|
| [AclNNInvocation](./AclNNInvocation) | 通过aclnn调用的方式调用AddCustom算子。 |
| [AclNNInvocationNaive](./AclNNInvocationNaive) | 通过aclnn调用的方式调用AddCustom算子, 简化了编译脚本。 |
| [AclOfflineModel](./AclOfflineModel) | 通过aclopExecuteV2调用的方式调用AddCustom算子。 |
| [AclOnlineModel](./AclOnlineModel) | 通过aclopCompile调用的方式调用AddCustom算子。 |
| [AddCustom](./AddCustom)       | AddCustom算子工程 |
| [CppExtensions](./CppExtensions)       | Pybind方式调用AddCustom算子 |
| [PytorchInvocation](./PytorchInvocation) | 通过pytorch调用的方式调用AddCustom算子。 |
| [TensorflowInvocation](./TensorflowInvocation) | 通过tensorflow调用的方式调用AddCustom算子。 |
| AddCustom.json       | AddCustom算子的原型定义json文件 |

## 样例支持的产品型号为：
- Atlas 训练系列产品
- Atlas 200/500 A2 推理产品
- Atlas 推理系列产品（Ascend 310P处理器）
- Atlas A2训练系列产品

## 编译算子工程部署算子包

### 1.获取源码包

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

### 2.编译算子工程

  编译自定义算子工程，构建生成自定义算子包

  - 执行如下命令，切换到算子工程AddCustom目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/AddCustom
    ```

  - 修改CMakePresets.json中ASCEND_CANN_PACKAGE_PATH为CANN软件包安装后的实际路径。


    ```
    {
        ……
        "configurePresets": [
            {
                    ……
                    "ASCEND_CANN_PACKAGE_PATH": {
                        "type": "PATH",
                        "value": "/usr/local/Ascend/ascend-toolkit/latest"        //请替换为CANN软件包安装后的实际路径。eg:/home/HwHiAiUser/Ascend/ascend-toolkit/latest
                    },
                    ……
            }
        ]
    }
    ```
  - 在算子工程AddCustom目录下执行如下命令，进行算子工程编译。

    ```
    ./build.sh
    ```
    编译成功后，会在当前目录下创建build_out目录，并在build_out目录下生成自定义算子安装包custom_opp_<target os>_<target architecture>.run，例如“custom_opp_ubuntu_x86_64.run”。
  - 备注：如果要使用dump调试功能，需要移除op_host内和CMakeLists.txt内的Atlas 训练系列产品、Atlas 200/500 A2 推理产品的配置

### 3.部署算子包

  - 执行如下命令，在自定义算子安装包所在路径下，安装自定义算子包。

    ```
    cd build_out
    ./custom_opp_<target os>_<target architecture>.run
    ```

    命令执行成功后，自定义算子包中的相关文件将部署至当前环境的OPP算子库的vendors/customize目录中。

## 配置环境变量

  这里的\$HOME需要替换为CANN包的安装路径。
  ```
  export ASCEND_HOME_DIR=$HOME/Ascend/ascend-toolkit/latest
  ```

## 通过aclnn调用的方式调用AddCustom算子工程

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/AclNNInvocation
    ```

  - 样例执行

    样例执行过程中会自动生成测试数据，然后编译与运行aclnn样例，最后检验运行结果。具体过程可参见run.sh脚本。
    ```
    bash run.sh
    ```

## 通过aclnn调用的方式调用AddCustom算子工程(代码简化)

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/AclNNInvocationNaive
    ```
  - 样例编译文件修改
    将CMakeLists.txt文件内“/usr/local/Ascend/ascend-toolkit/latest” 替换为CANN软件包安装后的实际路径。eg:/home/HwHiAiUser/Ascend/ascend-toolkit/latest

  - 环境变量配置
    需要设置NPU_HOST_LIB环境变量，以x86为例
    ```
    export NPU_HOST_LIB=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/x86_64-linux/lib64
    ```
  - 样例执行

    样例执行过程中会自动生成测试数据，然后编译与运行aclnn样例，最后打印运行结果。
    ```
    mkdir -p build
    cd build
    cmake .. && make
    ./execute_add_op
    ```

## 使用aclopExecuteV2模型调用的方式调用AddCustom算子工程
 - 该样例暂不支持Atlas 200/500 A2 推理产品

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/AclOfflineModel
    ```

  - 样例执行

    样例执行过程中会自动生成测试数据，然后编译与运行acl离线模型调用样例，最后检验运行结果。具体过程可参见run.sh脚本。
    ```
    bash run.sh
    ```

## 使用aclopCompile模型调用的方式调用AddCustom算子工程
 - 该样例暂不支持Atlas 200/500 A2 推理产品

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/AclOnlineModel
    ```

  - 样例执行

    样例执行过程中会自动生成测试数据，然后编译与运行aclopCompile调用方式的模型调用样例，最后检验运行结果。具体过程可参见run.sh脚本。
    ```
    bash run.sh
    ```

## 使用cpp-extension的方式调用AddCustom算子工程

  cpp-extension方式是通过编译出一个C++的算子扩展包的形式来调用，可以分为Jit（即时编译）和编译wheel包的形式。

### 使用Jit的方式调用
  - 安装pytorch (这里使用2.1.0版本为例)
    ```
    pip3 install pyyaml
    pip3 install setuptools
    ```
  - 安装torch-npu （以Pytorch2.1.0、python3.9、CANN版本8.0.RC1.alpha002为例）

    ```
    git clone https://gitee.com/ascend/pytorch.git -b v6.0.rc1.alpha002-pytorch2.1.0
    cd pytorch/
    bash ci/build.sh --python=3.9
    pip3 install dist/*.whl
    ```

  - 安装测试依赖

    ```
    pip3 install Ninja
    pip3 install expecttest
    ```

  - 其他版本pytorch安装参考：https://gitee.com/ascend/pytorch  readme描述

#### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/CppExtensions
    ```

  - 配置环境变量

    ```
    export LD_LIBRARY_PATH=$ASCEND_OPP_PATH/vendors/customize/op_api/lib/:$LD_LIBRARY_PATH
    ```

  - 样例执行

    样例执行过程中会自动生成测试数据，然后通过cpp_extension扩展机制，使用实时编译模式编译并执行AclNN算子接口，最后检验运行结果。
    ```
    python test_opp_add_jit.py
    ```

### 使用编译wheel包的方式调用

  样例以pytorch2.1版本为例，2.1以下版本中NPU设备绑定的设备名称有变化，参考样例代码中的注释说明

  - 安装pytorch和torch-npu

#### 编译自定义算子wheel包

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/CppExtensions
    ```

  - 执行编译命令

    ```
    python3 setup.py build bdist_wheel
    ```

  - 安装wheel包

    ```
    cd dist/
    pip3 install custom_ops-1.0-cp38-cp38-linux_aarch64.whl (需要修改为实际编译出的whl包)
    ```

#### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/CppExtensions
    ```

  - 配置环境变量

    ```
    export LD_LIBRARY_PATH=$ASCEND_OPP_PATH/vendors/customize/op_api/lib/:$LD_LIBRARY_PATH
    ```

  - 样例执行

    ```
    python3 test_opp_add_lib.py
    ```

## 使用pytorch调用的方式调用AddCustom算子工程
- 该样例脚本基于Pytorch2.1、python3.9 运行

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/PytorchInvocation
    ```

  - 样例执行

    样例执行过程中会自动生成测试数据，然后运行pytorch样例，最后检验运行结果。具体过程可参见run_op_plugin.sh脚本。
    ```
    bash run_op_plugin.sh
    ```

## 使用tensorflow调用的方式调用AddCustom算子工程

### 样例运行

  - 进入到样例目录

    ```
    cd $HOME/samples/operator/AddCustomSample/FrameworkLaunch/TensorflowInvocation
    ```

  - 样例执行(tensorflow1.15)

    样例执行过程中会自动生成随机测试数据，然后通过TensorFlow调用算子，最后对比cpu和aicore运行结果。具体过程可参见run_add_custom.py脚本。
    ```
    python3 run_add_custom.py
    ```
  - 样例执行(tensorflow2.x)

    样例执行过程中会自动生成随机测试数据，然后通过TensorFlow调用算子，最后对比cpu和aicore运行结果。具体过程可参见run_add_custom_tf2.py脚本。
    ```
    python3 run_add_custom_tf2.py
    ```

## 更新说明
  | 时间 | 更新事项 |
|----|------|
| 2023/10/24 | 新增TensorflowInvocation样例 |
| 2023/10/18 | 新增AclNNInvocation样例 |
| 2024/01/11 | 更改pytorch适配方式 |
| 2024/01/23 | 新增AclNNInvocationNaive样例|


## 已知issue

  暂无