# apollo.ros-1.0.0

## 简介

 此工程为基于 apollo 1.0.0 的ros1移植版本，移植主要目的如下：

- 学习apollo框架设计
- 学习apollo中的控制算法

目前只针对apollo中主要模块进行了移植，移植模块有:

```bash
apollo_ros
    ├── apollo_common
    ├── apollo_control
    ├── apollo_decision
    ├── apollo_localization
    ├── apollo_msgs
    ├── apollo_planning
    └── apollo_simulator
```

目前移植版本与原有版本改动点如下：

- 使用原生ros（基于noetic）替代apollo中更改的ros
- 使用ros pkg封装apollo中每个module
- 使用cmake进行编译
- 将protobuff版本提升到3.6.1
- 使用ros中的`std_msgs/String`替代apollo的`pd_msgs/xxx`消息
- 增加pnc仿真工具`apollo_simulator`

>此移植版本，能很好的将自己的算法增加到框架中，应用于机器人或者无人驾驶中。同时，由于apollo中的模块抽象，每个模块之间和中间件没有耦合，中间件能很容易从ros1移植到ros2、LCM等，具体开发可根据自己的需求进行魔改。

## 安装

### 依赖

- ros1（noetic，安装见ros官网）

- eigen3

  ```bash
  sudo apt install libeigen3-dev
  ```

- glog

  ```bash
  sudo apt install libgoogle-glog-dev
  ```

- gflags

  ```bash
  sudo apt install libgflags-dev 
  ```

- gtest

  ```bash
  sudo apt install libgtest-dev 
  ```

- protobuff 3.6.1

  ```shell
  wget https://github.com/google/protobuf/releases/download/v3.6.1/protobuf-cpp-3.6.1.tar.gz
  tar xzf protobuf-cpp-3.6.1.tar.gz
  
  pushd protobuf-3.6.1
  ./configure --prefix=/usr
  make -j8
  make install
  popd
  ```

### 编译

- 创建的ros工作空间
- 使用`catkin_make`或者`catkin build`指令进行代码编译

##  测试

这里主要使用`apollo_simulator`仿真工具进行规划控制（pnc）进行测试，测试指令如下：

```shell
roslaunch apollo_simulator simulation.launch
```

## 维护者

[@Forrest](709335543@qq.com)
