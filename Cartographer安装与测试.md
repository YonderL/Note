# Cartographer安装与测试

## 简介
`Cartographer`是`Google`推出的一套基于图优化的`激光SLAM`算法，它同时支持`2D`和`3D`激光SLAM，可以跨平台使用，支持`Lidar`、`IMU`、`Odemetry`、`GPS`、`Landmark`等多种传感器配置。是目前落地应用最广泛的`激光SLAM`算法之一。

## Cartographer的安装
`Cartographer`具有非常详细的[官方文档](https://google-cartographer-ros.readthedocs.io/en/latest/index.html)，该官方文档详细书写了如何在noetic和melodic两个版本的ROS上使用Cartographer，关于ROS2的文档，似乎没有官方文档，但也有一些相关的[参考资料](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html#technical-overview)。

`Cartographer`的安装可以按照官方文档进行，当这可能会出现一系列例如软件包版本冲突、`rosdep`初始化失败、连接不到外网等问题，因此这里个人推荐使用小鱼ROS的`Cartographer`一键安装。只需要在终端中输入
```
wget http://fishros.com/install -O fishros && . fishros
```
之后按照终端打印内容进行操作即可。

### 小鱼一键安装Cartographer可能会出现的问题
当安装过程中出现红色的Erro时，说明安装出现的一定问题，发生了安装失败的现象，这里仅记录自己安装失败时的Erro与解决方案。
```
Run CMD Task:[sudo apt-get remove ros-noetic-abseil-cpp -y]
[-]Result:code:100 

Run CMD Task:[catkin_make_isolated --install --use-ninja]
[-]Result:code:127 
```
这里表明运行以下两条指令失败：
```
sudo apt-get remove ros-noetic-abseil-cpp -y
catkin_make_isolated --install --use-ninja
```
解决方案：
进入`cartographer_ws`文件夹，依次重新运行以上两条指令即可。

## Cartographer的测试
安装成功后，需要测试其是否安装完毕，一个简单的测试方法即为在官方的德意志博物馆数据集上进行2D建图测试，官方文档中为：
```
wget -P ~/Downloads https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
其中第一条指令为下载数据集，但下载地址来自google，因此需要科学上网手段，且发现开启科学上网后运行该指令的下载速度不如直接在浏览器访问地址：https://storage.googleapis.com/cartographer-public-data/bags/backpack_2d/cartographer_paper_deutsches_museum.bag 下载速度更快。


下载完毕后，准备运行第二条指令，这需要首先进入`cartographer_ws`文件夹，运行以下指令，激活环境变量。
```
source install_isolated/setup.bash
```
之后运行
```
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag
```
`bag_filename`后面要替换为下载的数据集的文件地址。
效果：
![效果图](pic/23.png)