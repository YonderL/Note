# URDF机器人仿真建模

## 为什么我们需要机器人仿真？
- `低成本`:当前机器人成本居高不下，动辄几十万，仿真可以大大降低成本，减小风险
- `高效`:搭建的环境更为多样且灵活，可以提高测试效率以及测试覆盖率
- `高安全性`:仿真环境下，无需考虑耗损问题

仿真具有诸如以上的优点，但同样也存在一定的缺陷：

- 仿真器所使用的物理引擎目前还不能够完全精确模拟真实世界的物理情况
- 仿真器构建的是关节驱动器（电机&齿轮箱）、传感器与信号通信的绝对理想情况，目前不支持模拟实际硬件缺陷或者一些临界状态等情形

## 常用来进行机器人仿真的工具

### `URDF`
`URDF`是 `Unified Robot Description Format` 的首字母缩写，直译为统一(标准化)机器人描述格式，可以以一种 XML 的方式描述机器人的部分结构，比如底盘、摄像头、激光雷达、机械臂以及不同关节的自由度.....,该文件可以被 `C++` 内置的解释器转换成可视化的机器人模型，是 ROS 中实现机器人仿真的重要组件。\
简而言之，我们通常使用URDF来进行机器人的本体建模

### `Rviz`
`RViz` 是 `ROS Visualization Tool` 的首字母缩写，直译为ROS的三维可视化工具。\
它的主要目的是以三维方式显示`ROS`消息，可以将数据进行可视化表达。例如:可以显示机器人模型，可以无需编程就能表达激光测距仪（LRF）传感器中的传感器到障碍物的距离，`RealSense`、`Kinect`或`Xtion`等三维距离传感器的点云数据（PCD， Point Cloud Data），从相机获取的图像值等。\
通常，我们使用`Rviz`来进行机器人运动与感知数据的可视化。

### `Gazebo`
`Gazebo`是一款3D动态模拟器，用于显示机器人模型并创建仿真环境,能够在复杂的室内和室外环境中准确有效地模拟机器人。与游戏引擎提供高保真度的视觉模拟类似，`Gazebo`提供高保真度的物理模拟，其提供一整套传感器模型，以及对用户和程序非常友好的交互方式。\
通常，我们使用`Gazebo`来进行机器人实际仿真物理环境的搭建

### 综合使用
`URDF`只是一个单纯的`XML`文件，我们需要使用`Rviz`等工具进行渲染可视化，因此我们需要综合使用上面提到的三种仿真工具。
- 若不仿真环境信息，使用真实物理环境，则使用`URDF`结合`Rviz`直接显示感知的真实环境信息
- 若需要仿真环境，则使用`URDF`结合`Gazebo`搭建仿真环境，并结合`Rviz`显示感知的虚拟环境信息

## `URDF` + `Rviz`基本流程

### 功能包结构
    新的功能包依赖：

对于实际的机器人系统，我们通常使用一个单独的功能包来进行机器人建模，这里我们新建一个功能包，除前面提到的`roscpp` `rospy` `std_msgs`的依赖外，需要有`urdf`和`xacro`依赖。

    新的目录结构：（以下四个均为新的文件夹）
- `urdf`:存储`urdf`文件
- `meshes`:机器人模型渲染文件
- `config`:配置文件
- `launch`:`launch`启动文件

### 编写URDF文件
在`urdf`文件夹下新建`xxx.urdf`文件
示例内容：
```urdf
<robot name="mycar">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
        </visual>
    </link>
</robot>
```

### 编写`launch`文件
```launch
<launch>

    <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find urdf_rviz)/urdf/testone.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" />

</launch>

```

### 在 Rviz 中显示
![可视化](pic/5.png)
使用该方法打开可视化，但是却发现该报错：

    No transform from [base_link] to [map]

报错位置如下：
![报错](pic/6.png)

    问题解决：将Global Options-Fixed Frame改为base_link即可
![报错](pic/7.png)

为了方便我们一打开就可以看到模型，我们可以将`config`使用`save config as`保存入`config`目录，\
并在`launch`文件的`rviz`启动配置中添加参数`args`，值设置为`-d 配置文件路径`：
```
     <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find urdf_rviz)/urdf/testone.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz)/config/rviz/show_mycar.rviz"/>
```

## URDF语法


