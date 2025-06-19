# URDF机器人与Xacro仿真建模(Rviz下的使用)

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
URDF的本质是`xml`语法，`xml`的特点在于标签的分级和首尾要求相同，且标签的尾部需要有`/`，`xml`的属性跟在开头标签的后面，子标签需要换行缩进表示所属。这些特点，我们在下面也可以发现。
### 根标签`robot`
示例：
```xml
<robot name="mycar">
    ....
</robot>
```
属性name表示机器人模型的名称，urdf所有`link`和`joint`都需要包含于该根标签下。
### 刚体`link`
`link`标签用于描述机器人某个部件(也即`刚体`部分)的外观和物理属性，比如: 机器人底座、轮子、激光雷达、摄像头...每一个部件都对应一个 link, 在 link 标签内，可以设计该部件的形状、尺寸、颜色、惯性矩阵、碰撞参数等一系列属性。
#### 属性
- `name`
#### 子标签
- `visual`表示描述外观
    - `geometry`表示描述形状
      - `box` `cylinder` `sphere`分别对应长方体，圆柱，球体，属性各不相同(最小子标签)
    - `origin`设置偏移量与倾斜弧度，`xyz`和`rpy`两个属性
    - `material`设置材料颜色，属性`name`表示颜色
      - `color`色盘，属性`rgba`四种原色(最小子标签)

#### 示例：
```xml
    <link name="base_link">
    <!-- 属性name -->
        <!-- visual表示描述外观 -->
        <visual>
            <!-- geometry描述形状 -->
            <geometry>
                <!-- 长方体的长宽高 -->
                <box size="0.5 0.3 0.1" />
                <!-- 圆柱，半径和长度 -->
                <!-- <cylinder radius="0.5" length="0.1" /> -->
                <!-- 球体，半径-->
                <!-- <sphere radius="0.3" /> -->
                <!-- 注意，这里只会显示设置的第一个形状，如果我们想要再实例化一个物体，则需要新建一个link -->
            </geometry>
            <!-- origin 设置偏移量与倾斜弧度 xyz坐标 rpy翻滚俯仰与偏航角度(3.14=180度 1.57=90度) -->
            <origin xyz="0 0 0" rpy="0 0 0" />
            <!-- material材料颜色: r=red g=green b=blue a=alpha -->
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>

```
注意，最底层子标签的格式为：
    
    <*** 属性=""/>
### 关节`joint`
 `joint` 标签用于描述机器人关节的运动学和动力学属性，还可以指定关节运动的安全极限，机器人的两个部件(分别称之为 `parent link` 与 `child link`)以"关节"的形式相连接，不同的关节有不同的运动形式: 旋转、滑动、固定、旋转速度、旋转角度限制....,比如:安装在底座上的轮子可以360度旋转，而摄像头则可能是完全固定在底座上。
#### 属性
- `name`关节名
- `type`关节运动形式
    - `continuous`: 旋转关节，可以绕单轴无限旋转
    - `revolute`: 旋转关节，类似于 continues,但是有旋转角度限制
    - `prismatic`: 滑动关节，沿某一轴线移动的关节，有位置极限
    - `planer`: 平面关节，允许在平面正交方向上平移或旋转
    - `floating`: 浮动关节，允许进行平移、旋转运动
    - `fixed`: 固定关节，不允许运动的特殊关节
#### 子标签(全部为最小标签)
- `parent`，属性`link`=父级连杆的名字（强制属性）
- `child`，属性`link`=子级连杆的名字（强制属性）
- `origin` 属性和`link`中的`origin`相同，表示两个 `link` 的物理中心之间的偏移量，`xyz`可以看为子连杆物理中心在父连杆物理中心坐标系下的坐标，`rpy`可以看为子连杆三个坐标轴分别绕父连杆三个坐标轴旋转的角度，单位为弧度
- `axis` 属性`xyz`设置绕哪个关节轴旋转，注意这里的关节轴坐标为`旋转轴在子连杆坐标系下的坐标表示`，例如`0 0 1`表示`子连杆绕自身的z轴旋转`，而不是父连杆的z轴
#### 示例：
```xml
<robot name="mycar">
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>
```
上述代码用于展示一个小车底盘，底盘上放置一个可以绕z轴360旋转的摄像头。
为了测试关节的可使用性，我们在`launch`文件中加入一些可以控制关节运动的节点：
```xml
<launch>

    <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find urdf_rviz)/urdf/testone.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz)/config/rviz/show_mycar.rviz"/>
    <!-- 添加关节状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <!-- 添加机器人状态发布节点 -->
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
    <!-- 可选:用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
</launch>
```
通过joint_state_publisher_gui的UI我们可以测试关节的旋转角度\
X轴为红色，Y轴为绿色，Z轴为蓝色
如下图：
![关节连接示例](pic/9.png)
#### 底盘问题
小车此时的底盘一半处于平面之下，这是因为在建立`base_link`时，`origin`标签中的`xyz`属性为0 0 0，这个指的一般为刚体的物理中心，所以这就会导致一半在平面之下，调整该属性即可：
```xml
<robot name="mycar">
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.125" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>
```
#### 另外的思路：（`autolaber`的方法）
将初始 `link` 设置为一个尺寸极小的`link`(比如半径为 0.001m 的球体，或边长为 0.001m 的立方体)，然后再在初始`link`上添加底盘等刚体，这样实现，虽然仍然存在初始`link`半沉的现象，但是基本可以忽略了。这个初始`link`一般称之为 `base_footprint`。\
这样做的两个优势：
1. 我们此时所有`link`的`origin`都可以设为`0`，其他都通过`joint`来设置具体的位置
2. `base_footprint`可以作为机器人的起始投影点，做到一个标识的作用
```xml
<robot name="mycar">
    <link name="base_footprint">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>
    <!-- 底盘 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.2 0.1" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <!-- 摄像头 -->
    <link name="camera">
        <visual>
            <geometry>
                <box size="0.02 0.05 0.05" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="red">
                <color rgba="1 0 0 0.5" />
            </material>
        </visual>
    </link>

    <joint name="baselink2foot" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 0.05" rpy="0 0 0" />
    </joint>

    <!-- 关节 -->
    <joint name="camera2baselink" type="continuous">
        <parent link="base_link"/>
        <child link="camera" />
        <!-- 需要计算两个 link 的物理中心之间的偏移量 -->
        <origin xyz="0.2 0 0.075" rpy="0 0 0" />
        <axis xyz="0 0 1" />
    </joint>

</robot>
```
打开后，修改`Fixed Frame(固定基准系)`为`base_footprint`，底盘就不再沉入地下，如下图：
![](pic/11.png)
#### 问题解决
但这样存在一个问题，我们用这个GUI调节角度之后，摄像头的`link转来转去`，并不是停在我们GUI设置的关节角度上，除非`link`的旋转角度为`center`中间值
原因分析：
首先关节角度跳来跳去，我们推测是关节接收到角度信息，不断在改变，我们使用rqt_graph对当前的话题信息可视化，有：
![话题图](pic/10.png)
很明显，现在`joint_state_publisher`和`joint_state_publisher_gui`都在向`robot_state_publisher`节点发布关节信息，很可能就是`joint_state_publisher`不断发布一个固定的关节角度，`joint_state_publisher_gui`不断发布一个我们设置的关节角度。
因此，我们修改`launch`文件：
```xml
<launch>

    <!-- 设置参数 -->
    <param name="robot_description" textfile="$(find urdf_rviz)/urdf/testone.urdf" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz)/config/rviz/show_mycar.rviz"/>
    <!-- 添加关节状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <!-- 用于控制关节运动的节点 -->
    <node pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" name="joint_state_publisher_gui" />
</launch>
```
重新尝试，问题解决。

## 两轮差速运动小车URDF建模
需求：创建一个四轮圆柱状机器人模型，机器人参数如下,底盘为圆柱状，半径 10cm，高 8cm，四轮由两个驱动轮和两个万向支撑轮组成，两个驱动轮半径为 3.25cm,轮胎宽度1.5cm，两个万向轮为球状，半径 0.75cm，底盘离地间距为 1.5cm(与万向轮直径一致)

```xml
<robot name="diffcar">
    <link name="base_footlink">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="yellow">
                <color rgba="0.8 0.3 0.1 0.5" />
            </material>   
        </visual>
    </link>

    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.08" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0325" length="0.015" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>
    
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.0325" length="0.015" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>

    <link name="forward_wheel">
        <visual>
            <geometry>
                <sphere radius="0.0075"  />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>

    <link name="back_wheel">
        <visual>
            <geometry>
                <sphere radius="0.0075"  />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>

    <joint name="base_link2base_footlink" type="fixed">
        <parent link="base_footlink" />
        <child link="base_link" />
        <origin xyz="0 0 0.015" rpy="0 0 0" />
    </joint>

    <joint name="left_wheel2base_link" type="continous">
        <parent link="base_link" />
        <child link="left_wheel" />
        <origin xyz="0 -0.1075 -0.0225" rpy="1.57 0 0" />
        <axis xyz="0 0 1" />
    </joint>

    <joint name="right_wheel2base_link" type="continous">
        <parent link="base_link" />
        <child link="right_wheel" />
        <origin xyz="0 0.1075 -0.0225" rpy="-1.57 0 0" />
        <axis xyz="0 0 1" />
    </joint>

    <joint name="forward_wheel2base_link" type="continous">
        <parent link="base_link" />
        <child link="forward_wheel" />
        <origin xyz="0.0925 0 -0.0475" rpy="0 0 0" />
        <axis xyz="1 1 1" />
    </joint>

    <joint name="back_wheel2base_link" type="continous">
        <parent link="base_link" />
        <child link="back_wheel" />
        <origin xyz="-0.0925 0 -0.0475" rpy="0 0 0" />
        <axis xyz="1 1 1" />
    </joint>

</robot>
```

## URDF仿真建模的缺点
1. 使用URDF仿真一方面需要对每一个部分进行实例化，即使是物理属属性完全相同的的，例如我们上面的例子中的两个驱动轮和两个万向轮
2. 在设计关节的位置时，需要按照一定的公式计算，公式是固定的，但是在 URDF 中依赖于人工计算，存在不便，容易计算失误，且当某些参数发生改变时，还需要重新计算。

## Xacro
### 什么是Xacro
Xacro 是 XML Macros 的缩写，Xacro 是一种 XML 宏语言，是可编程的 XML。Xacro 可以声明变量，可以通过数学运算求解，使用流程控制控制执行顺序，还可以通过类似函数的实现，封装固定的逻辑，将逻辑中需要的可变的数据以参数的方式暴露出去，从而提高代码复用率以及程序的安全性。\
较之于纯粹的 URDF 实现，可以编写更安全、精简、易读性更强的机器人模型文件，且可以提高编写效率。
### Xacro语法
#### 属性与公式编写
Xacro支持将数字封装为一个属性，类似于编程语言中的变量命名，利用该方式，我们无需对车轮的位置进行手动计算，只需要将基础参数设置好即可。
使用方法：
```xml
# 定义属性
    <xacro:property name="xxxx" value="yyyy" /> 
# 属性调用
    ${属性名称} 
# 属性计算
    ${数学表达式}
```
示例：
```xml
    <xacro:property name="qudong_wheel_r" value="0.1" /> 
    <xacro:property name="qudong_wheel_l" value="0.08" /> 

    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="${qudong_wheel_r}" length="${qudong_wheel_l}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="black">
                <color rgba="0.7 0.5 0 0.5" />
            </material>
        </visual>
    </link>  
```
#### 宏
类似于函数，将参数代入宏，可以快速建立link或joint，可以大大优化代码结构，提高代码安全性。
```xml
# 宏定义
<xacro:macro name="宏名称" params="参数列表(多参数之间使用空格分隔)">

    .....

    参数调用格式: ${参数名}

</xacro:macro>

# 宏调用
<xacro:宏名称 参数1=xxx 参数2=xxx/>
```
#### 文件包含
机器人由多部件组成，不同部件可能封装为单独的xacro文件，最后再将不同的文件集成，组合为完整机器人，可以使用文件包含实现
```xml
<robot name="xxx" xmlns:xacro="http://wiki.ros.org/xacro">
      <xacro:include filename="my_base.xacro" />
      <xacro:include filename="my_camera.xacro" />
      <xacro:include filename="my_laser.xacro" />
      ....
</robot>
```
#### 使用Xacro优化两轮差速小车底盘模型
```xml
<robot name="diff_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <!-- 封装变量、常量 -->
    <xacro:property name="PI" value="3.141" />
    <xacro:property name="wheel_radius" value="0.0325" /> 
    <xacro:property name="wheel_width" value="0.015" /> 
    <xacro:property name="base_radius" value="0.1" /> 
    <xacro:property name="base_linkheight" value="0.08" /> 
    <xacro:property name="caster_radius" value="0.0075" /> 
    <xacro:property name="base_height" value="0.015" /> 
    <!-- 初始化基座 -->
    <link name="base_footlink">
        <visual>
            <geometry>
                <sphere radius="0.001" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="yellow">
                <color rgba="0.8 0.3 0.1 0.5" />
            </material>   
        </visual>
    </link>
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="${base_radius}" length="${base_linkheight}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>
    <joint name="base_link2base_footlink" type="fixed">
        <parent link="base_footlink" />
        <child link="base_link" />
        <origin xyz="0 0 ${base_height}" rpy="0 0 0" />
    </joint>
    <!-- 驱动轮宏 -->
    <xacro:macro name="add_wheels" params="name flag">
        <link name="${name}_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_width}" />
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <material name="black">
                    <color rgba="0.7 0.5 0 0.5" />
                </material>
            </visual>
        </link> 
        <joint name="${name}_wheel2base_link" type="continuous">
            <parent link="base_link" />
            <child link="${name}_wheel" />
            <origin xyz="0 ${flag*(base_radius + wheel_width/2)} ${wheel_radius - base_height - base_linkheight/2}" rpy="${flag*PI/2} 0 0" />
            <axis xyz="0 0 1" />
        </joint>
    </xacro:macro>
    <!-- 万向轮宏 -->
    <xacro:macro name="add_casters" params="name flag">
        <link name="${name}_caster">
            <visual>
                <geometry>
                    <sphere radius="${caster_radius}"  />
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0" />
            </visual>
        </link> 
        <joint name="${name}_caster2base_link" type="continuous">
            <parent link="base_link" />
            <child link="${name}_caster" />
            <origin xyz="${flag*(base_radius-caster_radius)} 0 ${-caster_radius-base_linkheight/2}" rpy="0 0 0" />
            <axis xyz="1 1 1" />
        </joint>
    </xacro:macro> 
    <!-- 利用宏加入驱动轮和万向轮 -->
    <xacro:add_wheels name="left" flag="1" />
    <xacro:add_wheels name="right" flag="-1" />
    <xacro:add_casters name="front" flag="1" />
    <xacro:add_casters name="back" flag="-1" />
</robot>
```

#### 如何使用Xacro
我们可以将`Xacro`转为`urdf`后使用，但更推荐直接使用`Xacro`
确保当前功能包已经导入了`xacro`功能包之后，在launch中使用如下指令展示：
```xml
<launch>
    <!-- 设置参数 -->
   <param name="robot_description" command="$(find xacro)/xacro $(find urdf_rviz)/urdf/xacro/diffcar.urdf.xacro" />

    <!-- 启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find urdf_rviz)/config/rviz/show_mycar.rviz"/>
    <!-- 添加关节状态发布节点 -->
    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
    <node pkg="joint_state_publisher" type="joint_state_publisher" name="joint_state_publisher" />
   
</launch>

```
核心为：
```xml
<param name="robot_description" command="$(find xacro)/xacro $(find urdf_rviz)/urdf/xacro/diffcar.urdf.xacro" />
```
与urdf的展示代码对比：
```xml
<param name="robot_description" textfile="$(find urdf_rviz)/urdf/testone.urdf" />
```
`textfile`换为了`command`，且引号内多了寻找`xacro`功能包的部分

#### 增加Xacro功能包(加入功能包的一般步骤)
若没有包含`xacro`功能包\
只需要在`package.xml`中加入
```xml
<build_depend>xacro</build_depend>
<exec_depend>xacro</exec_depend>
```
在`CMakeLists.txt`中加入
```txt
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  urdf
  xacro ## 新增的包
)
```