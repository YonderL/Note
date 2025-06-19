# 阿克曼底盘的URDF建模与Gazebo控制（使用Xacro优化）

## 阿克曼底盘建模
### 建模
我们使用后轮驱动，前轮转向的阿克曼底盘模型。\
那么对于后轮只需进行正常的`continous joint`连接即可\
对于前轮，有两个自由度，旋转和转向，而且由于`URDF`无法进行闭链关节建模，因此我们模拟一个转向杆，使用`revolute joint`限定转向角度首先将`base_link`与转向杆连接，之后使用`continous joint`将转向杆与车轮相连。\
如下为具体实现：
```xml
<robot name="akeman_base" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="PI" value="3.1415926" />
    <xacro:property name="wheel_radius" value="0.03" />
    <xacro:property name="wheel_height" value="0.02" />
    <xacro:property name="wheel_in" value="0.0125" />
    <xacro:property name="base_linkheight" value="0.075" />
    <xacro:property name="base_length" value="0.225" />
    <xacro:property name="base_width" value="0.2" />
    <xacro:property name="base_height" value="0.01" />

    <link name="base_footprint">
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
                <box size="${base_length} ${base_width} ${base_linkheight}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="blue">
                <color rgba="0 0 1.0 0.5" />
            </material>
        </visual>
    </link>

    <joint name="base_link2base_footlink" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 ${base_height}" rpy="0 0 0" />
    </joint>

    <xacro:macro name="define_rear_wheel" params="name flag">
        <link name="${name}_rear_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_height}" />
                </geometry>
                <origin xyz="0 0 0" rpy="${flag*PI/2} 0 0" />
                <material name="yellow">
                    <color rgba="0.8 0.3 0.1 0.5" />
                </material>
            </visual>
        </link>
        <joint name="${name}_rear_wheel2base_link" type="continuous">
            <parent link="base_link" />
            <child link="${name}_rear_wheel" />
            <origin xyz="${-(-wheel_in-wheel_radius+base_length/2)} ${flag*(wheel_height/2 + base_width/2)} ${-(base_linkheight/2+base_height-wheel_radius)}" />
            <axis xyz="0 1 0" />
        </joint>
    </xacro:macro>
    <xacro:define_rear_wheel name="left" flag="1" />
    <xacro:define_rear_wheel name="right" flag="-1" />

    <xacro:macro name="define_front_wheel" params="name flag">
        <link name="${name}_front_wheel">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_height}" />
                </geometry>
                <origin xyz="0 0 0" rpy="${flag*PI/2} 0 0" />
                <material name="yellow">
                    <color rgba="0.8 0.3 0.1 0.5" />
                </material>
            </visual>
        </link>
        <link name="${name}_steer_front_joint">
            <visual>
                <geometry>
                    <box size="0.01 0.01 0.01" />
                </geometry>
                <origin xyz="0 0 0" rpy="0 0 0" />
                <material name="black">
                    <color rgba="0.7 0.5 0 0.5" />
                </material>
            </visual>
        </link>

        <joint name="${name}_steer_front_joint2base_link" type="revolute">
            <parent link="base_link" />
            <child link="${name}_steer_front_joint" />
            <origin xyz="${-wheel_in-wheel_radius+base_length/2} ${flag*(wheel_height/2 + base_width/2)} ${0.01 + base_linkheight/2}" />
            <axis xyz="0 0 1" />
            <limit lower="-0.69" upper="0.69" effort="0" velocity="3"/>
        </joint>

        <joint name="${name}_front_wheel2${name}_steer_front_joint" type="continuous">
            <parent link="${name}_steer_front_joint" />
            <child link="${name}_front_wheel" />
            <origin xyz="0 0 ${-(0.01+base_linkheight+base_height-wheel_radius)}" />
            <axis xyz="0 1 0" />
        </joint>
    </xacro:macro>
    <xacro:define_front_wheel name="left" flag="1" />
    <xacro:define_front_wheel name="right" flag="-1" />

</robot>
```
### 效果
如下图，可以看到前轮可以通过`GUI`旋转`steer_link`实现转向
![效果图](pic/15.png)

## URDF集成Gazebo
较之于 rviz，gazebo在集成 URDF 时，需要做些许修改：
1. 必须添加 collision 碰撞属性相关参数
2. 必须添加 inertial 惯性矩阵相关参数
3. Gazebo颜色设置也必须做相应的变更
### collision 碰撞属性
如果机器人link是标准的几何体形状，和link的visual属性设置一致即可,但没有颜色属性
### inertial 惯性矩阵
对于标准几何体的惯性矩阵，有标准公式，因此我们可以使用一个专门的Xacro文件利用宏封装这些标准几何体的惯性矩阵的公式，之后在其他Xacro文件中包含并调用即可;
```xml
<robot name="base" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- Macro for inertia matrix 球体 圆柱体 立方体-->
    <xacro:macro name="sphere_inertial_matrix" params="m r">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                iyy="${2*m*r*r/5}" iyz="0" 
                izz="${2*m*r*r/5}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
        </inertial>
    </xacro:macro>

    <xacro:macro name="Box_inertial_matrix" params="m l w h">
       <inertial>
               <mass value="${m}" />
               <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                   iyy="${m*(w*w + l*l)/12}" iyz= "0"
                   izz="${m*(w*w + h*h)/12}" />
       </inertial>
   </xacro:macro>
</robot>
```
### 颜色设置
在Gazebo中显示link的颜色，必须指定标签，使用如下指令，该指令与link平级
```xml
<gazebo reference="link节点名称">
     <material>Gazebo/Blue</material>
</gazebo>
```

### URDF在Gazebo下可视化
```xml
<launch>
    <!-- 将 Urdf 文件的内容加载到参数服务器 -->
    <param name="robot_description" command="$(find xacro)/xacro $(find urdf_rviz)/urdf/xacro/akeman_base.xacro" />
    <!-- 启动 gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch" />

    <!-- 在 gazebo 中显示机器人模型 -->
    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model akemanbase -param robot_description"  />
</launch>
```