# ROS更改功能包名称

1. 更改功能包文件夹名称
   
2. 修改package.xml,将其中所有的旧功能包名称改为新的名称,具体在于：
```xml
<name>turn_on_car</name>
<description>The turn_on_car package</description>
```

3. 修改CMakeLists.txt，将其中所有的旧功能包名称改为新的名称，具体在于：
```xml
project(urdf_rviz)
```

4. 修改其他文件中的旧名称为新名称，例如launch文件等

5. 重新编译