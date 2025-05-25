# `CmakeList.txt`
   1. `cmake_minimum_required(VERSION 3.0.2) `\
      所需 `cmake` 版本
   2. `project(demo01_hello_vscode) `\
      包名称，会被 `${PROJECT_NAME}` 的方式调用
   3. `find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)`\
      设置构建所需要的软件包
   4. `add_executable(Hello_VSCode src/Hello_VSCode.cpp)`\
      声明 C++ 可执行文件\
      参数1为节点名字，可以随意设？注意区分\
      `建议直接设为跟程序名一样，因为这个名字涉及到rosrun的功能包中的功能！还有roslaunch`
      参数2为对应节点的程序相对地址
   5. `add_dependencies(Hello_VSCode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})`\
      添加可执行文件的 cmake 目标依赖
   6. `target_link_libraries(Hello_VSCode ${catkin_LIBRARIES})`\
      指定库、可执行文件的链接库\
      参数1为第5步设定的节点名字
   7. `catkin_install_python(PROGRAMS scripts/Hi.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})`\
      设置用于安装的可执行脚本

*5和6是每次构建功能包必须使用的*