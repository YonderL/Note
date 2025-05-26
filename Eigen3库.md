### 公式无法在github网页正常显示，查看[Html文件](Eigen3库.html)
# `Eigen`库

矩阵运算是一种非常重要的运算方式，在`Matlab`中，矩阵运算可以轻松的实现，但在`C++`这种偏底层的语言中，若不借助第三方库，矩阵运算需要我们进行较为复杂的代码设计。`Eigen`库是一个用于线性运算的`C++`模板库，它支持矩阵运算、矢量运算、数值分析以及相关的算法，安装`Eigen`库可以大大提高我们的开发效率。

## 安装与配置`Eigen`库
### 安装
在ubuntu系统中，安装`Eigen`库较为简单，只需在命令行中运行

    sudo apt-get install libeigen3-dev
之后，我们可以在系统中找到`/usr/include/eigen3`，这证明我们安装成功

### 在`Vscode`配置
使用`Vscode`可以很方便地配置`Eigen`库的编译，在`c_cpp_properties.json`的`includepath`中添加`"/usr/include/**"`即可，如果已经有了那就无需再添加。
```json
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "/opt/ros/noetic/include/**",
        "/usr/include/**",
        "/home/aliang/vehicle_sim/devel/include/**"
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++17"
    }
  ],
  "version": 4
}
```
### 与`ROS`中配置
在`ROS`中，我们还需在功能包的`CMakeList`中添加：

    include_directories("/usr/include/eigen3")
这样我们就可以在程序中用这种方法引入包：
```C++
    #include <Eigen/Eigen>
    #include <Eigen/Dense>
    #include <Eigen/Geometry>
    #include <Eigen/Eigenvalues>
```
否则，需要使用这种方法引入包：
```C++
    #include <eigen3/Eigen/Eigen>
    #include <eigen3/Eigen/Dense>
    #include <eigen3/Eigen/Geometry>
    #include <eigen3/Eigen/Eigenvalues>
```
### 验证是否可以正常使用
在`ROS`的功能包下新建一个`cpp`文件
```C++
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

int main(int argc, char *argv[]){
    // here is an arbitrary normal vector
// initialized to (1,2,3) upon instantiation
    Eigen::Vector3d normal_vec(1,2,3); 

    return 0;
}
```
在`CMakeList`中加入
```CMake
add_executable(vec_compute src/compute.cpp)
target_link_libraries(vec_compute ${catkin_LIBRARIES})
```
编译通过，证明可以正常使用
## 基于`Eigen`库的RBF径向基核函数的二维曲线拟合

### RBF径向基核函数的二维曲线拟合
`RBF径向基核函数`的二维曲线拟合的核心在于，根据$N$个拟合点求解出$N$个径向基核函数（$N$个径向基核函数分别对应$N$个拟合点）的加权数。对于目标二维曲线$F(x,y)=1$即优化下列方程：
$$
\sum_{k=1}^N w_k*f(r) = 1
$$
展开为向量形式：
$$
\begin{pmatrix}
f(||q-q_1||) & f(||q-q_2||) & \cdots & f(||q-q_N||)
\end{pmatrix}*
\begin{pmatrix}
w_1\\
w_2\\
\vdots\\
w_N
\end{pmatrix}=1
$$
其中$q_N$表示被拟合的$N$个点，由于每个被拟合点都满足该公式，替换掉$q$，可以再次展开为：
$$
\begin{pmatrix}
f(||q_1-q_1||) & f(||q_1-q_2||) & \cdots & f(||q_1-q_N||)\\
f(||q_2-q_1||) & f(||q_2-q_2||) & \cdots & f(||q_2-q_N||)\\
\vdots & \vdots & \ddots & \vdots\\
f(||q_N-q_1||) & f(||q_N-q_2||) & \cdots & f(||q_N-q_N||)\\
\end{pmatrix}*
\begin{pmatrix}
w_1\\
w_2\\
\vdots\\
w_N
\end{pmatrix}=1
$$
左边的矩阵可以看为一个Gram相关矩阵形式，则可以求出我们想要的$\vec{w}$：

$$
\vec{w} = GramMatrix^{-1}_{N \times N}*1_{N \times 1}
$$

选取径向基核函数为$f(r)=r^2ln(r+1)$
### 基于`Eigen`库的实现
```C++
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <cmath>

int main(int argc, char *argv[]){
    int num = 6;
    Eigen::MatrixXd p_x(1, num);
    Eigen::MatrixXd dis_x(num, num);
    Eigen::MatrixXd p_y(1, num);
    Eigen::MatrixXd dis_y(num, num);
    Eigen::MatrixXd dis_sqr(num, num);
    Eigen::MatrixXd dis_r(num, num);
    Eigen::MatrixXd w(num, 1);
    p_x << 1.5, 1.5, -0.75, -3, -0.75, 1.5;
    p_y << 0, 2.6, 1.3, 0, -1.3, -2.6;
    Eigen::MatrixXd mat_x = p_x.replicate(num, 1); 
    // 该函数将p_x作为一个矩阵元素，填充出n*m的矩阵
    Eigen::MatrixXd mat_y = p_y.replicate(num, 1);
    mat_x = mat_x - p_x.transpose().replicate(1, num);
    mat_y = mat_y - p_y.transpose().replicate(1, num);
    dis_x = mat_x.cwiseProduct(mat_x); //对应位置相乘
    dis_y = mat_y.cwiseProduct(mat_y);
    dis_sqr = dis_x + dis_y;
    dis_r = dis_sqr.unaryExpr([](double x){return std::sqrt(x);});
    std::cout << "矩阵：\n" << dis_r << std::endl;
    dis_r = dis_r.unaryExpr([](double x){return x*x*log(x+1);});
    dis_r = dis_r.inverse();
    w = dis_r.rowwise().sum();
    std::cout << "矩阵：\n" << w << std::endl;
    return 0;
}
```
与`Matlab`计算结果相同

    w=-0.0475269
       0.0351389
      -0.0475205
       0.0351453
      -0.0475205
       0.0351389

