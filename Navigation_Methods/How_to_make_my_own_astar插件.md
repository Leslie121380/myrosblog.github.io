# 包含aStar算法的插件包my_astar_planner
## 文件结构：
```
~/ros2_ws/src/
├── robot_patrol/             # 你现有的包
│   ├── setup.py
│   └── robot_patrol/
│       └── navigator_node.py # 你的 Python 导航逻辑
└── my_astar_planner/         # 新建的包（按照之前的 C++ 教程做）
    ├── CMakeLists.txt
    ├── plugins.xml
    ├── include/
    └── src/
```


## 文件分工：
头文件 `my_astar_planner.hpp': 把我编写算法所需要的包#include进去，定义函数名（便于后续override) 

源文件 `my_astar_planner.cpp`：A*路径规划算法的真正所在处，我们自己写的类 MyAStarPlanner 实际上是继承自 nav2_core::GlobalPlanner，也正因为nav2是使用C++编写的所以我们的算法源文件也需要用C++编写。

CMakeLists : 编译与安装。定义怎么把 刚刚我编写的源C++ 代码变成库，以及安装到哪里。将源文件编译为一个动态库(.so)，告诉其他组分这个编译后的动态库叫什么名字，它被安放到了哪里，它依赖于哪些其他包（Naz2的一些包msg等）
```
add_library(my_astar_planner_lib SHARED
  src/my_astar_planner.cpp
)
target_include_directories(my_astar_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
```

package.xml: 依赖声明。告诉系统这个插件包的编写依赖哪些组件，比如 nav2_core 等。

plugins.xml：当外界需要找这个插件包的时候应该用什么名字去寻找。`<library path="my_lib"> (必须与 CMake 里的库名一致)`；`<class name="..."> (你在 YAML 里写的类型名)`

🫏tip1: plugin.xml文件需要手动创建，package.xml文件和CMakeLists文件是运行`ros2 pkg create --build-type ament_cmake my_astar_planner`系统会自动创建的。但是package.xml 中你需要手动添加依赖项，并声明插件导出。在 CMakeLists.txt 中：
你需要手动添加编译指令，特别是这三部分：

~寻找依赖：find_package(nav2_core REQUIRED) 等。

~生成库：add_library(my_astar_planner_lib SHARED src/my_astar_planner.cpp)。

~插件注册：CMake将 plugins.xml 注册到 ament 资源索引中
`pluginlib_export_plugin_description_file(nav2_core plugins.xml)`

安装路径：手动写 install(...) 指令，否则编译出的 .so 文件会留在 build 文件夹里，而不会去 install 文件夹。

🫏tip2: VScode的C/C++编译器会不知道去哪里找ROS2的头文件，因此第一次编写cpp代码的时候会出现红色波浪线，用如下操作解决：
你需要告诉 VS Code 去哪里找 ROS 2 的头文件：

在 VS Code 中按 Ctrl + Shift + P。

输入并选择 C/C++: Edit Configurations (UI)。

在 Include path 栏目中添加两行：
```
${workspaceFolder}/**
/opt/ros/${env:ROS_DISTRO}/include/**
```












# 调用aStar算法的机器人包robot_simulation
## 文件结构

nav2_param.yaml: 这个地方其实就是我在Where_to_put_Algorithm的文章里面提到的planner_plugins响应后寻找路径规划算法的地方，这里的planner_server中的内容应该对应修改为刚才的aStar算法的命名空间/类名。
其余文件的作用和刚才提到的类似，遂略。

### 插件配置的对齐方式

库名对齐：CMakeLists.txt 的 add_library(X) == plugins.xml 的 path="X"。

类名对齐：plugins.xml 的 class name="Y" == nav2_params.yaml 的 plugin: "Y"

代码对齐：plugins.xml 的 type="Z" == C++ 代码里的 my_namespace::MyClass


# 关于.yaml文件
```
YAML 文件就是机器人的“说明书”和“调参面板”。

在 ROS 2 中，如果你把 C++ 代码比作机器人的大脑逻辑，那么 YAML 文件就是大脑里的记忆参数。

1. YAML 文件是用来干什么的？
它的核心作用是实现“逻辑”与“参数”的分离。

无需重新编译：如果没有 YAML，你修改一个 A* 算法的搜索步长，就得改 C++ 源码并重新花几分钟 colcon build。有了 YAML，你只需要改个数字，重新启动（Restart）一下，机器人就变样了。

动态开关：你可以通过 YAML 随时切换插件（比如从 A* 切换到 Dijkstra），而不需要动一行代码。

集中管理：Nav2 这种大型系统有成百上千个参数（膨胀半径、最大速度、容错距离等），全部堆在代码里是灾难，放在 YAML 里则一目了然。

2. 什么时候需要 YAML 文件？
当你遇到以下场景时，你就必须请出 YAML 文件了：

A. 配置复杂的中间件（如 Nav2, MoveIt）
像 Nav2 这种官方提供的包，其底层逻辑已经写死了，它留给你的唯一交互接口就是参数。你想让它加载你的插件、改变地图分辨率、调整避障距离，都必须写在 YAML 里。

B. 你自己的代码需要“调参”时
如果你写了一个巡逻节点，你不想把“巡逻速度”硬编码为 0.5 m/s，你可以把它写成参数。

代码里： this->declare_parameter("speed", 0.2);

YAML 里： speed: 1.0 这样你在不同环境下测试时，只需换个 YAML 即可。

C. 启动多个相似的节点时
如果你有两个激光雷达，代码是一样的，但它们挂载的串口号不同（/dev/ttyUSB0 和 /dev/ttyUSB1）。你可以准备两个 YAML 文件，在 Launch 启动时分别传给它们。

3. YAML 的数据流向图
这是你刚才经历的过程：

编写阶段：你在 src 文件夹下创建 .yaml 文件，写下 plugin: "my_astar..."。

安装阶段：通过 setup.py 或 CMakeLists.txt，将它搬运到 install 文件夹。

启动阶段：Launch 文件读取这个路径，并把参数“喂”给节点。

运行阶段：节点内部通过 get_parameter() 获取到你的配置，从而成功加载了插件。

```














