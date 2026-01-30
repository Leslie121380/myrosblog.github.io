# ROS2环境中代码的编译和运行顺序

*`colon build`是ROS2使用的总编译代码，它并不负责编译只是负责扫描/src下面的每个文件，根据内容调度工具和包的使用。*

step1: `colcon`读取所有包中的package.xml获取所有包之间的依赖关系，并对依赖关系进行拓扑排序确保编译顺序（A依赖B则选择先编译B）

step2: 给每个包在/build目录中创建独立的文件夹。这里我们可以注意到，使用C++（CMakeList）作为脚本的包会生成一个cmake文件夹其中有一个Makefile文件，负责存放C++编译过程中产生的临时对象文件(.o);而使用python（setup)作为脚本的包则只有config和share文件夹，负责给python解释器告诉CPU应该执行什么动作


😈这里可以看出C++和python在底层逻辑和范式上的区别😈：
```
C++ (编译型)：

过程：colcon 调用 CMake，将人类可读的 .cpp 源代码通过编译器（GCC/Clang）彻底瓦解，重新构建为机器指令（二进制 .so 或可执行文件）。

范式：“结果导向”。运行时的CPU并不认识你的代码，它只执行二进制指令。

性能：因为所有的逻辑在编译阶段就已经优化完毕，这个是编译器负责的工作，编译的时候就会发现代码错误并修正，最后底层的运行不浪费运行时的每一秒。

Python (解释型/脚本型)：

过程：colcon 并不“编译” Python。它只是在 install 文件夹里建立了一套路径索引，或者拷贝了源码。

范式：“过程导向”。运行时，Python 解释器（Interpreter）会逐行读取你的 .py 文件，现场翻译给 CPU 听。

性能：可以随时改一行代码并立即运行，但代价是运行时开销大，因为运行的时候是CPU听解释器一条条读指令一条条找包，所以很多错误是运行的时候才发现的。

In conclusion:

ROS2中，“底层 C++，顶层 Python” ：

如果你要写算法、驱动、实时控制：请开启 C++ 的 OOP 模式，严谨地定义类和接口。

如果你要写测试脚本、UI 界面、状态机调度：开启 Python 的过程化或轻量 OOP 模式，追求效率和快速迭代

```
step3: C++调用 make 或 ninja。编译器将 .cpp 源代码变成机器码;Python：运行 setuptools。根据setup.py文件和package.xml文件检查依赖关系，整理脚本。

step4: 被显示指定的文件被拷贝到/install目录下，比如：可执行文件：编译好的二进制程序；库文件：.so 动态链接库（比如你的 A* 插件）；资源文件：你在 CMakeLists.txt 的 install() 或 setup.py 的 data_files 中指定的 YAML、XML、Launch 文件。

💮aStar插件的完整流程（C++)💮：
```
命令：colcon build --packages-select my_astar_planner。

CMake 介入：读取 CMakeLists.txt，发现你要把 my_astar_planner.cpp 编译成名为 my_astar_planner 的库。

编译：在 build/my_astar_planner 下生成了 libmy_astar_planner.so。

安装：因为你写了 install(TARGETS...)，这个 .so 文件被拷贝到了 install/my_astar_planner/lib/。

插件注册：因为你写了 pluginlib_export...，一个索引文件被放进了 install/my_astar_planner/share/ament_index/...。

最后：Nav2 运行脚本，通过索引找到了这个 .so 文件，成功加载
```

⁉️不要修改`opt/ros/humble/'下的文件！！！⁉️
```
第一次做插件开发时最容易踩的坑。/opt/ros/ 是系统只读目录（或者说受保护目录），即便你用 sudo 修改了它，由于 ROS 2 的启动机制和环境变量的加载顺序，你启动时运行的往往还是原始版本，或者你的修改在系统更新时会被覆盖。
更重要的是，ROS 2 的路径优先级遵循：工作空间 (install/) > 系统路径 (/opt/ros/)，所以应该去/src目录下对源文件做出修改，保证代码编写环境(/src)和运行环境（/install)的隔离干净，因为install空间是执行colcon的时候系统根据/src文件内容自行建立的。
不是在 install 文件夹下直接维护，而是在 src 文件夹下修改，通过 colcon build 同步到 install 文件夹。如果你直接修改 install 里的文件，下次你运行 colcon build 时，你的修改会被 src 里的旧内容彻底覆盖。
```

























