# Write something about my ROS2 project for the first time 2026/01/20
  创建ROS2工作空间，使用Gazebo，Rviz2和Turtlebot3联合仿真。
  从Turtlebot3中导入机器人模型（burger)和地图，机器人模型本质是URDF文件，使用Rviz2可视化，地图是默认地图在Gazebo中可视化
## Step1:建立工作空间
      mkdir -p ~/minicar_ws/src
## step2:创建功能包
      ros2 pkg create --build-type ament_python
      注意这里ament后面用python的话就是使用setup.py作为运行脚本，如果使用cmake的话就是使用CMakeLists.txt作为运行脚本，运行脚本的主要功能大约就是把需要运行的文件列表排序，说明接口，定义文件的搬运逻辑。以python为例，这个代码运行完之后会自动生成setup文件，同时引进ros2空间的基本包（生成package.xml，是包的清单文件），生成存放源码的文件夹（demo）等
## step3:建立小车模型/3D地图模型/小车运动模型——编写launch文件
      这个小车模型和地图模型可以从Turtlebot3里面直接导入，但是要记得在launch中把包一个个放进去（launch的主要作用就是明确包的来路，其中包括自己的包（node)和Turtlebot3的自带包，具体操作如下：
       # The model of car is 'burger' from turtlebot3 
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    
    #The map and the move of car also from turtlebot3 
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_bringup = get_package_share_directory('turtlebot3_bringup')

    #initial Gazebo and put map(now in pkg_gazebo) in it
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    #initial Rviz2 and put car_move(now in pkg_bringup) in it
    start_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'rviz2.launch.py')
        )
    )
    #walk 
    run_walker = Node(
        package='my_turtle_demo',
        executable='walker_node', 
        output='screen'
    )

    return LaunchDescription([
        set_model,
        start_gazebo,
        start_rviz,
        run_walker
    ])
  ## step4:配置好setup.py文件
        刚才自动生成的setup.py文件里面只有ros的基本文件，要加入launch文件和我们自己写的node文件（entry_points)里面
  ## step5:运行
        cd ~/turtle_ws
        # 1. 编译
        colcon build --symlink-install
        # 2. 刷新环境,🏁注意，这一步实际上是要告诉现在这个bash该去哪里找文件，刚刚编译完的文件会自己放在install文件夹中，所以要source这个位置，所以每次编译之后都要重新source
       source install/setup.bash
        # 3. 启动地图
       ros2 launch my_turtle_demo my_turtle.launch.py————这个是打开gazebo环境和Rivz2
       Gazebo 提供物理世界（房子/地图），Rviz2 提供视觉监测，而功能包launch.py文件负责把小车放进去并控    制它
        # 4. 启动walker让小车自己动（新开终端启动walker节点)
        source ~/turtle_ws/install/setup.bash
        ros2 run my_turtle_demo walker_node

        
## Tips🩹:
### ~ 1. 每次编译完之后都要s先ource一下install/setup,再用ros run指令。source的本质就是告诉bash和环境去哪里找文件，ROS2环境中的系统编译完的文件会自动存放到install文件夹中，最终运行程序的时候ROS会在工作空间中的install里面搜寻，里面有功能包和python脚本。
### ~ 2. 最好不要用snap安装，它会把环境变量改成它自己，导致路径改成每次要找的时候都通过snap，但是其实snap很多包都是旧版的，最好官方安装
### ~ 3. ROS1使用的是catkin指令，ROS2使用的是colcon指令对项目进行编译
### ~ 4. 基本文件目录结构
  <img width="1382" height="974" alt="Tree" src="https://github.com/user-attachments/assets/00b77feb-340d-436e-a5fc-28987cff95af" />

        
        5. 编译出现问题之后可以先手动清楚刚刚编译出来的install和build和log文件，在重新编译以免出现不必要的冲突，使用rm -rf /build/install/log/
        
        
        

        
      
