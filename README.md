#fomation_simulation

#v1.0.0

该项目基于px4源码所搭建五架四旋翼无人机的gazebo仿真，模拟编写了地面站与各架无人机的通信与命令控制程序框架；目前仅实现五架无人机的一键起飞，功能待完善...

#使用说明：

    1. 将launch文件中的five_uav_mavros_sitl.launch拷贝到px4源码的launch文件中
<<<<<<< HEAD

    2. 将下列环境变量添加到你的.bashrc，替换为你的px4源码路径，确保roslaunch px4 five_uav_mavros_sitl.launch

        source ~/Documents/pixhawk/src/Firmware/Tools/setup_gazebo.bash ~/Documents/pixhawk/src/Firmware/ ~/Documents/pixhawk/src/Firmware/build/px4_sitl_default
=======
    
    2. 将下列环境变量添加到你的.bashrc，确保roslaunch px4 five_uav_mavros_sitl.launch
    
        source ~/Documents/pixhawk/src/Firmware/Tools/setup_gazebo.bash ~/Documents/pixhawk/src/Firmware/                 ~/Documents/pixhawk/src/Firmware/build/px4_sitl_default
>>>>>>> 51951e0f7746bbd5b1d17169495c080da85921ac
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Documents/pixhawk/src/Firmware
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Documents/pixhawk/src/Firmware/Tools/sitl_gazebo

    3. catkin_make程序所在的工作空间
<<<<<<< HEAD

    4. 运行sitl_gazebo_formation.sh脚本

=======
    
    4. 运行sitl_gazebo_formation.sh脚本
    
>>>>>>> 51951e0f7746bbd5b1d17169495c080da85921ac
    5. 在地面控制端进入offboard模式后，解锁实现一键起飞
    


<<<<<<< HEAD

    
=======
    
>>>>>>> 51951e0f7746bbd5b1d17169495c080da85921ac
