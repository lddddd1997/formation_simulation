fomation_simulation

v1.0.0

该项目基于px4源码所搭建五架四旋翼无人机的gazebo仿真，模拟编写了地面站与各架无人机的通信与命令控制程序框架；目前仅实现五架无人机的一键起飞，功能待完善...

使用说明：

    1. 将launch文件中的five_uav_mavros_sitl.launch拷贝到px4源码的launch文件中

    2. 将下列境变量添加到你的.bashrc，替换为你的px4源码路径，确保roslaunch px4 five_uav_mavros_sitl.launch

        source ~/Documents/pixhawk/src/Firmware/Tools/setup_gazebo.bash ~/Documents/pixhawk/src/Firmware/ ~/Documents/pixhawk/src/Firmware/build/px4_sitl_default
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Documents/pixhawk/src/Firmware
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Documents/pixhawk/src/Firmware/Tools/sitl_gazebo

    3. catkin_make程序所在的工作空间

    4. 运行sitl_gazebo_formation.sh脚本
    
    5. 在地面控制端进入offboard模式后，解锁实现一键起飞
    
***该仓库不再更新，请移步到新仓库[px4_application](https://github.com/lddddd1997/px4_application)***
