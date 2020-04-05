#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <sensor_msgs/Imu.h>
#include <bitset>
#include <Eigen/Eigen>
using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变量声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state_uav0;
Eigen::Vector3d pos_drone_uav0;
Eigen::Vector3d vel_drone_uav0;

mavros_msgs::State current_state_uav1;
Eigen::Vector3d pos_drone_uav1;
Eigen::Vector3d vel_drone_uav1;

mavros_msgs::State current_state_uav2;
Eigen::Vector3d pos_drone_uav2;
Eigen::Vector3d vel_drone_uav2;

mavros_msgs::State current_state_uav3;
Eigen::Vector3d pos_drone_uav3;
Eigen::Vector3d vel_drone_uav3;

mavros_msgs::State current_state_uav4;
Eigen::Vector3d pos_drone_uav4;
Eigen::Vector3d vel_drone_uav4;

Eigen::Vector3d Takeoff_position_uav0 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav1 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav2 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav3 = Eigen::Vector3d(0.0,0.0,0.0);
Eigen::Vector3d Takeoff_position_uav4 = Eigen::Vector3d(0.0,0.0,0.0);

Eigen::Vector3d delta_uav0 = Eigen::Vector3d(2.0 , 0.0,  2.0 );
Eigen::Vector3d delta_uav1 = Eigen::Vector3d(4.0 , 0.0,  4.0 );
Eigen::Vector3d delta_uav2 = Eigen::Vector3d(6.0 , 0.0,  6.0 );
Eigen::Vector3d delta_uav3 = Eigen::Vector3d(8.0 , 0.0,  8.0 );
Eigen::Vector3d delta_uav4 = Eigen::Vector3d(10.0, 0.0,  10.0);

Eigen::Vector3d pos_sp(0, 0, 0);
double yaw_sp = 0;

mavros_msgs::PositionTarget pos_setpoint0;
mavros_msgs::PositionTarget pos_setpoint1;
mavros_msgs::PositionTarget pos_setpoint2;
mavros_msgs::PositionTarget pos_setpoint3;
mavros_msgs::PositionTarget pos_setpoint4;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

float get_time_in_sec(ros::Time begin);
void prinft_drone_state(float current_time);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void uav0_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav0 = *msg;
}

void uav0_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav0  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav0_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav0 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

void uav1_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav1 = *msg;
}

void uav1_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav1  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav1_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav1 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

void uav2_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav2 = *msg;
}

void uav2_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav2  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav2_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav2 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

void uav3_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav3 = *msg;
}

void uav3_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav3  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav3_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav3 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

void uav4_state_sub_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state_uav4 = *msg;
}

void uav4_position_sub_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone_uav4  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void uav4_velocity_sub_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    vel_drone_uav4 = Eigen::Vector3d(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_gcs_display");
    ros::NodeHandle nh("~");

    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, uav0_state_sub_cb);
    ros::Subscriber uav0_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 100, uav0_position_sub_cb);
    ros::Subscriber uav0_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/local_position/velocity_local", 100, uav0_velocity_sub_cb);
    ros::Publisher uav0_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav0/mavros/setpoint_raw/local", 10);
 
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, uav1_state_sub_cb);
    ros::Subscriber uav1_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 100, uav1_position_sub_cb);
    ros::Subscriber uav1_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav1/mavros/local_position/velocity_local", 100, uav1_velocity_sub_cb);
    ros::Publisher uav1_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 10);

    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>("/uav2/mavros/state", 10, uav2_state_sub_cb);
    ros::Subscriber uav2_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 100, uav2_position_sub_cb);
    ros::Subscriber uav2_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 100, uav2_velocity_sub_cb);
    ros::Publisher uav2_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav2/mavros/setpoint_raw/local", 10);

    ros::Subscriber uav3_state_sub = nh.subscribe<mavros_msgs::State>("/uav3/mavros/state", 10, uav3_state_sub_cb);
    ros::Subscriber uav3_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose", 100, uav3_position_sub_cb);
    ros::Subscriber uav3_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local", 100, uav3_velocity_sub_cb);
    ros::Publisher uav3_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav3/mavros/setpoint_raw/local", 10);

    ros::Subscriber uav4_state_sub = nh.subscribe<mavros_msgs::State>("/uav4/mavros/state", 10, uav4_state_sub_cb);
    ros::Subscriber uav4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav4/mavros/local_position/pose", 100, uav4_position_sub_cb);
    ros::Subscriber uav4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav4/mavros/local_position/velocity_local", 100, uav4_velocity_sub_cb);
    ros::Publisher uav4_setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/uav4/mavros/setpoint_raw/local", 10);


    ros::Rate rate(10.0);

    // int check_flag;
    // // 这一步是为了程序运行前检查一下参数是否正确
    // // 输入1,继续，其他，退出程序
    // cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
    // cin >> check_flag;

    // if(check_flag != 1)
    // {
    //     return -1;
    // }

    // 先读取一些飞控的数据
    int i =0;
    for(i=0;i<20;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }


    ros::Time begin_time = ros::Time::now();

    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        // 当前时间
        float cur_time = get_time_in_sec(begin_time);

        prinft_drone_state(cur_time);
        
        rate.sleep();
    }

    return 0;

}

// 【获取当前时间函数】 单位：秒
float get_time_in_sec(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

void prinft_drone_state(float current_time)
{

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV0 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav0.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav0.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav0.mode<<" ]   " <<endl;

    cout << "Position_uav0 [X Y Z] : " << pos_drone_uav0[0] << " [ m ] "<< pos_drone_uav0[1]<<" [ m ] "<<pos_drone_uav0[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav0 [X Y Z] : " << vel_drone_uav0[0] << " [m/s] "<< vel_drone_uav0[1]<<" [m/s] "<<vel_drone_uav0[2]<<" [m/s] "<<endl;
    cout<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV1 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav1.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav1.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav1.mode<<" ]   " <<endl;

    cout << "Position_uav1 [X Y Z] : " << pos_drone_uav1[0] << " [ m ] "<< pos_drone_uav1[1]<<" [ m ] "<<pos_drone_uav1[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav1 [X Y Z] : " << vel_drone_uav1[0] << " [m/s] "<< vel_drone_uav1[1]<<" [m/s] "<<vel_drone_uav1[2]<<" [m/s] "<<endl;
    cout<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV2 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav2.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav2.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav2.mode<<" ]   " <<endl;

    cout << "Position_uav2 [X Y Z] : " << pos_drone_uav2[0] << " [ m ] "<< pos_drone_uav2[1]<<" [ m ] "<<pos_drone_uav2[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav2 [X Y Z] : " << vel_drone_uav2[0] << " [m/s] "<< vel_drone_uav2[1]<<" [m/s] "<<vel_drone_uav2[2]<<" [m/s] "<<endl;
    cout<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV3 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav3.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav3.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav3.mode<<" ]   " <<endl;

    cout << "Position_uav3 [X Y Z] : " << pos_drone_uav3[0] << " [ m ] "<< pos_drone_uav3[1]<<" [ m ] "<<pos_drone_uav3[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav3 [X Y Z] : " << vel_drone_uav3[0] << " [m/s] "<< vel_drone_uav3[1]<<" [m/s] "<<vel_drone_uav3[2]<<" [m/s] "<<endl;
    cout<<endl;

    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>UAV4 State<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    cout << "Time: " << current_time <<" [s] ";

    //是否和飞控建立起连接
    if (current_state_uav4.connected == true)
    {
        cout << " [ Connected ]  ";
    }
    else
    {
        cout << " [ Unconnected ]  ";
    }

    //是否上锁
    if (current_state_uav4.armed == true)
    {
        cout << "  [ Armed ]   ";
    }
    else
    {
        cout << "  [ DisArmed ]   ";
    }

    cout << " [ " << current_state_uav4.mode<<" ]   " <<endl;

    cout << "Position_uav4 [X Y Z] : " << pos_drone_uav4[0] << " [ m ] "<< pos_drone_uav4[1]<<" [ m ] "<<pos_drone_uav4[2]<<" [ m ] "<<endl;
    cout << "Velocity_uav4 [X Y Z] : " << vel_drone_uav4[0] << " [m/s] "<< vel_drone_uav4[1]<<" [m/s] "<<vel_drone_uav4[2]<<" [m/s] "<<endl;
    cout<<endl;
}


