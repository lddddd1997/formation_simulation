
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
#include "formation_simulation/communication.h"
#include <bitset>
#include <Eigen/Eigen>
using namespace std;

formation_simulation::communication other_uav;

void Uav0PositionSubscriberCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    other_uav.position[0].x  = msg->pose.position.x;
    other_uav.position[0].y  = msg->pose.position.y;
    other_uav.position[0].z  = msg->pose.position.z;
}

void Uav0VelocitySubscriberCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    other_uav.velocity[0].x = msg->twist.linear.x;
    other_uav.velocity[0].y = msg->twist.linear.y;
    other_uav.velocity[0].z = msg->twist.linear.z;
}

void Uav2PositionSubscriberCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    other_uav.position[1].x  = msg->pose.position.x;
    other_uav.position[1].y  = msg->pose.position.y;
    other_uav.position[1].z  = msg->pose.position.z;
}

void Uav2VelocitySubscriberCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    other_uav.velocity[1].x = msg->twist.linear.x;
    other_uav.velocity[1].y = msg->twist.linear.y;
    other_uav.velocity[1].z = msg->twist.linear.z;
}

void Uav3PositionSubscriberCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    other_uav.position[2].x  = msg->pose.position.x;
    other_uav.position[2].y  = msg->pose.position.y;
    other_uav.position[2].z  = msg->pose.position.z;
}

void Uav3VelocitySubscriberCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    other_uav.velocity[2].x = msg->twist.linear.x;
    other_uav.velocity[2].y = msg->twist.linear.y;
    other_uav.velocity[2].z = msg->twist.linear.z;
}

void Uav4PositionSubscriberCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    other_uav.position[3].x  = msg->pose.position.x;
    other_uav.position[3].y  = msg->pose.position.y;
    other_uav.position[3].z  = msg->pose.position.z;
}

void Uav4VelocitySubscriberCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    other_uav.velocity[3].x = msg->twist.linear.x;
    other_uav.velocity[3].y = msg->twist.linear.y;
    other_uav.velocity[3].z = msg->twist.linear.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_uav1_communication");
    ros::NodeHandle nh("~");

    ros::Subscriber uav0_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 100, Uav0PositionSubscriberCallback);
    ros::Subscriber uav0_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/local_position/velocity_local", 100, Uav0VelocitySubscriberCallback);

    ros::Subscriber uav2_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav2/mavros/local_position/pose", 100, Uav2PositionSubscriberCallback);
    ros::Subscriber uav2_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav2/mavros/local_position/velocity_local", 100, Uav2VelocitySubscriberCallback);
 
    ros::Subscriber uav3_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav3/mavros/local_position/pose", 100, Uav3PositionSubscriberCallback);
    ros::Subscriber uav3_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav3/mavros/local_position/velocity_local", 100, Uav3VelocitySubscriberCallback);
 
    ros::Subscriber uav4_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/uav4/mavros/local_position/pose", 100, Uav4PositionSubscriberCallback);
    ros::Subscriber uav4_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/uav4/mavros/local_position/velocity_local", 100, Uav4VelocitySubscriberCallback);

    ros::Publisher commnication_pub = nh.advertise<formation_simulation::communication>("/uav1_sim/uav1_communication/pos_vel/local", 10);

    ros::Rate rate(10.0);

    // 先读取一些飞控的数据
    int i = 0;
    for(i = 0; i < 20; i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ros::Time begin_time = ros::Time::now();
    cout<<"uav1 communication node started!!!"<<endl;
    while(ros::ok())
    {
        //执行回调函数
        ros::spinOnce();

        commnication_pub.publish(other_uav);

        rate.sleep();
    }
    return 0;
}

