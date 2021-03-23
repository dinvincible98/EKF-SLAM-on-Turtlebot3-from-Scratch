/// \file  turtle_interface.cpp
/// \brief low-level control and sensor routine of turtlebot
///
/// Publisher:
///     + wheel_pub(nuturtlebot/WheelCommand): publish wheel velocity message
///     + joint_pub(sensor_msgs/JointState):  publish joint states message
/// Subscriber:
///     + cmd_sub(geometry_msgs/Twist): subcribe to cmd_vel topic 
///     + sensor_sub(nuturtlebot::SensorData): subscribe to sensor_data topic 

#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h>
#include<nuturtlebot/WheelCommands.h>
#include<nuturtlebot/SensorData.h>
#include<sensor_msgs/JointState.h>
#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>
#include<string>
#include<cmath>
#include<iostream>


// global variables
static rigid2d::Twist2D twist;                              // cmd_vel
static bool twist_flag;                                     // twist callback flag
static double left_cur,right_cur;                           // encoders reading
static bool sensor_flag;                                    // sensor callback flag



void cmdCallback(const geometry_msgs::Twist::ConstPtr & cmd)
{
    twist.vx = cmd->linear.x;
    twist.vy = 0.0;
    twist.w = cmd->angular.z;

    twist_flag = true;

}

void sensorCallback(const nuturtlebot::SensorData::ConstPtr & data)
{
    left_cur = data->left_encoder;
    right_cur = data->right_encoder;

    sensor_flag = true;
}






int main(int argc, char** argv)
{
    ros::init(argc, argv,"turtle_interface");
    ros::NodeHandle nh_("~");                                              // private nodehandler
    ros::NodeHandle nh;                                                    // public nodehandler
    
    ros::Subscriber cmd_sub = nh.subscribe("cmd_vel",10,cmdCallback);
    
    ros::Subscriber sensor_sub = nh.subscribe("sensor_data",10,sensorCallback);

    ros::Publisher wheel_pub = nh.advertise<nuturtlebot::WheelCommands>("wheel_cmd",10);

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);

    // parameters from turtlebot3 data sheet
    double max_rot = 2.84, max_trans = 0.22;
    double wheel_base = 0.16, wheel_radius = 0.033;
    double gear_ratio = 258.5;
    int rpm = 57;
    int resolution = 4096;
    std::string left_wheel_joint, right_wheel_joint;

    // load parameters
    nh.getParam("/max_rot", max_rot);
    nh.getParam("/max_trans", max_trans);
    nh.getParam("/wheel_base", wheel_base);
    nh.getParam("/wheel_radius",wheel_radius);
    nh.getParam("/left_wheel_joint",left_wheel_joint);
    nh.getParam("/right_wheel_joint",right_wheel_joint);

    ROS_INFO("max_rot: %f", max_rot);
    ROS_INFO("max trans: %f", max_trans);
    ROS_INFO("wheel_base: %f", wheel_base);
    ROS_INFO("wheel_radius: %f", wheel_radius);
    ROS_INFO("left_whee_joint: %s",left_wheel_joint.c_str());
    ROS_INFO("right_wheel_joint: %s",right_wheel_joint.c_str());

    ROS_INFO("Successfully launched the turtle_interface node");

    // initial pose(0,0,0)
    rigid2d::Pose2D pose;

    rigid2d::DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now();

    while (nh.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();
        last_time = cur_time;

        // receive cmd_vel callback
        if (twist_flag)
        {
            // cap wheel twist
            if (twist.w > max_rot)
            {
                twist.w = max_rot;
            }
            else if (twist.w < -max_rot)
            {
                twist.w = -max_rot; 
            }
            else
            {
                twist.w = twist.w;
            }

            if (twist.vx > max_trans)
            {
                twist.vx = max_trans;
            }
            else if (twist.vx <-max_trans)
            {
                twist.vx = -max_trans;
            }
            else
            {
                twist.vx = twist.vx;
            }


            rigid2d::WheelVelocities vels;
            vels = diffDrive.twistToWheels(twist);
             
            nuturtlebot::WheelCommands wheel_cmd;
            wheel_cmd.left_velocity = std::round(vels.u_left * gear_ratio / (2*rigid2d::PI * rpm/60));
            wheel_cmd.right_velocity = std::round(vels.u_right * gear_ratio / (2*rigid2d::PI * rpm/60));

            twist_flag = false;

            // publish wheel_cmd
            wheel_pub.publish(wheel_cmd);
        }

        // receive sensor_data callback 
        if (sensor_flag)
        {
            double left_angle, right_angle;
            left_angle = rigid2d::normalize_angle((2*rigid2d::PI*rpm / 60) * (left_cur/resolution));
            right_angle = rigid2d::normalize_angle((2*rigid2d::PI*rpm / 60) * (right_cur/resolution));
        
            // update odometery to get wheel velocity
            rigid2d::WheelVelocities vels;
            vels = diffDrive.updateOdom(left_angle,right_angle);

            sensor_msgs::JointState joint_state;
            
            joint_state.header.stamp = cur_time;

            joint_state.name.push_back(left_wheel_joint);
            joint_state.name.push_back(right_wheel_joint);

            joint_state.position.push_back(left_angle);
            joint_state.position.push_back(right_angle);

            joint_state.velocity.push_back(vels.u_left);
            joint_state.velocity.push_back(vels.u_right);

            sensor_flag = false;

            // publish joint_state
            joint_pub.publish(joint_state);
        }
    }
    return 0;
        
}



/// end file

