/// \file fake_turtle.cpp
/// \brief A kinematics simulation of a differential drive robot using DiffDrive class
/// Publisher: joint_states (sensor_msgs/JointStates): Publish joint states message
///
/// Subscriber: cmd_vel (geometry_msgs/Twist): Subscribe to cmd_vel and get Twist message


#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>

#include<iostream>
#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>

// define global variables
static std::string left_wheel_joint, right_wheel_joint;                // joint names
static rigid2d::Twist2D cmd;                                           // commanded twist
static bool flag;                                                      // callback flag


/// \brief updates the body twist of the robot
/// \param msg - twist msg
void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd.vx = msg->linear.x;
    cmd.vy = msg->linear.y;                                // no y component shoule be 0
    cmd.w = msg->angular.z;

    flag = true;
}





int main(int argc, char** argv)
{
    // initiate fake_turtle node
    ros::init(argc, argv, "fake_turtle");
    ros::NodeHandle nh_("~");                               // private node handler
    ros::NodeHandle nh;                                     // public node handler

    // initiate twist subscriber
    ros::Subscriber twist_sub = nh.subscribe("cmd_vel",10,twistCallback); 
    // initiate joint publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);

    float wheel_base, wheel_radius;
    // read parameters
    nh.getParam("/left_wheel_joint", left_wheel_joint);
    nh.getParam("/right_wheel_joint",right_wheel_joint);
    
    nh.getParam("/wheel_base", wheel_base);
    nh.getParam("/wheel_radius",wheel_radius);

    // login info
    ROS_INFO("left_wheel_joint: %s", left_wheel_joint.c_str());
    ROS_INFO("right_wheel_joint: %s", right_wheel_joint.c_str());
    ROS_INFO("wheel_base: %f",wheel_base);
    ROS_INFO("wheel_radius: %f", wheel_radius);

    ROS_INFO("successfully launched the fake_turtle node!");

    // check if flag is received
    flag = false;

    // assume robot pose starts at (0,0,0)
    rigid2d::Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    // diff_drive
    rigid2d::DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // define fixed frequency
    int freq = 10;                                                         //Hz
    ros::Rate loop_rat(freq);
    
    // Timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now();

    while (nh.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();

        // received message
        if (flag)
        {
            // scaled twist based on frequency
            rigid2d::Twist2D new_cmd;
            new_cmd.vx = cmd.vx * 1.0 / freq;
            new_cmd.vy = 0.0;
            new_cmd.w = cmd.w * 1.0 / freq;

            // feedforward control
            diffDrive.feedforward(new_cmd);

            // wheel ecnoders
            rigid2d:: WheelEncoders enc;
            enc = diffDrive.getEncoders();

            // joint_state
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = cur_time;
            joint_state.name.push_back(left_wheel_joint);
            joint_state.name.push_back(right_wheel_joint);

            // pose
            joint_state.position.push_back(enc.left);
            joint_state.position.push_back(enc.right);

            flag = false;

            // publish joint state
            joint_pub.publish(joint_state);
        }

        last_time = cur_time;
        loop_rat.sleep();

    }

    return 0;
}




/// end file

