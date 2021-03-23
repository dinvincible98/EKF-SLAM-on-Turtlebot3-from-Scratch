/// \file odometer node
/// \brief This node publish odometery message. It also broadcast the transform 
///        between odom_frame to body_frame
/// Publisher:
///     odom (nav_msgs/Odometery): pose of the robot in odom frame and twist in body frame
/// Subscriber:
///     joint_states (sensor_msgs/JointState): angular wheel positions
/// Service:
///     set_pose (set_pose): request provides the configuration of the robot and reset the location
///     of the odometery 


#include<ros/ros.h>
#include<ros/console.h>
#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<tf2_ros/transform_broadcaster.h>
#include<string>
#include<vector>
#include<iostream>
#include<rigid2d/diff_drive.hpp>
#include<rigid2d/set_pose.h>


// global variables
static std::string left_wheel_joint, right_wheel_joint;                  //joint names
static double left_pose, right_pose;                                     // wheel angular pose
static rigid2d::Pose2D pose_reset;                                       // pose set by srv
static bool srv_active;                                                  // set pose srv activated 


// jointStateCallback function
void jointStateCallback(const sensor_msgs::JointState::ConstPtr & msg)
{
    left_pose = msg->position.at(0);                     // left_idx = 0
    right_pose = msg->position.at(1);                   // right_idx = 1


}

// setPose service callback function
bool setPoseCallback(rigid2d::set_pose::Request &req, rigid2d::set_pose::Response &res)
{
    // requested pose
    pose_reset.theta = req.theta;
    pose_reset.x = req.x;
    pose_reset.y = req.y;

    ROS_INFO("pose x: %f y: %f theta: %f",pose_reset.x,pose_reset.y,pose_reset.theta);

    // set response
    res.set_pose_state = true;

    // activate service flag
    srv_active = true;

    ROS_INFO("set pose service activated!");

    return true;
}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"odometer_node");                     //initiate ros node 
    ros::NodeHandle nh_("~");                                 // private handle to ros
    ros::NodeHandle nh;                                       // public handle to ros

    // variables
    std::string odom_frame_id, body_frame_id;
    float wheel_base, wheel_radius;

    // global paramters
    nh.getParam("/odom_frame_id",odom_frame_id);
    nh.getParam("/body_frame_id",body_frame_id);
    nh.getParam("/left_wheel_joint", left_wheel_joint);
    nh.getParam("/right_wheel_joint", right_wheel_joint);

    nh.getParam("/wheel_base", wheel_base);
    nh.getParam("/wheel_radius",wheel_radius);

    //Info
    ROS_INFO("odom frame id: %s", odom_frame_id.c_str());
    ROS_INFO("base frame id: %s", body_frame_id.c_str());
    ROS_INFO("left wheel joint: %s", left_wheel_joint.c_str());
    ROS_INFO("right wheel joint: %s", right_wheel_joint.c_str());
    ROS_INFO("wheel_base: %f", wheel_base);
    ROS_INFO("wheel radius: %f", wheel_radius);

    ROS_INFO("Successfully launched the odometer node!");

    // initiate subscriber
    ros::Subscriber joint_sub = nh.subscribe("joint_states",10,jointStateCallback);
    // initiate publisher
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",10);
    // initiate service server
    ros::ServiceServer setPose_srv = nh.advertiseService("set_pose",setPoseCallback);
    // initiate transform broadcaster
    tf2_ros::TransformBroadcaster odom_br;

    // service requested
    srv_active = false;

    // Assume Pose starts at (0,0,0)
    rigid2d::Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    // diff_drive
    rigid2d::DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    //Timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now();

    while(nh.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();

        // setPose srv flag
        if (srv_active)
        {
            diffDrive.reset(pose_reset);

            srv_active = false;
        }
        
        // update only when jointstates not 0
        if (left_pose!=0 && right_pose!=0)
        {        
            // update odometer
            diffDrive.updateOdom(left_pose,right_pose);
        }

        // pose relative to odom frame
        pose = diffDrive.pose();

        // body twist
        rigid2d::WheelVelocities vels;
        vels = diffDrive.wheelVelocities();
        rigid2d::Twist2D Vb;
        Vb = diffDrive.wheelsToTwist(vels);

        // convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0,0,pose.theta);
        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf2::toMsg(q);

        // broadcast transform from odom to body
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = cur_time;
        odom_tf.header.frame_id = odom_frame_id;
        odom_tf.child_frame_id = body_frame_id;

        odom_tf.transform.translation.x = pose.x;
        odom_tf.transform.translation.y = pose.y;
        odom_tf.transform.translation.z = 0.0;

        odom_tf.transform.rotation = odom_quat;

        // send transform
        odom_br.sendTransform(odom_tf);

        // publish odom msg over ros
        nav_msgs::Odometry odom;
        odom.header.stamp = cur_time;
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = body_frame_id;

        // pose in odom frame
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // velocity in body frame
        odom.twist.twist.linear.x = Vb.vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = Vb.w; 

        // publish odom
        odom_pub.publish(odom);

        last_time = cur_time;
    

    }
    return 0;

}

// end file 