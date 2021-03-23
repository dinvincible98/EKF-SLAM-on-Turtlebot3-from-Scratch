/// \file odometer node
/// \brief This node publish odometery message. It also broadcast the transform 
///        between odom_frame to body_frame
/// Publisher:
///     + odom (nav_msgs/Odometery): pose of the robot in odom frame and twist in body frame
///     + slam_path(nav_msgs/Path): path for ekf slam in map frame
///     + odom_path(nav_msgs/Path): path for odom in map frame
///     + lm_pub(visualization_msgs/MarkerArray): marker array for landmark in range
///     + slam_error(nuslam/pose_error): publish average slam error
///     + odom_erro(nuslam/pose_error): publish average odom error
/// Subscriber:
///     + joint_states (sensor_msgs/JointState): angular wheel positions
///     + measurement(nurtlesim/map): get landmark position in robot frame
///     + landmarks(nuslam/Map): get detected landmark from laser
///     + turtle_pose(nuslam/pose_error): get turtle pose from simulator
///


#include<ros/ros.h>
#include<ros/console.h>
#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<tf2_ros/transform_broadcaster.h>
#include<geometry_msgs/PoseStamped.h>
#include<visualization_msgs/MarkerArray.h>


#include<string>
#include<vector>
#include<iostream>


#include<rigid2d/diff_drive.hpp>
#include<rigid2d/set_pose.h>

#include<nurtlesim/map.h>
#include<nuslam/ekf_filter.hpp>
#include<nuslam/Map.h>
#include<nuslam/pose_error.h>


using rigid2d::Twist2D;
using rigid2d::Vector2D;
using rigid2d::Pose2D;
using rigid2d::Transform2D;
using rigid2d::TransformData2D;
using rigid2d::normalize_angle;

// global variables
static std::string left_wheel_joint, right_wheel_joint;                  //joint names
static double left_pose, right_pose;                                     // wheel angular pose for odometry
static rigid2d::Pose2D pose_reset;                                       // pose set by srv
static bool srv_active;                                                  // set pose srv activated 
static double ekf_left, ekf_right;                                       // wheel angular pose for SLAM

static std::vector<double> x_pos, y_pos;                                 // x/y position of landmark
static std::vector<int> id;                                              // id for landmark
static std::vector<double> Q;                                            // motion noise
static std::vector<double> R;                                            // sensor noise

static bool wheel_odom_flag;                                             // odom update flag
static bool map_flag;

static std::vector<Vector2D> measures;                                   // x/y location of cylinder relative to robot                                     
static std::vector<Vector2D> measures2;                                  // x/y location of landmark relative to robot       

static double pose_x,pose_y,pose_theta;                                  // pose of turtle

bool knownDataAssociation;                                               // EKF with known data association
bool unknonDataAssociation;                                              // EKF with unknown data association

// jointStateCallback function
void jointStateCallback(const sensor_msgs::JointState::ConstPtr & msg)
{

    left_pose = msg->position.at(0);                     // left_idx = 0
    right_pose = msg->position.at(1);                   // right_idx = 1

    wheel_odom_flag = true;

}

// mapCallback function
void mapCallback(const nurtlesim::map::ConstPtr &msg)
{
    measures.clear();
    measures.reserve(msg->cx.size());

    for (unsigned int i=0;i<msg->cx.size();++i)
    {
        double x = msg->cx.at(i);
        double y = msg->cy.at(i);

        // std::cout<<x<<std::endl;
        // std::cout<<y<<std::endl;
        
        Vector2D v;
        v.x = x;
        v.y = y;
        if (v.x!=0 && v.y!=0)
        {
            measures.push_back(v);
        }
        // ROS_INFO_STREAM("measure x:"<<measures[i].x);
        // ROS_INFO_STREAM("measure y:"<<measures[i].y);
    }


    map_flag = true;
}

// landmarkCallback func
void landmarkCallback(const nuslam::Map::ConstPtr &msg)
{
    measures2.clear();
    measures2.reserve(msg->x.size());

    for (unsigned int i=0;i<msg->x.size();++i)
    {
        double x2 = msg->x.at(i);
        double y2 = msg->y.at(i);

        // std::cout<<x<<std::endl;
        // std::cout<<y<<std::endl;
        
        Vector2D v2;
        v2.x = x2;
        v2.y = y2;

        measures2.push_back(v2);
        // ROS_INFO_STREAM("measure2 x:"<<measures2[i].x);
        // ROS_INFO_STREAM("measure2 y:"<<measures2[i].y);
    }
}

void poseCallback(const nuslam::pose_error::ConstPtr &msg)
{
    pose_x = msg->x_error;
    pose_y = msg->y_error;
    pose_theta = msg->theta_error;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"slam");                              //initiate ros node 
    ros::NodeHandle nh_("~");                                 // private handle to ros
    ros::NodeHandle nh;                                       // public handle to ros

    // variables
    std::string map_frame_id, odom_frame_id, body_frame_id;
    float wheel_base, wheel_radius;
    double radius;
    double max_range;

    // bool unknonDataAssociation;                                              // EKF with unknown data association

    // global paramters
    nh.getParam("/map_frame_id", map_frame_id);
    nh.getParam("/odom_frame_id",odom_frame_id);
    nh.getParam("/body_frame_id",body_frame_id);
    nh.getParam("/left_wheel_joint", left_wheel_joint);
    nh.getParam("/right_wheel_joint", right_wheel_joint);

    nh.getParam("/wheel_base", wheel_base);
    nh.getParam("/wheel_radius",wheel_radius);
    nh.getParam("/r",radius);
    nh.getParam("/max_range",max_range);
    
    // get landmark pose
    nh.getParam("/x", x_pos);
    nh.getParam("/y", y_pos);
    nh.getParam("/ids", id);

    // get noise
    nh.getParam("/sensor_noise",R);
    nh.getParam("/motion_noise",Q); 

    // known / unknown SLAM
    // nh.getParam("known",knownDataAssociation);
    // nh.getParam("unknown",unknonDataAssociation);
    // ROS_INFO_STREAM("knonw"<<knownDataAssociation);
    //Info
    ROS_INFO("map frame id: %s", map_frame_id.c_str());
    ROS_INFO("odom frame id: %s", odom_frame_id.c_str());
    ROS_INFO("base frame id: %s", body_frame_id.c_str());
    ROS_INFO("left wheel joint: %s", left_wheel_joint.c_str());
    ROS_INFO("right wheel joint: %s", right_wheel_joint.c_str());
    ROS_INFO("wheel_base: %f", wheel_base);
    ROS_INFO("wheel radius: %f", wheel_radius);

    ROS_INFO("Successfully launched the odometer node!");

    // initiate joint subscriber
    ros::Subscriber joint_sub = nh.subscribe("joint_states",10,jointStateCallback);

    // measurement subscriber
    ros::Subscriber fake_measure_sub = nh.subscribe("measurement",10, mapCallback);  
    
    // odom_path publisher
    ros::Publisher odom_path_pub = nh.advertise<nav_msgs::Path>("odom_path",10); 

    // slam_path publisher
    ros::Publisher slam_path_pub = nh.advertise<nav_msgs::Path>("slam_path",10);    

    // landmark publisher
    ros::Publisher lm_pub = nh.advertise<visualization_msgs::MarkerArray>("landmark",10);
    
    // odom publisher
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",10);

    // landmark subsriber
    ros::Subscriber lm_sub = nh.subscribe("landmarks",10,landmarkCallback);

    // slam pose error publisher
    ros::Publisher slam_error_pub = nh.advertise<nuslam::pose_error>("slam_error",10);
    // odom pose error publisher
    ros::Publisher odom_error_pub = nh.advertise<nuslam::pose_error>("odom_error",10);
    
    // turtle pose sub;
    ros::Subscriber pose_sub = nh.subscribe("turtle_pose",10,poseCallback);

    // initiate transform broadcaster 
    tf2_ros::TransformBroadcaster br;




    // service requested
    srv_active = false;

    // Assume Pose starts at (0,0,0)
    rigid2d::Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    // diff_drive model for Odometry
    rigid2d::DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // diff_drive model for SLAM
    rigid2d::DiffDrive diffDrive_slam(pose,wheel_base,wheel_radius);

    // number of landmark 
    int n = 12;
    double md_max = 750;
    double md_min = 100;
    nuslam::EKF ekf(n,Q,R,md_max,md_min);

    // path from odometry
    nav_msgs::Path odom_path;
    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = map_frame_id;

    
    // path from SLAM
    nav_msgs::Path slam_path;
    slam_path.header.stamp = ros::Time::now();
    slam_path.header.frame_id = map_frame_id;

    // received joint msg
    wheel_odom_flag = false;

    map_flag = false;
    
    //Timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now();

    bool knownDataAssociation;                                               // EKF with known data association
    nh.getParam("known",knownDataAssociation);
    // std::cout<<"known: "<<knownDataAssociation<<std::endl;



    while(nh.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();

        
        // update only when jointstates not 0
        if (left_pose!=0 && right_pose!=0)
        {        
            // update odometry
            diffDrive.updateOdom(left_pose,right_pose);
            // pose relative to odom frame
            pose = diffDrive.pose();
            // std::cout<<"Pose X:"<<pose.x<<std::endl;

            // set ekf wheel encoder to current odometry encoders
            ekf_left = left_pose;
            ekf_right = right_pose;
            
            diffDrive_slam.updateOdom(ekf_left,ekf_right);
        
        }
        rigid2d::WheelVelocities vel = diffDrive_slam.wheelVelocities();
        rigid2d::Twist2D twist = diffDrive_slam.wheelsToTwist(vel);
        if (knownDataAssociation)
        {
            ekf.knownDataSLAM(measures,twist);

        }
        else
        {  
            ekf.unknownDataSLAM(measures2,twist);
        }

        /////////////////////////////////////////////////////////////////////////////////////

        // broadcaster transform from map to odom
        // transform from map to robot
        Transform2D Tmr = ekf.getRobotState();
        // std::cout<<Tmr<<std::endl;

        // transform from odom to robot
        Vector2D Vor(pose.x, pose.y);
        Transform2D Tor(Vor,pose.theta);
        // std::cout<<Tor<<std::endl;

        // transform from robot to odom
        Transform2D Tro = Tor.inv();

        // get tranform from map to odom
        Transform2D Tmo = Tmr * Tro;
        // std::cout<<Tmo<<std::endl;
        TransformData2D pose_mapToOdom = Tmo.displacement();

        
        tf2::Quaternion q_mo;
        q_mo.setRPY(0,0,pose_mapToOdom.theta);
        geometry_msgs::Quaternion quat_mo;
        quat_mo = tf2::toMsg(q_mo);

        // broadcast transfrom from map to odom
        geometry_msgs::TransformStamped tf_mo;
        tf_mo.header.stamp = ros::Time::now();
        tf_mo.header.frame_id = map_frame_id;
        tf_mo.child_frame_id = odom_frame_id;

        tf_mo.transform.translation.x = pose_mapToOdom.x;
        tf_mo.transform.translation.y = pose_mapToOdom.y;
        tf_mo.transform.translation.z = 0.0;
        tf_mo.transform.rotation = quat_mo;

        br.sendTransform(tf_mo);

        //////////////////////////////////////////////////////////////////////////////////////////////////
        // odom publisher
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
        br.sendTransform(odom_tf);

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

        /////////////////////////////////////////////////////////////////////////////////////////////

        // path from SLAM
        geometry_msgs::PoseStamped slam_pose;
        TransformData2D pose_mapToRobot = Tmr.displacement();
        
        tf2::Quaternion q_mr;
        q_mr.setRPY(0,0,pose_mapToRobot.theta);
        geometry_msgs::Quaternion quat_mr;
        quat_mr = tf2::toMsg(q_mr);
        
        slam_pose.header.stamp = ros::Time::now();
        slam_pose.header.frame_id = map_frame_id;
        slam_pose.pose.position.x = pose_mapToRobot.x;
        slam_pose.pose.position.y = pose_mapToRobot.y;
        slam_pose.pose.position.z = 0.0;
        slam_pose.pose.orientation = quat_mr;
        
        slam_path.poses.push_back(slam_pose);

        // publish slam path
        slam_path_pub.publish(slam_path);

        // path from odom
        geometry_msgs::PoseStamped odom_pose;
        tf2::Quaternion q_or;
        q_or.setRPY(0,0,pose.theta);
        geometry_msgs::Quaternion quat_or;
        quat_or = tf2::toMsg(q_or);

        odom_pose.header.stamp = ros::Time::now();
        odom_pose.header.frame_id = map_frame_id;
        odom_pose.pose.position.x = pose.x;
        odom_pose.pose.position.y = pose.y;
        odom_pose.pose.position.z = 0.0;
        odom_pose.pose.orientation = quat_or;

        odom_path.poses.push_back(odom_pose);

        // publish odom path
        odom_path_pub.publish(odom_path);

        // pose error message
        // slam & turtle
        nuslam::pose_error slam_pose_err;
        double slam_x,slam_y,slam_theta; 
        slam_x = pose_mapToRobot.x - pose_x;
        slam_y = pose_mapToRobot.y - pose.y;
        slam_theta = pose_mapToRobot.theta - pose.theta;

        std::vector<double> x_list,y_list,theta_list;
        x_list.push_back(slam_x);
        y_list.push_back(slam_y);
        theta_list.push_back(slam_theta);

        double sum_x,sum_y,sum_theta;
        sum_x = 0;
        sum_y = 0;
        sum_theta = 0;
        for (unsigned int i=0;i<x_list.size();++i)
        {
            sum_x += x_list[i];
            sum_y += y_list[i];
            sum_theta += theta_list[i];
        }
        
        double mean_x,mean_y,mean_theta;
        mean_x = sum_x/x_list.size();
        mean_y = sum_y/x_list.size();
        mean_theta = sum_theta/theta_list.size();
        
        slam_pose_err.header.frame_id = "map";
        slam_pose_err.header.stamp = ros::Time::now();
        slam_pose_err.x_error = mean_x;
        slam_pose_err.y_error = mean_y;
        slam_pose_err.theta_error = mean_theta;
        
        slam_error_pub.publish(slam_pose_err);
        
        // odom & turtle
        nuslam::pose_error odom_pose_err;
        double odom_x,odom_y,odom_theta;
        odom_x = pose.x - pose_x;
        odom_y = pose.y - pose_y;
        odom_theta = pose.theta - pose_theta;

        std::vector<double> x_list2,y_list2,theta_list2;
        x_list2.push_back(odom_x);
        y_list2.push_back(odom_y);
        theta_list2.push_back(odom_theta);

        double sum_x2,sum_y2,sum_theta2;
        sum_x2 = 0;
        sum_y2 = 0;
        sum_theta2 = 0;
        for (unsigned int i=0;i<x_list2.size();++i)
        {
            sum_x2 += x_list2[i];
            sum_y2 += y_list2[i];
            sum_theta2 += theta_list2[i];
        }
        
        double mean_x2,mean_y2,mean_theta2;
        mean_x2 = sum_x2/x_list.size();
        mean_y2 = sum_y2/x_list.size();
        mean_theta2 = sum_theta2/theta_list.size();
        
        odom_pose_err.header.frame_id = "map";
        odom_pose_err.header.stamp = ros::Time::now();
        odom_pose_err.x_error = mean_x2;
        odom_pose_err.y_error = mean_y2;
        odom_pose_err.theta_error = mean_theta2;

        odom_error_pub.publish(odom_pose_err);

        // map
        std::vector<Vector2D> map;
        // marker array of landmark
        ekf.getMap(map);


        // visualization marker array
        visualization_msgs::MarkerArray lm_marker_array;
        lm_marker_array.markers.resize(map.size());

        for (unsigned int i=0; i<map.size(); ++i)
        {
            lm_marker_array.markers[i].header.frame_id = "map";
            lm_marker_array.markers[i].header.stamp = ros::Time::now();
            lm_marker_array.markers[i].lifetime = ros::Duration(1.0/5.0);
            lm_marker_array.markers[i].ns = "landmark";
            lm_marker_array.markers[i].id = i;

            lm_marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
            double distance = std::sqrt(std::pow(map[i].x - pose_mapToRobot.x,2) + std::pow(map[i].y-pose_mapToRobot.y,2));

            if (distance <= max_range)
            {
                lm_marker_array.markers[i].action = visualization_msgs::Marker::ADD;

                lm_marker_array.markers[i].pose.position.x = map[i].x;
                lm_marker_array.markers[i].pose.position.y = map[i].y;
                lm_marker_array.markers[i].pose.position.z = 0.8;

                lm_marker_array.markers[i].pose.orientation.w = 1.0;

                lm_marker_array.markers[i].scale.x = radius * 1.2;
                lm_marker_array.markers[i].scale.y = radius * 1.2;
                lm_marker_array.markers[i].scale.z = 0.2;           

                lm_marker_array.markers[i].color.r = 1.0;
                lm_marker_array.markers[i].color.g = 1.0;
                lm_marker_array.markers[i].color.b = 0.0;
                lm_marker_array.markers[i].color.a = 1.0;
            }
            else
            {
                lm_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;    
            }

        }

        lm_pub.publish(lm_marker_array);

        last_time = cur_time;

    }

    return 0;

}

// end file 
