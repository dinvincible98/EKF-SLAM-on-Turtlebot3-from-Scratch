/// \file landmark.cpp
/// \brief landmark detecion node

/// Publisher:
///        + landmarks(nuslam::Map): publish landmarks location
/// Subscriber:
///        + scan(sensor_msgs/Laserscan): get laserscan data


#include<ros/ros.h>
#include<ros/console.h>
#include<iostream>
#include<vector>
#include<sensor_msgs/LaserScan.h>
#include<visualization_msgs/MarkerArray.h>

#include<nuslam/landmark.hpp>
#include<rigid2d/rigid2d.hpp>
#include<nuslam/Map.h>


using rigid2d::Vector2D;
using rigid2d::deg2rad;
using nuslam::Laserscan;
using nuslam::Landmark;

static std::vector<float> scan;                                // laserscan
static bool scan_flag;                                          // scan flag
// static double range;                                                                    

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = msg->ranges;
    // std::cout<<scan.back()<<std::endl;
    scan_flag = true;

}




int main(int argc, char** argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle nh_("~");                                    // private nodehandler 
    ros::NodeHandle nh;                                          // public nodehandler

    ros::Subscriber scan_sub = nh.subscribe("scan",10,scanCallback);
    ros::Publisher circle_pub = nh.advertise<nuslam::Map>("landmarks",10);

    ROS_INFO("Successfully launched landmarks node");

    // laser properties
    double beam_min = 0.0;
    double beam_max = rigid2d::deg2rad(360.0);
    double beam_delta = rigid2d::PI * 2 / 360.0;
    double range_min = 0.12;
    double range_max = 3.5;


    Laserscan ls(beam_min,beam_max,beam_delta,range_min,range_max);
    // std::cout<<ls.beam_max<<std::endl;

    // landmark classifier
    double epsilon = 0.04;
    Landmark landmark(ls,epsilon);
    // std::cout<<landmark.beam_min<<std::endl;

    scan_flag = false;

    while (nh.ok())
    {

        ros::spinOnce();

        if(scan_flag)
        {
            // std::cout<<"reached"<<std::endl;
            // find feature in scan
            landmark.featureDetection(scan);
            // std::cout<<"here"<<std::endl;

            // publish new map
            nuslam::Map map;
            map.header.frame_id = "turtle";                             // change "turtle" to "base_scan" when running on turtlebot3
            map.header.stamp = ros::Time::now();

            // std::cout<<landmark.lm.size()<<std::endl;
            for (unsigned int i=0;i<landmark.lm.size();++i)
            {
                map.x.push_back(landmark.lm[i].x_hat);
                // std::cout<<"x: "<<map.x.back()<<std::endl;
                map.y.push_back(landmark.lm[i].y_hat);
                // std::cout<<"y: "<<map.y.back()<<std::endl;
                map.r.push_back(landmark.lm[i].radius);
                // std::cout<<"r: "<<map.r.back()<<std::endl;
            }        
            circle_pub.publish(map);
       
            scan_flag = false;       
        }


    }

    return 0;

}




















/// end file