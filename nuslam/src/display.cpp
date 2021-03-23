/// \file display.cpp
/// \brief  display the detected landmark

/// Publisher:
///        + circle(visualization_msgs::MarkerArray): publish circle markers
/// Subscriber:
///        + landmarks(nuslam::Map): get landmark pose

#include<ros/ros.h>
#include<ros/console.h>
#include<vector>
#include<iostream>

#include<nuslam/Map.h>
#include<visualization_msgs/MarkerArray.h>

// parameters
static bool map_flag;                           // map flag
static std::vector<double> x;                   // circle centroid x
static std::vector<double> y;                   // circle centroid y
static std::vector<double> r;                   // circle radius
static std::string frame_id;                    // frame

void mapCallback(const nuslam::Map::ConstPtr &msg)
{
    frame_id = msg->header.frame_id;
    x = msg->x;
    y = msg->y;
    r = msg->r;
    map_flag = true;

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"display");
    ros::NodeHandle nh_("~");                               // private nodehandler
    ros::NodeHandle nh;                                     // public nodehandler

    ros::Subscriber map_sub = nh.subscribe("landmarks",10,mapCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("circle",10);

    ROS_INFO("Successfully launched display node");
    
    map_flag = false;

    while (nh.ok())
    {
        ros::spinOnce();
        if (map_flag)
        {
            
            visualization_msgs::MarkerArray map_arr;
            map_arr.markers.resize(r.size());
            
            for (unsigned int i=0;i<r.size();++i)
            {
                map_arr.markers[i].header.frame_id = "turtle";                          // change "turtle" to "base_scan" if run on the turtlebot
                map_arr.markers[i].header.stamp = ros::Time::now();
                map_arr.markers[i].lifetime = ros::Duration(1.0/5.0);
                map_arr.markers[i].ns = "map_markers";
                map_arr.markers[i].id = i;

                map_arr.markers[i].type = visualization_msgs::Marker::CYLINDER;
                map_arr.markers[i].action = visualization_msgs::Marker::ADD;

                map_arr.markers[i].pose.position.x = x[i];
                map_arr.markers[i].pose.position.y = y[i];
                map_arr.markers[i].pose.position.z = 0.5;
                map_arr.markers[i].pose.orientation.w = 1.0;

                map_arr.markers[i].scale.x = 2 * r[i];
                map_arr.markers[i].scale.y = 2 * r[i];
                map_arr.markers[i].scale.z = 0.1;

                map_arr.markers[i].color.r = 0.0;
                map_arr.markers[i].color.g = 1.0;
                map_arr.markers[i].color.b = 0.0;
                map_arr.markers[i].color.a = 1.0;

            }
            marker_pub.publish(map_arr);
            
            map_flag = false;
        }

    }

    return 0;
}

/// end file