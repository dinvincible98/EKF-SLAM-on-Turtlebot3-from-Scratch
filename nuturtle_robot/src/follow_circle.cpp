/// \file  follow_circle.cpp
/// \brief This is a node let trutlebot drive in a circle of a specified raidus at a specified speed
///
/// Publisher: 
///     + cmd_vel(geometry_msgs/Twist) publish twist velocity command to the robot
/// Service:  
///     + control_srv(nuturtle_robot/control) set direction for the robot to move or stop the its motion 
///      (0:clockwise; 1:counter-clockwise; other int numbers: stop)

#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h>
#include<nuturtle_robot/control.h>
#include<nuturtlebot/WheelCommands.h>
#include<rigid2d/diff_drive.hpp>
#include<rigid2d/rigid2d.hpp>


class Circle
{

public:
    /// \brief class constructor
    /// \param nh- Nodehandler 
    explicit Circle(ros::NodeHandle &nh);


private:

    double linear;
    double radius;
    bool control_srv_flag;
    int dir;
    
    /// \brief private nodehandler
    ros::NodeHandle nh_;

    /// \brief cmd vel publisher
    ros::Publisher cmd_pub;

    /// \brief control service
    ros::ServiceServer control_srv;

    /// \brief control service callback
    bool controlCallback(nuturtle_robot::control::Request&, nuturtle_robot::control::Response&);


};

Circle::Circle(ros::NodeHandle &nh): nh_(nh)
{

    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

    control_srv = nh_.advertiseService("control",&Circle::controlCallback,this);


    nh.getParam("/linear",linear);
    nh.getParam("/radius",radius);

    ROS_INFO("linear velocity: %f", linear);
    ROS_INFO("radius: %f",radius);
    ROS_INFO("Successfully launched follow_circle node!");

    geometry_msgs::Twist twist;

    control_srv_flag = false;
    // Timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now(); 

    while(ros::ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();
        last_time = cur_time;

        // executes only when control sevice is called
        if (control_srv_flag)
        {


            // clockwise rotation
            if (dir == 0)
            {
                twist.linear.x = linear;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = -1.0 * (linear/radius);
                cmd_pub.publish(twist);
            }
            // counter-clockwise rotation
            else if (dir == 1)
            {

                twist.linear.x = linear;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = linear/radius;
                cmd_pub.publish(twist);

            }
            // stop the motion
            else
            {

                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = 0.0;
                cmd_pub.publish(twist);


            }


        }
    }


}

bool Circle::controlCallback(nuturtle_robot::control::Request &req, nuturtle_robot::control::Response &res)
{

    dir = req.direction;
    res.set_direction = true;

    control_srv_flag = true;
    
    ROS_INFO("Service started!");
    return true;

}





int main(int argc,char** argv)
{
    ros::init (argc, argv, "follow_circle");
    ros::NodeHandle nh;
    Circle circle(nh);
    ros::spin();
    return 0;
}


/// end file
