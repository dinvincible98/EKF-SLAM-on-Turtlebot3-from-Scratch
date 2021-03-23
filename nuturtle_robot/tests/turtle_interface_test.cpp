/// \file turtle_interface_test.cpp
/// \brief this is a test file for turtle_interface_node

#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<nuturtlebot/WheelCommands.h>
#include<nuturtlebot/SensorData.h>
#include<catch_ros/catch.hpp>

/// \brief stores turtlebot wheel joint state
struct WheelJoint
{
    std::string name;
    double position = 0.0;
    double velocity = 0.0;
};

/// \brief helps test the turtlebot wheel speed and sensor msg
struct TurtlebotHelper
{
    // parameters
    int left_vel, right_vel;                                  // left/right wheel velocity
    bool wheelVel_flag;                                       // flag for wheelVel callback
    bool jointState_flag;                                     // flag for jointState callback
    int ctr;                                                  // counter for number of callback received  
    
    WheelJoint left_joint;
    WheelJoint right_joint;

    
    TurtlebotHelper():left_vel(0), right_vel(0),
                      wheelVel_flag(false),jointState_flag(false),ctr(0) {};

    void wheelCallback(const nuturtlebot::WheelCommands::ConstPtr & msg)
    {
        left_vel = msg->left_velocity;
        right_vel = msg->right_velocity;

        wheelVel_flag = true;
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr & data)
    {
        // left wheel joint
        left_joint.name = data->name.at(0);
        left_joint.position = data->position.at(0);
        left_joint.velocity = data->velocity.at(0);
    
        // right wheel joint
        right_joint.name = data->name.at(1);
        right_joint.position = data->position.at(1);
        right_joint.velocity = data->velocity.at(1);

        jointState_flag = true;
    
        ctr++;
    }

};

TEST_CASE("twistPureTrans","[turtle_interface]")
{
    
    ros::NodeHandle nh;
    TurtlebotHelper turtle;

    // publisher
    ros::Publisher cmd_pub;
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1,true);

    // subscriber
    ros::Subscriber wheel_sub;
    wheel_sub = nh.subscribe("/wheel_cmd",1,&TurtlebotHelper::wheelCallback, &turtle);

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = 0.0;

    while (!turtle.wheelVel_flag)
    {
        ros::spinOnce();
        cmd_pub.publish(cmd);
    }

    REQUIRE(turtle.left_vel == 124);
    REQUIRE(turtle.right_vel == 124);
    
}

TEST_CASE("twistPureRot","[turtle_interface]")
{
    
    ros::NodeHandle nh;
    TurtlebotHelper turtle;

    // publisher
    ros::Publisher cmd_pub;
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1,true);

    // subscriber
    ros::Subscriber wheel_sub;
    wheel_sub = nh.subscribe("/wheel_cmd",1,&TurtlebotHelper::wheelCallback, &turtle);

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = -0.1;

    while (!turtle.wheelVel_flag)
    {
        ros::spinOnce();
        cmd_pub.publish(cmd);
    }

    REQUIRE(turtle.left_vel == 105);
    REQUIRE(turtle.right_vel == -105);
    
}

TEST_CASE("sensorJointState", "[turtle_interface]")
{
    ros::NodeHandle nh;
    TurtlebotHelper turtle;

    ros::Publisher sensor_pub;
    sensor_pub = nh.advertise<nuturtlebot::SensorData>("/sensor_data",1,true);

    ros:: Subscriber joint_sub;
    joint_sub = nh.subscribe("/joint_states",1,&TurtlebotHelper::jointStateCallback,&turtle);

    nuturtlebot::SensorData sensor_data;
    sensor_data.left_encoder = 100;
    sensor_data.right_encoder = 100;

    while(nh.ok())
    {
        ros::spinOnce();
        if (turtle.jointState_flag)
        {
            // check steady state condition of joint state
            if(turtle.ctr == 0)
            {
                if(turtle.left_joint.velocity!=0 && turtle.right_joint.velocity!=0)
                {
                    continue;
                }
            }
            else if(turtle.ctr!=0)
            {
                if (turtle.left_joint.velocity==0 && turtle.right_joint.velocity==0)
                {
                    break;
                }
            }
        }
        sensor_pub.publish(sensor_data);

    }

    REQUIRE(turtle.left_joint.position == Approx(0.153).margin(0.001));
    REQUIRE(turtle.right_joint.position == Approx(0.153).margin(0.001));
    REQUIRE(turtle.left_joint.velocity == 0.0);
    REQUIRE(turtle.right_joint.velocity == 0.0);

}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"turtle_interface_test");
    ROS_INFO("Successfully launched the test node!");
    
    return 0;
}