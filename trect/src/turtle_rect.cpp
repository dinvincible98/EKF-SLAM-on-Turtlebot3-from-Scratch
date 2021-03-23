/// \file turtle_rect.cpp
/// \brief Turtle moves in a rectangle trajectory path

/// Publishes:
///     turtle1/cmd_vel(Twist): Linear and angular velocity of the turtle
/// Subscribes:
///     turtle1/pose(Pose): pose of the turtle
/// Services:
///     start: Initiate the turtle to move in a rectangular trajectory

#include<ros/ros.h>
#include<ros/console.h>
#include<turtlesim/Pose.h>
#include<turtlesim/SetPen.h>
#include<turtlesim/TeleportAbsolute.h>
#include<geometry_msgs/Twist.h>
#include<std_srvs/Empty.h>
#include<math.h>
#include<string>


using namespace std;

class Rect
{

public:
    /// \brief class constructor
    /// \param nh - the NodeHandler
    explicit Rect(ros::NodeHandle &nh);

    /// \brief destructor
    ~Rect();

    /// \brief Control loop
    void Control();


private:
    // parameters 
    int x_;                            // x coordinate for lower left corner of rectangle 
    int y_;                            // y coordinate for lower left corner of rectangle 
    int width_;                        // width of the rectangle
    int height_;                       // height of the rectangle
    int freq_;                         // publishing rate for control
    int iter_;                         // counter for determining times for state
    float trans_vel_;                  // linear velocity
    float rot_vel_;                    // angular velocity
    
    bool init_pos_;                    // turtle starts at the beginning

    // pose of turtle
    turtlesim::Pose pose_;

    // Handle node 
    ros::NodeHandle & nh_;
    
    // subscirber to pose topic
    ros::Subscriber pose_sub_;
    
    // publisher to cmd_vel;
    ros::Publisher vel_pub_;
    
    // service for starting at the beginning of the rectangle  
    ros::ServiceServer traj_srv_;
    
    // set turtle's pen turtlesim srv
    ros::ServiceClient pen_client_;
    
    // set teleport turtle turtlesim srv
    ros::ServiceClient tele_client_; 

    // set pen service object
    turtlesim::SetPen pen_srv_;

    // teleport service object
    turtlesim::TeleportAbsolute tele_srv_;


    /// \brief start turtle
    /// \param request - empty request
    /// \param response - empty response
    /// \return bool
    bool trajCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

    /// \brief update the pose 
    /// \param pose_msg - const pointer to turtle pose
    /// \return void
    void poseCallback(const turtlesim::Pose::ConstPtr & pose_msg);


};

Rect::Rect(ros::NodeHandle &nh): nh_(nh)
{
    // turtle not moving at the beginning
    init_pos_ = false;
    
    
    // subscribe to pose of turtle
    pose_sub_= nh_.subscribe("turtle1/pose", 1, &Rect::poseCallback, this);
    
    // publish velocity
    vel_pub_= nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // start the trajectory service
    traj_srv_ = nh_.advertiseService("start", &Rect::trajCallback, this);

    // teleport client
    tele_client_ = nh_.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    // pen client
    pen_client_ = nh_.serviceClient<turtlesim::SetPen>("turtle1/set_pen");

    // read parameters from server
    nh.getParam("/x", x_);
    nh.getParam("/y", y_);
    nh.getParam("/width", width_);
    nh.getParam("/height", height_);
    nh.getParam("/max_xdot", trans_vel_);
    nh.getParam("/max_wdot", rot_vel_);
    nh.getParam("/frequency", freq_);
    
    // Output trect parameters
    ROS_INFO("x: %d", x_);
    ROS_INFO("y: %d", y_);
    ROS_INFO("width: %d", width_);
    ROS_INFO("height: %d", height_);
    ROS_INFO("max_xdot: %f", trans_vel_);
    ROS_INFO("max_wdot: %f", rot_vel_);
    ROS_INFO("frequency: %d", freq_);

    // wait for service 
    ros::service::waitForService("turtle1/set_pen", -1);
    ros::service::waitForService("turtle1/teleport_absolute", -1);

    // set pen off
    pen_srv_.request.r = 0;
    pen_srv_.request.g = 0;
    pen_srv_.request.b = 0;
    pen_srv_.request.width = 0;
    pen_srv_.request.off = 1;

    pen_client_.call(pen_srv_);

    // teleport
    tele_srv_.request.x = x_;
    tele_srv_.request.y = y_;
    tele_srv_.request.theta = 0;

    tele_client_.call(tele_srv_);
    
    // set pen on
    pen_srv_.request.r = 255;
    pen_srv_.request.g = 255;
    pen_srv_.request.b = 255;
    pen_srv_.request.width = 1;
    pen_srv_.request.off = 0;

    pen_client_.call(pen_srv_);


    ROS_INFO("Successfully launch trect node!");

}

Rect::~Rect()
{

}

void Rect::Control()
{
    ros::Rate loop_rat(freq_);
    geometry_msgs::Twist twist_msg;

    // Time for each state
    float horizon_t = (float)(width_)/ trans_vel_;                   // time for horizontal movement
    float vertical_t = (float)(height_) / trans_vel_;                // time for vertical movement
    float turn_t = (float)(M_PI/2) / rot_vel_;                       // time for turnning
    float cur_t = 0.0;

    //states
    string states[3] = {"H", "T", "V"};                              // H: Horizontal T: Turn V: Vertical
    string state;                                                    // current state
    string prev_state;                                               // previous state


    // pose
    float x = x_;
    float y = y_;
    float theta = 0;

    iter_ = 0;

    while(ros::ok())
    {   
        // set control based on states
        if (state=="H" && cur_t<=horizon_t )
        {
            ROS_INFO("Moving horizontally");
            twist_msg.linear.x = trans_vel_;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;
            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            twist_msg.angular.z = 0;
        }

        else if(state=="T" && cur_t<=turn_t)
        {   
            ROS_INFO("Turnning");
            twist_msg.linear.x = 0;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;
            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            twist_msg.angular.z = rot_vel_;
        }

        
        else if(state=="V" && cur_t<=vertical_t)
        {
            ROS_INFO("Moving vertically");
            twist_msg.linear.x = trans_vel_;
            twist_msg.linear.y = 0;
            twist_msg.linear.z = 0;
            twist_msg.angular.x = 0;
            twist_msg.angular.y = 0;
            twist_msg.angular.z = 0;
            
        }

        // set state transitions
        if (state=="H" && prev_state=="T" && cur_t>=horizon_t){
            state = "T";
            prev_state = "H";
            iter_= 0;
        }

        else if (state=="T" && prev_state=="H" && cur_t>=turn_t){
            state = "V";
            prev_state = "T";
            iter_= 0;
        }

        else if (state=="V" && prev_state=="T" && cur_t>=vertical_t){
            state = "T";
            prev_state = "V";
            iter_= 0;
        }

        else if (state=="T" && prev_state=="V" && cur_t>=turn_t){
            state = "H";
            prev_state = "T";
            iter_= 0;
        }

        // update pose
        x += twist_msg.linear.x * cos(theta) * 1.0 / (float)freq_;
        y += twist_msg.linear.x * sin(theta) * 1.0 / (float)freq_;
        theta += twist_msg.angular.z * 1.0 / (float)freq_;

        // wrap -PI -> PI
        theta = fmod(theta+M_PI, 2*M_PI);
        if (theta < 0)
        {
            theta += 2 * M_PI;
        }
        theta -= M_PI;

        if (!init_pos_ && pose_.x==x_ && pose_.y ==y_ && pose_.theta==0)
        {
            ROS_INFO("At starting position");
            iter_ = 0;
            state = "H";
            prev_state = "T";

            x = x_;
            y = y_;
            theta = 0;

        }
        else if (init_pos_)
        {
            vel_pub_.publish(twist_msg);
        }

        iter_++;
        cur_t = (float)(iter_ )/(float)(freq_);

        ros::spinOnce();
        loop_rat.sleep();


    }


}



bool Rect::trajCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{   

    // publish no control to make sure turtle does not move when it spawns
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    vel_pub_.publish(twist_msg);

    // turn pen off
    pen_srv_.request.r = 0;
    pen_srv_.request.g = 0;
    pen_srv_.request.b = 0;
    pen_srv_.request.width = 0;
    pen_srv_.request.off = 1;

    pen_client_.call(pen_srv_);

    // teleport
    tele_client_.call(tele_srv_);
    
    // turn pen on(red color)
    pen_srv_.request.r = 255;
    pen_srv_.request.g = 0;
    pen_srv_.request.b = 0;
    pen_srv_.request.width = 2;
    pen_srv_.request.off = 0;

    pen_client_.call(pen_srv_);

    // let turtle begin to move
    init_pos_ = true;

    return true;

}

void Rect::poseCallback(const turtlesim::Pose::ConstPtr & pose_msg)
{
    pose_ = *pose_msg;
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_rect");
    ros::NodeHandle nh;
    Rect rect(nh);
    rect.Control();
    ros::spin();

    return 0;
}



// end file
