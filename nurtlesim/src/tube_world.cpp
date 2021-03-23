/// \file fake_turtle.cpp
/// \brief A kinematics simulation of a differential drive robot using DiffDrive class
/// Publisher: 
///     + joint_states (sensor_msgs/JointStates): Publish joint states message
///     + tubes (visualization_msgs/MarkerArray): Publish cylindrical tubes at given position and represent robot as a sphere
///     + fake_sensor (visualization_msgs/MarkerArray): Publish fake sensor when the robot is in the range of max scan.
///     + real_path (nav_msgs/Path): Publish the robot path trajectory
///     + cmd_vel (geometry_msgs/Twist): Publish cmd_vel when the collision happens
///     + sphere (visualization_msgs/MarkerArray): simulate robot as a sphere
///     + measurement (nurtlesim/map): publish landmark position in robot frame
///     + scan(sensor_msgs/Laserscan): publish fake laserscan
///     + turtle_pose(nuslam/pose_error): publish turtle_pose
/// Subscriber: 
///     + cmd_vel (geometry_msgs/Twist): Subscribe to cmd_vel and get Twist message
/// Broadcaster:
///     + turtle_br (ros::Transformbroadcaster): Broadcaster tf from world->turtle, world->odom->base_footprint
/// 

#include<ros/ros.h>
#include<ros/console.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/LaserScan.h>
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Path.h>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<random>
#include<cmath>
#include<iostream>
#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>
#include<nurtlesim/map.h>
#include<nuslam/Map.h>
#include<nuslam/pose_error.h>

// define global variables
static std::string left_wheel_joint, right_wheel_joint;                // joint names
static rigid2d::Twist2D cmd;                                           // commanded twist
static bool flag;                                                      // callback flag
static nurtlesim::map map;                                             // measurement map
static std::vector<rigid2d::Vector2D> measure;


/// \brief updates the body twist of the robot
/// \param msg - twist msg
void twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd.vx = msg->linear.x;
    cmd.vy = msg->linear.y;                                // no y component shoule be 0
    cmd.w = msg->angular.z;

    flag = true;
}

/// \brief generate random number
std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    
    return mt;
}

/// \brief intersection func
bool intersect(const rigid2d::Vector2D v1, const rigid2d::Vector2D v2,double radius)
{
    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dr = std::sqrt(dx*dx + dy*dy);
    double D = v1.x*v2.y - v2.x*v1.y;

    double delta = radius*radius * dr*dr - D*D;
    if (delta > 0)
    {
        return true;
    }
    return false;
}



int main(int argc, char** argv)
{
    // initiate fake_turtle node
    ros::init(argc, argv, "tube_world");
    ros::NodeHandle nh_("~");                               // private node handler
    ros::NodeHandle nh;                                     // public node handler

    // initiate twist subscriber
    ros::Subscriber twist_sub = nh.subscribe("cmd_vel",10,twistCallback); 
    // initiate joint publisher
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",10);
    // marker array publisher
    ros::Publisher marker_arrary_pub = nh.advertise<visualization_msgs::MarkerArray>("tubes",10,true);
    // path publisher
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("real_path",10);
    // turtle frame
    tf2_ros::TransformBroadcaster turtle_br;
    // faker sensor 
    ros::Publisher sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_sensor",10);
    // robot collision sphere
    ros::Publisher sphere_pub = nh.advertise<visualization_msgs::MarkerArray>("sphere",10);
    // fake measurement publisher
    ros::Publisher fake_measure_pub = nh.advertise<nurtlesim::map>("measurement",10);
    // fake laserscan publisher
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan",10);
    // turtle pose
    ros::Publisher pose_pub = nh.advertise<nuslam::pose_error>("turtle_pose",10);




    // parameters
    float wheel_base, wheel_radius;
    double slip_max, slip_min;
    std::vector<double> x_pos, y_pos;
    double robot_radius = 0.12;
    double obstacle_radius;
    double max_range;
    double tube_var;
    double noise_mean = 0.0;
    double noise_var = 0.01;
    
    double samples, angle_increment;
    double Min, Max;
    double resolution;
    double noise;
    double wall_size;

    // read parameters
    nh.getParam("/left_wheel_joint", left_wheel_joint);
    nh.getParam("/right_wheel_joint",right_wheel_joint);
    nh.getParam("/wheel_base", wheel_base);
    nh.getParam("/wheel_radius",wheel_radius);
    nh.getParam("/slip_max",slip_max);
    nh.getParam("/slip_min",slip_min);
    nh.getParam("/x", x_pos);
    nh.getParam("/y", y_pos);
    nh.getParam("/r", obstacle_radius);
    nh.getParam("/max_range",max_range);
    nh.getParam("/tube_var",tube_var);
    
    nh.getParam("/Min",Min);
    nh.getParam("/Max",Max);
    nh.getParam("/resolution",resolution);
    nh.getParam("/angle_increment",angle_increment);
    nh.getParam("/samples",samples);
    nh.getParam("/noise",noise);
    nh.getParam("/wall_size",wall_size);
    
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

    // diff_drive robot
    rigid2d::DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // diff_drive turtle
    rigid2d::DiffDrive diffDrive_fake(pose,wheel_base,wheel_radius);
    
    // define fixed frequency
    int freq = 10;                                                         //Hz
    ros::Rate loop_rat(freq);
    
    // Timing
    ros::Time cur_time, last_time;
    cur_time = ros::Time::now();
    last_time = ros::Time::now();

        
    // turtle path
    nav_msgs::Path turtle_path;
    turtle_path.header.stamp = ros::Time::now();
    turtle_path.header.frame_id = "map";

    // ros::Rate r(5.0);
    
    while (nh.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();
        // visualize cylinders
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.resize(x_pos.size());
        for (unsigned int i=0;i<x_pos.size();++i)
        {
            marker_array.markers[i].header.frame_id = "world";
            marker_array.markers[i].header.stamp = ros::Time::now();
            marker_array.markers[i].lifetime = ros::Duration(1.0/5.0);
            marker_array.markers[i].ns = "real";
            marker_array.markers[i].id = i;
            
            // markers type
            marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
            marker_array.markers[i].action = visualization_msgs::Marker::ADD;

            // markers pose
            marker_array.markers[i].pose.position.x = x_pos[i];
            marker_array.markers[i].pose.position.y = y_pos[i];
            marker_array.markers[i].pose.position.z = 0.3;
            marker_array.markers[i].pose.orientation.w = 1.0;

            // marker scale
            marker_array.markers[i].scale.x = 2 * obstacle_radius;
            marker_array.markers[i].scale.y = 2 * obstacle_radius;
            marker_array.markers[i].scale.z = 0.6;

            marker_array.markers[i].color.r = 0.0;
            marker_array.markers[i].color.g = 0.0;        
            marker_array.markers[i].color.b = 1.0;
            marker_array.markers[i].color.a = 1.0;         

        }
        marker_arrary_pub.publish(marker_array);

        // received cmd_vel message
        if (flag)
        {

            // scaled twist based on frequency
            rigid2d::Twist2D new_cmd;
            // add noise to twist
            std::normal_distribution<> d(noise_mean,noise_var);
            double omg = cmd.w;
            double vx = cmd.vx;
            if(omg != 0)
            {
                double noise_omg = d(get_random());
                omg = omg + noise_omg;
            }
            if(vx != 0)
            {
                double noise_vx = d(get_random());
                vx = vx + noise_vx;
            }
            

            new_cmd.vx = vx * 1.0 / freq;
            new_cmd.vy = 0.0;
            new_cmd.w = omg * 1.0 / freq;

            // robot
            diffDrive.feedforward(new_cmd);
            // turtle
            diffDrive_fake.feedforward(new_cmd);

            // wheel encoders(robot)
            rigid2d::WheelEncoders enc_robot;
            enc_robot = diffDrive.getEncoders();

            // joint_state (robot)
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = cur_time;
            joint_state.name.push_back(left_wheel_joint);
            joint_state.name.push_back(right_wheel_joint);

            // pose
            joint_state.position.push_back(enc_robot.left);
            joint_state.position.push_back(enc_robot.right);

            // publish joint state
            joint_pub.publish(joint_state);

            // wheel ecnoders (turtle)
            rigid2d:: WheelEncoders enc_turtle;
            enc_turtle = diffDrive_fake.getEncoders();   
            
            // add noise to encoders
            std::uniform_real_distribution<> d_slip(slip_min,slip_max);
            double enc_left = enc_turtle.left;
            double enc_right = enc_turtle.right;
            if (enc_left != 0)
            {
                enc_left *= d_slip(get_random()) + 1;
            }
            if (enc_right != 0)
            {
                enc_right *= d_slip(get_random()) + 1;
            }

            // update odometery (turtle)
            diffDrive_fake.updateOdom(enc_left,enc_right);

            // get robot pose(turtle)
            pose = diffDrive_fake.pose();

            // publish turtle_pose
            nuslam::pose_error tr_pose;
            tr_pose.x_error = pose.x;
            tr_pose.y_error = pose.y;
            tr_pose.theta_error = pose.theta;
            pose_pub.publish(tr_pose);

            // check collison
            // simulate robot as a sphere
            visualization_msgs::MarkerArray turtle_array;
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "fake_slip";
            marker.id = 0;
            
            // marker type
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            // marker pose
            marker.pose.position.x = pose.x;
            marker.pose.position.y = pose.y;
            marker.pose.position.z = 0.04;
            marker.pose.orientation.w = 1.0;
            
            // marker scale
            marker.scale.x = 2.0 * robot_radius;
            marker.scale.y = 2.0 * robot_radius;
            marker.scale.z = 2.0 * robot_radius;
            
            // color
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            turtle_array.markers.push_back(marker);
            sphere_pub.publish(turtle_array);

            // ROS_INFO("publish turtle circle");
            
            // check collision
            for (unsigned int i=0; i<x_pos.size();++i)
            {
                double distance = std::sqrt(std::pow(x_pos[i]-pose.x,2) + std::pow(y_pos[i]-pose.y,2));
                if (distance <= robot_radius+obstacle_radius)
                {
                    // ROS_INFO("detect collision");
                    double dist_x = x_pos[i] - pose.x;
                    double dist_y = y_pos[i] - pose.y;
                    double angle = std::atan2(dist_y,dist_x);
                    double sign = - angle / std::fabs(angle);
                    double tx = sign * distance * sin(angle);
                    double ty = sign * distance * cos(angle);
                    double t_omg = distance * atan2(ty,tx);

                    rigid2d::Twist2D t;
                    t.vx = 0.5 * tx;
                    t.w = 0.5 * t_omg;
                    // update turtle
                    diffDrive_fake.feedforward(t);

                    break;

                }
            }


            // convert yaw to quaternion
            tf2::Quaternion q;
            q.setRPY(0,0,pose.theta);
            geometry_msgs::Quaternion turtle_quat;
            turtle_quat = tf2::toMsg(q);

            // broadcast transform from world to turtle
            geometry_msgs::TransformStamped turtle_tf;
            turtle_tf.header.stamp = ros::Time::now();
            turtle_tf.header.frame_id = "world";
            turtle_tf.child_frame_id = "turtle";
            
            turtle_tf.transform.translation.x = pose.x;
            turtle_tf.transform.translation.y = pose.y;
            turtle_tf.transform.translation.z = 0.0;

            turtle_tf.transform.rotation = turtle_quat;
            
            // send transform
            turtle_br.sendTransform(turtle_tf);

            // transform from world to map
            geometry_msgs::TransformStamped tf_wm;
            tf_wm.header.frame_id = "world";
            tf_wm.header.stamp = ros::Time::now();
            tf_wm.child_frame_id = "map";
            tf_wm.transform.translation.x = 0.0;
            tf_wm.transform.translation.y = 0.0;
            tf_wm.transform.translation.z = 0.0;
            tf_wm.transform.rotation.x = 0.0;
            tf_wm.transform.rotation.y = 0.0;
            tf_wm.transform.rotation.z = 0.0;
            tf_wm.transform.rotation.w = 1.0;
            turtle_br.sendTransform(tf_wm);


            // turtle posestamped
            geometry_msgs::PoseStamped turtle_pose;
            turtle_pose.header.stamp = ros::Time::now();
            turtle_pose.header.frame_id = "map";
            turtle_pose.pose.position.x = pose.x;
            turtle_pose.pose.position.y = pose.y;
            turtle_pose.pose.position.z = 0.0;
            turtle_pose.pose.orientation = turtle_quat;

            turtle_path.poses.push_back(turtle_pose);
            path_pub.publish(turtle_path);
            // ROS_INFO("publish path");

            map.header.frame_id = "map";
            map.header.stamp = ros::Time::now();
            map.cx.clear();
            map.cy.clear();

            // fake sensor detection
            visualization_msgs::MarkerArray sensor_array;
            sensor_array.markers.resize(x_pos.size());
            for (unsigned int i=0;i<x_pos.size();++i)
            {
                sensor_array.markers[i].header.frame_id = "world";
                sensor_array.markers[i].header.stamp = ros::Time::now();
                sensor_array.markers[i].lifetime = ros::Duration(1.0/5.0);
                sensor_array.markers[i].ns = "fake_sensed";
                sensor_array.markers[i].id = i;
                
                // marker type
                sensor_array.markers[i].type = visualization_msgs::Marker::CYLINDER;

                double dist = std::sqrt(std::pow((x_pos[i] - pose.x),2) + std::pow((y_pos[i] - pose.y),2));

                // add markers if distance is in range
                if (dist <= max_range)
                {
                    sensor_array.markers[i].action = visualization_msgs::Marker::ADD;

                    // add gussian noise
                    std::normal_distribution<> d2(0,tube_var);
                    double noise_x = d2(get_random());
                    double noise_y = d2(get_random());

                    // markers pose
                    sensor_array.markers[i].pose.position.x = x_pos[i]+noise_x;
                    sensor_array.markers[i].pose.position.y = y_pos[i]+noise_y;
                    sensor_array.markers[i].pose.position.z = 0.3;
                    sensor_array.markers[i].pose.orientation.w = 1.0;

                    // marker scale
                    sensor_array.markers[i].scale.x = 2 * obstacle_radius;
                    sensor_array.markers[i].scale.y = 2 * obstacle_radius;
                    sensor_array.markers[i].scale.z = 0.6;

                    sensor_array.markers[i].color.r = 1.0;
                    sensor_array.markers[i].color.g = 0.0;        
                    sensor_array.markers[i].color.b = 0.0;
                    sensor_array.markers[i].color.a = 1.0;
                    
                    // get fake measurement from landmark to robot

                    rigid2d::Vector2D v;
                    v.x = x_pos[i]+noise_x;
                    v.y = y_pos[i]+noise_y;
                    // map to landmark
                    rigid2d::Transform2D Tml(v);

                    // robot to map Transform
                    rigid2d::Vector2D v2;
                    v2.x = pose.x;
                    v2.y = pose.y;
                    rigid2d::Transform2D Tmr(v2, pose.theta);

                    // get landmark in robot frame
                    rigid2d::Transform2D Trl = Tmr.inv() * Tml;

                     // get Trl data
                     rigid2d::TransformData2D pose_landmarkToRobot;
                     pose_landmarkToRobot = Trl.displacement();

                    map.cx.push_back(pose_landmarkToRobot.x);
                    map.cy.push_back(pose_landmarkToRobot.y);
                }
                else
                {
                    sensor_array.markers[i].action = visualization_msgs::Marker::DELETE;
                    
                    // push NAN of not in radius
                    map.cx.push_back(NAN);
                    map.cy.push_back(NAN);
                }

            }
            sensor_pub.publish(sensor_array);

            fake_measure_pub.publish(map);
            
        }
        //#########################################################################################
        
        // laserscan
        // gussian noise
        std::normal_distribution<> nd(0,noise);
        double gau_noise = nd(get_random());

        // laser scan
        sensor_msgs::LaserScan scan;
        scan.header.frame_id = "turtle";
        scan.header.stamp = ros::Time::now();
        scan.range_max = Max;
        scan.range_min = Min;
        scan.angle_increment = 6.28 / samples;
        scan.angle_min = 0;
        scan.angle_max = 6.28;
        scan.time_increment = 0.00002985;

        scan.ranges.reserve(samples);

        for (unsigned int i=0;i<samples;++i)
        {
            double angle = angle_increment * i;
            angle = std::fmod(angle,rigid2d::PI/2);

            double r,x,y;
            r = wall_size;

            if (angle<=rigid2d::PI/4)
            {
                r = wall_size/cos(angle);
            }
            else
            {
                r = wall_size/sin(angle);
            }
            if (i<=90)
            {
                x = r * cos(angle);
                y = r * sin(angle);
            }
            else if (i<=180)
            {
                x = -r * sin(angle);
                y = r * cos(angle); 
            }
            else if (i<=270)
            {
                x = -r * cos(angle);
                y = -r * sin(angle);
            }
            else
            {
                x = r * sin(angle);
                y = -r * cos(angle);
            }

            double range;
            range = std::sqrt(std::pow(x,2) + std::pow(y,2)) + gau_noise;
            // range = r + gau_noise;

            for (unsigned int i=0;i<x_pos.size();++i)
            {
                //robot pose to landmark
                rigid2d::Vector2D V_lm(x_pos[i],y_pos[i]);
                rigid2d::Transform2D Tml(V_lm);
                rigid2d::Vector2D V_r(pose.x,pose.y);
                rigid2d::Transform2D Tmr(V_r,pose.theta);
                rigid2d::Transform2D Tlr = Tml.inv() * Tmr;
                
                rigid2d::TransformData2D pose_robotToLandmark = Tlr.displacement();

                // range pose to landmark
                rigid2d::Vector2D V_laser(x,y);
                rigid2d::Transform2D T_robotlaser(V_laser,rigid2d::normalize_angle(angle));
                rigid2d::Transform2D T_lmLaser = Tlr * T_robotlaser;

                rigid2d::TransformData2D pose_laserToLandmark = T_lmLaser.displacement();


                // turtle
                rigid2d::Vector2D v1;
                v1.x = pose_robotToLandmark.x;
                v1.y = pose_robotToLandmark.y;
                // ROS_INFO_STREAM("v1.x: "<<v1.x);
                // ROS_INFO_STREAM("v1.y: "<<v1.y);

                // laser
                rigid2d::Vector2D v2;
                v2.x = pose_laserToLandmark.x;
                v2.y = pose_laserToLandmark.y;
                // ROS_INFO_STREAM("v2.x: "<<v2.x);
                // ROS_INFO_STREAM("v2.y: "<<v2.y);
                // ROS_INFO_STREAM("angle"<<rigid2d::rad2deg(angle));

                bool inter = intersect(v1,v2,0.08);
                // if intersect
                if (inter)
                { 
                    // intersection
                    double dx = v2.x - v1.x;
                    double dy = v2.y - v1.y;
                    double dr = std::sqrt(dx*dx + dy*dy);
                    double D = v1.x*v2.y - v2.x*v1.y;
                    double delta = 0.08*0.08 * dr*dr - D*D;
                    // if (delta > 0)
                    // {
                    //     intersection = true;
                    // }
                    double sign = 1;
                    if(dy < 0)
                    {
                        sign = -1;
                    }
                    double x1 = (D*dy - sign*dx*std::sqrt(delta)) / (dr*dr);
                    double y1 = (-D*dx - std::fabs(dy)*std::sqrt(delta)) / (dr*dr);
                    double x2 = (D*dy + sign*dx*std::sqrt(delta)) / (dr*dr);
                    double y2 = (-D*dx + std::fabs(dy)*std::sqrt(delta)) / (dr*dr);
                    
                    double dist1 = std::sqrt(std::pow(x1-v1.x,2.0) + std::pow(y1-v1.y,2.0));
                    double dist2 = std::sqrt(std::pow(x2-v1.x,2.0) + std::pow(y2-v1.y,2.0));
                    // ROS_INFO_STREAM("dist1: "<<dist1);
                    // ROS_INFO_STREAM("dist2: "<<dist2);
                    // ROS_INFO_STREAM("v1.x: "<<v1.x);
                    // ROS_INFO_STREAM("v1.y: "<<v1.y);
                    // ROS_INFO_STREAM("v2.x: "<<v2.x);
                    // ROS_INFO_STREAM("v2.y: "<<v2.y);
                    // ROS_INFO_STREAM("angle"<<rigid2d::rad2deg(angle));
                    double min_dist = std::min(dist1,dist2);

                    // check if tube is in front of the robot
                    bool check = std::sqrt(std::pow(v1.x,2)+std::pow(v1.y,2)) + std::sqrt(std::pow(v2.x,2.0)+std::pow(v2.y,2.0))<=range; 
                    if (check && min_dist<=range)
                    {
                        range = min_dist;
                    }           
                }
            }


            scan.ranges.push_back(range);


        }
        // publish laserscan
        scan_pub.publish(scan);

        last_time = cur_time;
        loop_rat.sleep();
        // r.sleep();

    }

    return 0;
}




/// end file

