/// \file test_diff_drive
/// \brief This is a test file for diff_drive class

#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>
#include<catch_ros/catch.hpp>
#include<iostream>
#include<sstream>

using namespace rigid2d;

/// \brief test for twistToWheels
TEST_CASE("twistToWheels","[diff_drive]")
{
    WheelVelocities vels;
    Twist2D V;
    Pose2D pose;

    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // pure rotation
    V.w = 1.0;
    V.vx = 0.0;
    V.vy = 0.0;
    
    vels = diffDrive.twistToWheels(V);

    REQUIRE(vels.u_left == Approx(-25).margin(0.1));
    REQUIRE(vels.u_right == Approx(25).margin(0.1));
    // pure translation
    V.w = 0.0;
    V.vx = 0.1;
    V.vy = 0.0;

    vels = diffDrive.twistToWheels(V);

    REQUIRE(vels.u_left == Approx(5).margin(0.1));
    REQUIRE(vels.u_right == Approx(5).margin(0.1));

    // rotation & translation
    V.w = 1.0;
    V.vx = 1.0;
    V.vy = 0.0;

    vels = diffDrive.twistToWheels(V);

    REQUIRE(vels.u_left == Approx(25).margin(0.1));
    REQUIRE(vels.u_right == Approx(75).margin(0.1));

}

/// \brief Test for wheelToTwist
TEST_CASE("wheelToTwist","[diff_drive]")
{
    WheelVelocities vels;
    Twist2D V;
    Pose2D pose;

    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;
    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // pure translation
    vels.u_left = 0.1047;
    vels.u_right = 0.1047;

    V = diffDrive.wheelsToTwist(vels);

    REQUIRE(V.w == 0.0);
    REQUIRE(V.vx == 0.002094);
    REQUIRE(V.vy == 0.0);

    // pure rotation
    vels.u_left = 1.0;
    vels.u_right = -1.0;
    
    V = diffDrive.wheelsToTwist(vels);

    REQUIRE(V.w == -0.04);
    REQUIRE(V.vx == 0.0);
    REQUIRE(V.vy == 0.0);

    // rotation and translation
    vels.u_left = 1.0;
    vels.u_right = 0.0;
    
    V = diffDrive.wheelsToTwist(vels);

    REQUIRE(V.w == -0.02);
    REQUIRE(V.vx == 0.01);
    REQUIRE(V.vy == 0.0);   
    
}

/// \brief Test for odometery(translation)
TEST_CASE("pureTransOdom", "[diff_drive]")
{
    WheelVelocities vels;
    Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // both wheel rotate PI/30
    double wheel_angle = rigid2d::PI/30;
    vels = diffDrive.updateOdom(wheel_angle,wheel_angle);

    pose = diffDrive.pose();

    REQUIRE(vels.u_left == Approx(0.1047).margin(0.0001));
    REQUIRE(vels.u_right == Approx(0.1047).margin(0.0001));
    REQUIRE(pose.theta == 0.0);
    REQUIRE(pose.x == Approx(0.0021).margin(0.0001));
    REQUIRE(pose.y == 0.0);    

}


/// \brief Test for  odometery(no movement)
TEST_CASE("noMoveOdom", "[diff_drive]")
{
    WheelVelocities vels;
    Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // both wheel 
    double wheel_angle = 0;
    vels = diffDrive.updateOdom(wheel_angle,wheel_angle);

    pose = diffDrive.pose();

    REQUIRE(vels.u_left == 0.0);
    REQUIRE(vels.u_right == 0.0);
    REQUIRE(pose.theta == 0.0);
    REQUIRE(pose.x == 0.0);
    REQUIRE(pose.y == 0.0);    

}

/// \brief Test for odometery(rotation)
TEST_CASE("pureRotOdom", "[diff_drive]")
{
    WheelVelocities vels;
    Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // wheel angle 
    double left_wheel = -rigid2d::PI/30, right_wheel = rigid2d::PI/30;
    vels = diffDrive.updateOdom(left_wheel,right_wheel);

    pose = diffDrive.pose();

    REQUIRE(vels.u_left == Approx(-0.1047).margin(0.0001));
    REQUIRE(vels.u_right == Approx(0.1047).margin(0.0001));
    REQUIRE(pose.theta == Approx(0.00419).margin(0.0001));
    REQUIRE(pose.x == 0.0);
    REQUIRE(pose.y == 0.0);    

}

/// \brief Test for odometery(rotation and translation)
TEST_CASE("rotTransOdom", "[diff_drive]")
{
    WheelVelocities vels;
    Pose2D pose;
    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    // wheel angle 
    double left_wheel = 0, right_wheel = rigid2d::PI/30;
    vels = diffDrive.updateOdom(left_wheel,right_wheel);

    pose = diffDrive.pose();

    REQUIRE(vels.u_left == 0.0);
    REQUIRE(vels.u_right == Approx(0.1047).margin(0.0001));
    REQUIRE(pose.theta == Approx(0.0021).margin(0.0001));
    REQUIRE(pose.x == Approx(0.00105).margin(0.0001));
    REQUIRE(pose.y == Approx(0.0000109).margin(0.0001));    

}

/// \brief Test feedforward (translation)
TEST_CASE("feedFowardTrans","[diff_drive]")
{
    Twist2D cmd;
    Pose2D pose;

    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;
    
    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    cmd.w = 0.0;
    cmd.vx = 0.01;
    cmd.vy = 0.0;

    diffDrive.feedforward(cmd);

    pose = diffDrive.pose();

    REQUIRE(pose.theta == 0.0);
    REQUIRE(pose.x == Approx(0.01).margin(0.001));
    REQUIRE(pose.y == 0.0);
    
    WheelEncoders enc;
    enc = diffDrive.getEncoders();
    REQUIRE(enc.left == Approx(0.5).margin(0.001));
    REQUIRE(enc.right == Approx(0.5).margin(0.001));
}

/// \brief Test feedforward (rotation)
TEST_CASE("feedFowardRot","[diff_drive]")
{
    Twist2D cmd;
    Pose2D pose;

    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;
    
    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    cmd.w = rigid2d::PI/10;
    cmd.vx = 0.0;
    cmd.vy = 0.0;

    diffDrive.feedforward(cmd);

    pose = diffDrive.pose();

    REQUIRE(pose.theta == Approx(0.3142).margin(0.0001));
    REQUIRE(pose.x == 0.0);
    REQUIRE(pose.y == 0.0);
}


/// \brief Test feedforward in translation and rotation
TEST_CASE("feedFowardTransRot","[diff_drive]")
{
    Twist2D cmd;
    Pose2D pose;

    pose.theta = 0.0;
    pose.x = 0.0;
    pose.y = 0.0;

    double wheel_radius = 0.02, wheel_base = 1.0;
    
    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    cmd.w = PI/4;
    cmd.vx = 1.0;
    cmd.vy = 0.0;

    diffDrive.feedforward(cmd);

    pose = diffDrive.pose();


    REQUIRE(pose.theta == Approx(0.7853).margin(0.0001));
    REQUIRE(pose.x == Approx(0.9003).margin(0.0001));
    REQUIRE(pose.y == Approx(0.3729).margin(0.0001));
}

TEST_CASE("feedforwardOdom", "[diff_drive]")
{
    Twist2D cmd;
    
    Pose2D pose1;
    Pose2D pose2;

    WheelEncoders enc1;
    WheelEncoders enc2;

    WheelVelocities vel1;
    WheelVelocities vel2;

    pose1.theta = 0.0;
    pose1.x = 0.0;
    pose1.y = 0.0;

    pose2 = pose1;

    double wheel_radius = 0.02, wheel_base = 1.0;

    DiffDrive diffDrive1(pose1,wheel_base,wheel_radius);
    DiffDrive diffDrive2(pose2,wheel_base,wheel_radius);

    cmd.w = 0.0;
    cmd.vx = 0.01;
    cmd.vy = 0.0;

    // feedforward twist
    diffDrive1.feedforward(cmd);
    pose1 = diffDrive1.pose();
    enc1 = diffDrive1.getEncoders();
    vel1 = diffDrive1.wheelVelocities();

    // update odom
    vel2 = diffDrive2.updateOdom(enc1.left,enc1.right);
    pose2 = diffDrive2.pose();
    enc2 = diffDrive2.getEncoders();
    vel2 = diffDrive2.wheelVelocities();

    REQUIRE(pose1.theta == Approx(pose2.theta).margin(0.001));
    REQUIRE(pose1.x == pose2.x);
    REQUIRE(pose1.y == pose2.y);
    
    REQUIRE(enc1.left == enc2.left);
    REQUIRE(enc1.right == enc2.right);

    REQUIRE(vel1.u_left == vel2.u_left);
    REQUIRE(vel1.u_right == vel2.u_right);
}

TEST_CASE("pose", "[diff_drive]")
{
    Pose2D pose;
    pose.theta = 0.01;
    pose.x = 0.02;
    pose.y = 0.04;
    double wheel_radius = 0.02, wheel_base = 0.01;

    DiffDrive diffDrive(pose,wheel_base,wheel_radius);

    Pose2D pose_new;
    pose_new = diffDrive.pose();
    
    REQUIRE(pose_new.x == 0.02);
    REQUIRE(pose_new.y == 0.04);
    REQUIRE(pose_new.theta == Approx(0.01).margin(0.001));

}
/// end file
