/// \file  circle_test.cpp
/// \brief test the circle fitting algorithm

#include<cmath>
#include<iostream>
#include<catch_ros/catch.hpp>

#include<rigid2d/rigid2d.hpp>
#include<nuslam/landmark.hpp>

using nuslam::Laserscan;
using nuslam::Landmark;
using rigid2d::Vector2D;
using rigid2d::deg2rad;


TEST_CASE("CircleFitting1","[Landmark]")
{
    // laser properties
    double beam_min = 0.12;
    double beam_max = deg2rad(360.0);
    double beam_delta = deg2rad(0.0);
    double range_min = 0.12;
    double range_max = 3.5;

    Laserscan ls(beam_min,beam_max,beam_delta,range_min,range_max);

    // landmark classifier
    double epsilon = 0.5;
    Landmark Landmark(ls,epsilon);

    Vector2D v1(1.0,7.0);
    Vector2D v2(2.0,6.0);
    Vector2D v3(5.0,8.0);
    Vector2D v4(7.0,7.0);
    Vector2D v5(9.0,5.0);
    Vector2D v6(3.0,7.0);

    nuslam::Cluster cluster;
    cluster.points.push_back(v1);
    cluster.points.push_back(v2);
    cluster.points.push_back(v3);
    cluster.points.push_back(v4);
    cluster.points.push_back(v5);
    cluster.points.push_back(v6);

    centroid(cluster);
    shiftCentroidToOrigin(cluster);
    composeCircle(cluster);

    // check result
    REQUIRE(cluster.x_hat == Approx(4.615842).margin(0.001));
    REQUIRE(cluster.y_hat == Approx(2.807354).margin(0.001));
    REQUIRE(cluster.radius == Approx(4.8275).margin(0.001));
}

TEST_CASE("CircleFitting2","[Landmark]")
{
    // laser properties
    double beam_min = 0.12;
    double beam_max = deg2rad(360.0);
    double beam_delta = deg2rad(0.0);
    double range_min = 0.12;
    double range_max = 3.5;

    Laserscan ls(beam_min,beam_max,beam_delta,range_min,range_max);

    // landmark classifier
    double epsilon = 0.5;
    Landmark Landmark(ls,epsilon);

    Vector2D v1(-1.0,0.0);
    Vector2D v2(-0.3,-0.06);
    Vector2D v3(0.3,0.1);
    Vector2D v4(1.0,0.0);

    nuslam::Cluster cluster;
    cluster.points.push_back(v1);
    cluster.points.push_back(v2);
    cluster.points.push_back(v3);
    cluster.points.push_back(v4);

    centroid(cluster);
    shiftCentroidToOrigin(cluster);
    composeCircle(cluster);

    // check result
    REQUIRE(cluster.x_hat == Approx(0.4908357).margin(0.001));
    REQUIRE(cluster.y_hat == Approx(-22.15212).margin(0.001));
    REQUIRE(cluster.radius == Approx(22.17919).margin(0.001));
}