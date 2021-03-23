/// \file
/// \brief This is a test file for Transform2D class
// Compile & run:
// g++ -Wall -Wextra -g -std=c++17 -o test test.cpp rigid2d.cpp
// ./test

#include"rigid2d/rigid2d.hpp"
#include<catch_ros/catch.hpp>
#include<iostream>
#include<sstream>
using namespace rigid2d;

/// \brief Test for vector addition
TEST_CASE("+","[Vector2D]")
{
    Vector2D v, v1, v2;
    v1.x = 3;
    v1.y = 2;
    v2.x = 6;
    v2.y = 4;
    v = v1 + v2;
    REQUIRE(v.x == 9);
    REQUIRE(v.y == 6); 
}

/// \brief Test for vector subtraction
TEST_CASE("-","[Vector2D]")
{
    Vector2D v, v1, v2;
    v1.x = 5;
    v1.y = 6;
    v2.x = 3;
    v2.y = 1;
    v = v1 - v2;
    REQUIRE(v.x == 2);
    REQUIRE(v.y == 5); 
}

/// \brief Test for vector&scalar multiplication
TEST_CASE("*","[Vector2D]")
{
    Vector2D v, v_left, v_right;
    v.x = 3;
    v.y = 2;
    double scalar = 2;

    // Test multiplication from left and right side
    v_left = scalar * v;
    v_right = v * scalar;

    REQUIRE(v_left.x == 6);
    REQUIRE(v_left.y == 4);
    REQUIRE(v_right.x == 6);
    REQUIRE(v_right.y == 4);
}

/// \brief Test case for magnituded of a vector 
TEST_CASE("mag","[Vector2D]")
{
    Vector2D v;
    v.x = 2;
    v.y = 3;
    double res = mag(v);

    REQUIRE(res == Approx(3.605).margin(0.1));

}

/// \brief Test case for angle between 2 vectors 
TEST_CASE("angle","[Vector2D]")
{
    Vector2D v1, v2;
    v1.x = 2;
    v1.y = 3;
    v2.x = 1;
    v2.y = 2;
    double theta = angle(v1,v2);

    REQUIRE(theta == Approx(0.124).margin(0.1));

}

/// \brief Test for normalize Vector2D
TEST_CASE("norm","[Vector2D]")
{
    Vector2D v;
    v.x = 2;
    v.y = 5;
    NormalVec2D nvec = norm(v);

    REQUIRE(nvec.nx == Approx(0.371).margin(0.1));
    REQUIRE(nvec.ny == Approx(0.928).margin(0.1));
}

/// \brief Test for transforming a Vector2D to new frame
TEST_CASE("Vector2D in new frame", "[transform]")
{   
    // vector in frame b
    Vector2D vb;
    vb.x = 2;
    vb.y = 1;
    // define transform Tab
    double angle = PI/4;
    Vector2D v;
    v.x = 3;
    v.y = 2;
    Transform2D Tab(v,angle);
    // transform vb into frame a
    Vector2D va = Tab(vb);
    REQUIRE(va.x == Approx(3.7).margin(0.1));
    REQUIRE(va.y == Approx(4.1).margin(0.1));

}



/// \brief Test for transforming a Twist2D to new frame
TEST_CASE("Twist2D in new frame", "[transform]")
{
    // define twist in frame c
    Twist2D tc;
    tc.w = 1;
    tc.vx = 2;
    tc.vy = 3;
    // define Transform2D Tbc
    double angle = PI/4;        //radians
    Vector2D v;
    v.x = 1;
    v.y = -2;
    Transform2D Tbc(v,angle);
    // transform tc to frame b
    Twist2D tb = Tbc(tc);
    REQUIRE(tb.w == 1.0);
    REQUIRE(tb.vx == Approx(-2.71).margin(0.1));
    REQUIRE(tb.vy == Approx(2.53).margin(0.1));
}


/// \brief Test for inverting a Transform2D
TEST_CASE("inverse", "[transform]") 
{
    // declare desired output
    std::string output = "theta (degrees): -90 x: -1 y: 1\n";
    // define transform2D inv
    double angle = PI/2;          //radians
    Vector2D v;
    v.x = 1;
    v.y = 1;
    Transform2D T(v,angle);
    // inverse T
    Transform2D T_inv;
    T_inv = T.inv();
    // read inverse
    std::stringstream ss_out;
    ss_out<<T_inv;
    REQUIRE(ss_out.str() == output);
}


/// \brief Test for  tf displacement
TEST_CASE("displacement","[transform]")
{
    Vector2D V;
    V.x = 0.01;
    V.y = 2.0;
    Transform2D T(V,PI/2.0);
    TransformData2D tf_data;
    tf_data = T.displacement();

    REQUIRE(tf_data.theta == PI/2.0);
    REQUIRE(tf_data.x == 0.01);
    REQUIRE(tf_data.y == 2.0);

}

/// \brief Test for intergrateTwist(pure translation)
TEST_CASE("integrateTwist_trans","[transform]")
{
    Transform2D T;
    // declare input
    std::string input = "0 0 0";
    std::stringstream ss_in(input);
    ss_in>>T;

    // pure translation
    Twist2D twist;
    twist.w = 0.0;
    twist.vx = 0.002094;
    twist.vy = 0.0;
    
    //integrate
    Transform2D T_twist;
    T_twist = T.integrateTwist(twist);
    TransformData2D T_data;
    T_data = T_twist.displacement();
    REQUIRE(rad2deg(T_data.theta) == Approx(0.0).margin(0.001));
    REQUIRE(T_data.x == Approx(0.00209).margin(0.0001));
    REQUIRE(T_data.y == 0.0);

}

/// \brief Test for intergrateTwist(no move)
TEST_CASE("integrateTwist_zero","[transform]")
{
    Transform2D T;
    // declare input
    std::string input = "90 -1 3";
    std::stringstream ss_in(input);
    ss_in>>T;

    // pure translation
    Twist2D twist;
    twist.w = 0.0;
    twist.vx = 0.0;
    twist.vy = 0.0;
    
    //integrate
    Transform2D T_twist;
    T_twist = T.integrateTwist(twist);
    TransformData2D T_data;
    T_data = T_twist.displacement();
    REQUIRE(rad2deg(T_data.theta) == Approx(90.0).margin(0.01));
    REQUIRE(T_data.x == -1.0);
    REQUIRE(T_data.y == 3.0);

}

/// \brief Test for intergrateTwist(both rotation and translation)
TEST_CASE("integrateTwist","[transform]")
{
    Transform2D T;
    // declare input
    std::string input = "90 -1 3";
    std::stringstream ss_in(input);
    ss_in>>T;

    //translation and rotation
    Twist2D twist;
    std::string input2 = "1 1 1";
    std::stringstream ss_in2(input2);
    ss_in2>>twist;
    
    //integrate
    Transform2D T_twist;
    T_twist = T.integrateTwist(twist);

    TransformData2D T_data;
    T_data = T_twist.displacement();

    //declare desired output

    REQUIRE(rad2deg(T_data.theta) == Approx(147.296).margin(0.01));
    REQUIRE(T_data.x == Approx(-2.301).margin(0.01));
    REQUIRE(T_data.y == Approx(3.381).margin(0.01));

}


/// \brief Test for multiplication of Transform2D
TEST_CASE("multiplication", "[transform]")
{
    // declare desired output
    std::string output = "theta (degrees): 90 x: 2.59808 y: 4.23205\n";
    // Tab
    Vector2D v1;
    v1.x = 1;
    v1.y = 1;
    double d1 = PI/6;   //radians
    Transform2D Tab(v1,d1);
    // Tbc
    Vector2D v2;
    v2.x = 3;
    v2.y = 2;
    double d2 = PI/3;   //radians
    Transform2D Tbc(v2,d2);
    // multiply
    Transform2D Tac = Tab * Tbc;
    // output
    std::stringstream ss_out;
    ss_out<<Tac;
    REQUIRE(ss_out.str() == output);

}

/// \brief Test operator>> for Vector2D input
TEST_CASE("operator>>v","[Vector2D]")
{
    Vector2D v;
    // declare desired input
    std::string input = "1 1";
    std::stringstream ss_in(input);
    // read vector
    ss_in>>v;

    REQUIRE(ss_in.str() == input);
}

/// \brief Test operator<< for Vector2D output
TEST_CASE("operator<<v","[Vector2D]")
{
    Vector2D v;
    // declare input
    std::string input = "1 1";
    // declare desired output
    std::string output = "[1, 1]\n";
    // read vector
    std::stringstream ss_in(input);
    ss_in>>v;
    // output vector
    std::stringstream ss_out;
    ss_out<<v;

    REQUIRE(ss_out.str() == output);
}

/// \brief Test operator>> for Twist2D input
TEST_CASE("operator>>t","[Twist2D]")
{
    Twist2D t;
    // declare desired input;
    std::string input = "3 4 1";
    std::stringstream ss_in(input);
    // read twist
    ss_in>>t;

    REQUIRE(ss_in.str() == input);
}

/// \brief Test operator<< for Twist2D output
TEST_CASE("operator<<t","[Twist2D]")
{
    Twist2D t;
    // declare input;
    std::string input = "3 4 1";
    // declare desired output;
    std::string output = "[3, 4, 1]\n";
    // read twist
    std::stringstream ss_in(input);
    ss_in>>t;
    // output twist
    std::stringstream ss_out;
    ss_out<<t;

    REQUIRE(ss_out.str() == output);
}

/// \brief Test for operator>> for tf input
TEST_CASE("operator>>tf", "[transform]")
{
    Transform2D T;
    // declare desired input
    std::string input = "45 2 2 ";
    std::stringstream ss_in(input);
    // read tf
    ss_in>>T;

    REQUIRE(ss_in.str() == input);
}

/// \brief Test for operator<< for tf output
TEST_CASE("operator<<tf", "[transform]")
{
    Transform2D T;
    // declare input
    std::string input = "90 2 2";
    // declare desired output
    std::string output = "theta (degrees): 90 x: 2 y: 2\n";
    // read input
    std::stringstream ss_in(input);
    ss_in>>T;
    // read output
    std::stringstream ss_out; 
    ss_out<<T;

    REQUIRE(ss_out.str() == output);
}







// end file

