/// \file
/// \brief This is a implementaion of functions in rigid2d.hpp

#include<iostream>
#include"rigid2d/rigid2d.hpp"

namespace rigid2d
{

// Vector 2D
Vector2D::Vector2D(){
    x = 0.0;
    y = 0.0;
}

Vector2D::Vector2D(double vec_x,double vec_y)
{
    x = vec_x;
    y = vec_y;
}

Vector2D & rigid2d::Vector2D::operator+= (const Vector2D & v)
{
    x += v.x;
    y += v.y;
    return *this;
}

Vector2D & rigid2d::Vector2D::operator-= (const Vector2D & v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

Vector2D & rigid2d::Vector2D::operator*= (const double & scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2D operator+(Vector2D v1, const Vector2D & v2)
{
    v1 += v2;
    return v1;
}

Vector2D operator-(Vector2D v1, const Vector2D & v2)
{
    v1 -= v2;
    return v1;
}

Vector2D operator*(Vector2D v, const double & scalar)
{
    v *= scalar;
    return v;
}

Vector2D operator*(const double & scalar, Vector2D v)
{
    v *= scalar;
    return v;
}

double mag(const Vector2D & v)
{
    return std::sqrt(pow(v.x,2) + pow(v.y,2));
}

double angle(const Vector2D & v1, const Vector2D & v2)
{
    double dot_product = v1.x*v2.x + v1.y*v2.y;
    double theta = std::acos(dot_product/(mag(v1) * mag(v2)));
    return theta; 
}


NormalVec2D norm(const Vector2D & v)
{
    NormalVec2D nvec;
    double mag = sqrt(pow(v.x,2)+pow(v.y,2));
    nvec.nx = v.x / mag;
    nvec.ny = v.y / mag;

    return nvec;
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
    os<<"[" <<v.x<< ", " <<v.y<<"]"<<"\n";
    return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
    std::cout<<"Enter dx: "<<std::endl;
    is>>v.x;
    std::cout<<"Enter dy: "<<std::endl;
    is>>v.y;
    return is;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & twist)
{
    os<<"["<<twist.w<<", "<<twist.vx<<", "<<twist.vy<<"]"<<"\n";
    return os;
}

std::istream & operator>>(std::istream & is, Twist2D & twist)
{
    std::cout<<"Enter angular component: "<<std::endl;
    is>>twist.w;
    std::cout<<"Enter linear velocity x: "<<std::endl;
    is>>twist.vx;
    std::cout<<"Enter linear velocity y: "<<std::endl;
    is>>twist.vy;

    return is;
}  

// Transform 2D
// public

Transform2D::Transform2D()
{
    theta = 0.0;
    ctheta = 1.0;
    stheta = 0.0;
    x = 0.0;
    y = 0.0;
}

Transform2D::Transform2D(const Vector2D & trans)
{   
    // Pure translation 
    x = trans.x;
    y = trans.y;
    theta = 0.0;
    ctheta = 1.0;
    stheta = 0.0; 
}



Transform2D::Transform2D(double radians)
{   
    // Pure rotation
    x = 0.0;
    y = 0.0;
    theta = radians;
    ctheta = cos(theta);
    stheta = sin(theta);
}

Transform2D::Transform2D(const Vector2D & trans, double radians)
{   
    // Rotation and Translation
    x = trans.x;
    y = trans.y;
    theta = radians;
    ctheta = cos(theta);
    stheta = sin(theta);
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    Vector2D v_new;
    v_new.x = ctheta * v.x - stheta * v.y + x;
    v_new.y = stheta * v.x + ctheta * v.y + y;
    return v_new;
}

Twist2D Transform2D::operator()(Twist2D twist) const
{
    Twist2D twist_new;
    twist_new.w = twist.w;
    twist_new.vx = ctheta * twist.vx - stheta * twist.vy + twist.w * y;
    twist_new.vy = stheta * twist.vx + ctheta * twist.vy - twist.w * x;

    return twist_new;
}

Transform2D Transform2D::inv() const
{
    // Create a temp of 2d transform 
    Transform2D temp2d(theta, ctheta, stheta, x, y);
    
    // for transepose, flip stheta
    temp2d.stheta = -temp2d.stheta;
    temp2d.theta = atan2(temp2d.stheta, temp2d.ctheta);

    // p'= -R^T * p
    temp2d.x = -(temp2d.ctheta * x - temp2d.stheta * y);
    temp2d.y = -(temp2d.stheta * x + temp2d.ctheta * y);
    
    return temp2d;
}

Transform2D Transform2D::integrateTwist(const Twist2D & twist) const
{
    // define a screw
    Screw2D S;

    // in Modern Robotics this is theta
    // for 1 unit of time beta = beta_dot
    auto beta = 0.0;

    // compose screw
    if (twist.w != 0.0)
    {
        // beta is the magnitude of the angular twist velocity
        beta = std::abs(twist.w);

        S.w = twist.w / beta;
        S.vx = twist.vx / beta;
        S.vy = twist.vy / beta;
    }

    // all elements of twist are zero
    else if (twist.w==0.0 && twist.vx==0.0 && twist.vy==0.0)
    {
        return *this;
    }

    else
    {
        // beta is the magnitude of the linear twist velocity
        beta = std::sqrt(std::pow(twist.vx, 2) + std::pow(twist.vy, 2));

        // the screw angular component is alread 0
        S.vx = twist.vx / beta;
        S.vy = twist.vy / beta;
    }


    // trig components of beta
    const auto cbeta = std::cos(beta);
    const auto sbeta = std::sin(beta);


    // compose new transform
    // rotation component
    const auto theta_new = std::atan2(sbeta * S.w, 1 + (1 - cbeta)*(-1.0 * std::pow(S.w,2)));

    const auto ctheta_new = std::cos(theta);
    const auto stheta_new = std::sin(theta);

    // translation component
    const auto x_new = S.vx*(beta + (beta - sbeta)*(-1.0 * std::pow(S.w,2))) + S.vy*((1 - cbeta)*(-1.0 * S.w));


    const auto y_new = S.vx*((1 - cbeta) * S.w) + S.vy*(beta + (beta - sbeta)*(-1.0 * std::pow(S.w,2)));



    Transform2D T_new(theta_new, ctheta_new, stheta_new, x_new, y_new);
    Transform2D T(theta, ctheta, stheta, x, y);

    return T*T_new;

}

TransformData2D Transform2D::displacement() const
{
    TransformData2D tf2d;
    tf2d.theta = normalize_angle(this->theta);
    tf2d.x = this->x;
    tf2d.y = this->y;

    return tf2d;
}

// private

Transform2D::Transform2D(double theta, double ctheta, double stheta, double x, double y)
{
    this->theta = theta;
    this->ctheta = ctheta;
    this->stheta = stheta;
    this->x = x;
    this->y = y;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
    x = ctheta * rhs.x - stheta * rhs.y + x;
    y = stheta * rhs.x + ctheta * rhs.y + y;
    theta += rhs.theta;
    ctheta = cos(theta);
    stheta = sin(theta);
    return *this;
}



std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
    os<<"theta (degrees): "<<rad2deg(tf.theta)<<" x: "<<tf.x<<" y: "<<tf.y<<"\n";
    return os;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
    double deg;
    Vector2D v;
    
    std::cout<<"Enter angles in degree: " <<std::endl;
    is>>deg;

    std::cout<<"Enter dx: "<<std::endl;
    is>>v.x;

    std::cout<<"Enter dy: "<<std::endl;
    is>>v.y;

    Transform2D temp(v, deg2rad(deg));
    tf = temp;

    return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}




} // end namespace

// end file


