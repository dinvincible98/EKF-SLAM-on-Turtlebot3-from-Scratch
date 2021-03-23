/// \file  This is a implementation file for diff_drive.hpp

#include<rigid2d/diff_drive.hpp>
#include<stdexcept>
#include<iostream>

namespace rigid2d
{

DiffDrive ::DiffDrive()
{
    // initial params
    theta = 0.0;
    x = 0.0;
    y = 0.0;

    // fixed geometry from nuturtle_description config
    wheel_base = 0.16;
    wheel_radius = 0.033;

    // set wheel encoder angles
    left_cur = 0.0;
    right_cur = 0.0;

    // assume robot starts still
    u_left = 0.0;
    u_right = 0.0;

    // assume timestep = 1.0
    dt = 1.0;

}

DiffDrive::DiffDrive(const Pose2D & pose, double wheel_base, double wheel_radius)
{
    this->theta = pose.theta;
    this->x = pose.x;
    this->y = pose.y;
    this->wheel_base = wheel_base;
    this->wheel_radius = wheel_radius;

    // wheel encoders
    left_cur = 0.0;
    right_cur = 0.0;
    
    // assume robot starts still
    u_left = 0.0;
    u_right = 0.0;

    // assuem dt = 1.0
    dt = 1.0;

}

WheelVelocities DiffDrive::twistToWheels(const Twist2D & twist) const
{
    double d = wheel_base / 2;
    // use Eq 1 from the diff_drive derivation file in doc
    WheelVelocities vels;
    vels.u_left = (1 / wheel_radius) * (-d*twist.w + 1.0*twist.vx);
    vels.u_right = (1 / wheel_radius) * (d*twist.w + 1.0*twist.vx);

    // throw exception
    if (twist.vy != 0)
    {
        throw std::invalid_argument("Twist cannot have y velocity component!");
    }
    return vels;

}

Twist2D DiffDrive::wheelsToTwist(const WheelVelocities & vels) const
{
    // use Eq 2 from the diff_drive derivation file in doc
    Twist2D twist;
    twist.w =  wheel_radius * (-vels.u_left + vels.u_right) / wheel_base;
    twist.vx = wheel_radius * (vels.u_left + vels.u_right) / 2;
    twist.vy = 0.0;
    
    return twist;
}

WheelVelocities DiffDrive::updateOdom(double left, double right)
{
    WheelVelocities vels;
    
    // wheel velocities change in wheel angles
    vels.u_left = normalize_angle(left - left_cur);
    vels.u_right = normalize_angle(right - right_cur);

    // update wheel velocities
    this->u_left = vels.u_left;
    this->u_right = vels.u_right;

    // update wheel angles
    left_cur = normalize_angle(left);
    right_cur = normalize_angle(right);

    // body frame twist given wheel velocities
    Twist2D Vb = wheelsToTwist(vels);

    // integrate twist Tb -> Tb_bprime
    Transform2D Tb_bprime;
    Tb_bprime = Tb_bprime.integrateTwist(Vb);

    // pose is transfrom form world to b prime;
    Vector2D v;
    v.x = this->x;
    v.y = this->y;
    Transform2D Twb(v,this->theta);

    // update pose to b prime
    Transform2D Tw_bprime;
    Tw_bprime = Twb * Tb_bprime;

    // world to robot
    TransformData2D Twr;
    Twr = Tw_bprime.displacement();

    // update pose
    this->theta = normalize_angle(Twr.theta);
    this->x = Twr.x;
    this->y = Twr.y;
    
    return vels;

}

void DiffDrive::feedforward(const Twist2D & cmd)
{
    // wheel velocities to achieve the cmd
    WheelVelocities vel = twistToWheels(cmd);


    // update wheel velocities
    u_left = normalize_angle(vel.u_left);
    u_right = normalize_angle(vel.u_right);

    // update encoder readings
    left_cur = normalize_angle(left_cur + vel.u_left);
    right_cur = normalize_angle(right_cur + vel.u_right);

    // integrate twist
    Transform2D Tb_bprime;
    Tb_bprime = Tb_bprime.integrateTwist(cmd);


    // pose is transform form world to b prime
    Vector2D v;
    v.x = this->x;
    v.y = this->y;
    Transform2D Twb(v, this->theta);

    // update pose to b prime
    Transform2D Tw_bprime = Twb * Tb_bprime;

    // world to robot
    TransformData2D Twr = Tw_bprime.displacement();

    // update pose
    this->theta = Twr.theta;
    this->x = Twr.x;
    this->y = Twr.y;

}

Pose2D DiffDrive::pose() const
{
  Pose2D pose;
  pose.theta = normalize_angle(this->theta);
  pose.x = this->x;
  pose.y = this->y;

  return pose;
}


WheelVelocities DiffDrive::wheelVelocities() const
{
    WheelVelocities vels;
    vels.u_left = this->u_left;
    vels.u_right = this->u_right;
    return vels;
}

void DiffDrive::reset(Pose2D pose)
{
    theta = pose.theta;
    x = pose.x;
    y = pose.y;
}

WheelEncoders DiffDrive::getEncoders() const
{
    WheelEncoders enc;
    enc.left = this->left_cur;
    enc.right = this->right_cur;
    
    return enc;
}


}

// end file