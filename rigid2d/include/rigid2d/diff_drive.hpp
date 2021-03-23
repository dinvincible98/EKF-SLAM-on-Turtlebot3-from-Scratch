/// \file  This is a header file for the diff_drive class
/// \brief Library for two-dimensional rigid body for a diff drive robot

#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include<cmath>
#include<iosfwd>
#include<rigid2d/rigid2d.hpp>

namespace rigid2d
{

    /// \brief A 2D dimensional pose
    struct Pose2D
    {
        double theta = 0.0;
        double x = 0.0;
        double y = 0.0;

    };

    /// \brief Wheel Velocity rad/s
    struct WheelVelocities
    {
        double u_left = 0.0;
        double u_right = 0.0;

    };

    /// \brief  wheelencoders
    struct WheelEncoders
    {
        double left = 0.0;
        double right = 0.0;

    };

    class DiffDrive
    {    
    public:

        /// \brief a default constructor creates a robot at (0,0,0) with fixe wheel base and radius
        DiffDrive();

        /// \brief creates a Diff model by specifing the model pose and geometry
        /// \param pose - the current robot position
        /// \param wheel_base - the distance between wheel center
        /// \param wheel_raidus - the radius of the wheel
        DiffDrive(const Pose2D & pose, double wheel_base, double wheel_radius);

        /// \brief determine the wheels velocities required to make the 
        ///        robot move with desired linear and angular velocities
        /// \param twist - the desired twist in body frame of the robot
        /// \return - the wheel velocities
        WheelVelocities twistToWheels(const Twist2D & twist) const;

        /// \brief determine the body twist of the robot from its wheels velocities
        /// \param vels - velocities of wheels, assumed to be held constant for one time unit
        /// \return twist in the original body frame
        Twist2D wheelsToTwist(const WheelVelocities & vels) const;

        /// \brief update the robot's odometery base on the current encoder readings
        /// \param left - the left encoder angle(rad)
        /// \param right - the right encoder angle(rad)
        /// \return the velocities of each wheel(assuming constant wheel velocities in-between updates)
        WheelVelocities updateOdom(double left, double right);

        /// \brief update the odometry of the diff drive robot assuming it follows the given body 
        ///        twist for one time unit
        /// \param cmd - the twist command send to the robot  
        void feedforward(const Twist2D & cmd);

        /// \brief get the current pose of the robot
        Pose2D pose() const;

        /// \brief get the wheels velocities based on the last encoder update
        WheelVelocities wheelVelocities() const;

        /// \brief reset the robot to the given positions and orientation
        void reset(Pose2D pose);

        /// \brief get the current wheel encoder readings
        /// \return the angular position of wheels
        WheelEncoders getEncoders() const;

    private:
        // theta, x, y of the robot pose and its geometry
        double theta, x, y, wheel_base, wheel_radius;
        // left and right current encoder readings
        double left_cur, right_cur;
        // current wheels velocities
        double u_left, u_right;
        // timestep
        double dt;
    };


}



#endif
// end file 