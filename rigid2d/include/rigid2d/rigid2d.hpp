#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for two-dimensional rigid body transformations.

#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>

namespace rigid2d
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {   
        return std::fabs(d1-d2)<epsilon? true:false;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr double deg2rad(double deg)
    {   
        return deg*PI/180;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {   
        return rad*180/PI;
    }
    
    /// \brief normalize angle and turn any angel into -pi ~ pi
    /// \param rad - radian to normalize
    /// \return normalized angle in radians
    constexpr double normalize_angle(double rad)
    {
      // floating point remainder essentially this is fmod
      const auto q  = std::floor((rad + PI) / (2.0*PI));
      rad = (rad + PI) - q * 2.0*PI;

      if (rad < 0)
      {
        rad += 2.0*PI;
      }

      return (rad - PI);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    // static_assert(almost_equal(0, 0), "is_zero failed");
    // static_assert(almost_equal(3.0, 3.0+1.0e-13),"is_zero failed");

    // static_assert(almost_equal(0.001, 0.005, 1.0e-2), "is_zero failed");
    // static_assert(almost_equal(0.003, 0.008, 1.0e-2), "is_zero failed");

    // static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    // static_assert(almost_equal(deg2rad(90.0), PI/2), "deg2rad failed");

    // static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    // static_assert(almost_equal(rad2deg(PI/3),60.0), "rad2deg) failed");

    // static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    // static_assert(almost_equal(deg2rad(rad2deg(PI/4)), PI/4), "deg2rad failed");


    
    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        double x;
        double y;

        /// \brief default constructor for 2D vector  
        Vector2D();

        /// \brief set elements of vector 
        /// \param vec_x - x component
        /// \param vec_y - y component  
        Vector2D(double vec_x, double vec_y);

        /// \brief add vector components
        /// \param v - components to add
        /// \return a reference to the newly constructed vector
        Vector2D & operator += (const Vector2D & v);

        /// \brief subtract vector components
        /// \param v - components to subtract
        /// \return a reference to the newly constructed vector
        Vector2D & operator -= (const Vector2D & v);

        /// \brief scalar multiplication of vector 
        /// \param scalar - multiply vector by
        /// \return a reference to the newly constructed vector
        Vector2D & operator *= (const double & scalar);

    };

    /// \brief A 2-Dimensional normal vector
    struct NormalVec2D
    {
        double nx = 0.0;
        double ny = 0.0; 

    };

    /// \brief A 2-Dimensional twist
    struct Twist2D
    {
        // rotation about z-axis
        double w = 0.0;
        // linear x velocity
        double vx = 0.0;
        // linear y velocity
        double vy = 0.0;

    };
    
    /// \brief A 2-Dimensional transform
    struct TransformData2D
    {
        double theta = 0.0;
        double x = 0.0;
        double y =0.0;
    };
    

    /// \brief A 2-Dimensional screw
    struct Screw2D
    {
        double w = 0.0;
        double vx = 0.0;
        double vy = 0.0;

    };
    


    /// \brief add two vectors components
    /// \param v1 - vector to add components
    /// \param v2 - added components
    /// \return composition of two vectors 
    Vector2D operator+(Vector2D v1, const Vector2D & v2);

    /// \brief subtract two vectors components
    /// \param v1 - vector to subtract components
    /// \param v2 - subtracted components
    /// \return composition of two vectors 
    Vector2D operator-(Vector2D v1, const Vector2D & v2);

    /// \brief scalar multipliocation of vectors
    /// \param v - vector
    /// \param scalar - multiply vector by
    /// \return scaled vector 
    Vector2D operator*(Vector2D v, const double & scalar);

    /// \brief scalar multipliocation of vector 
    /// \param scalar - scalar
    /// \param v - multiply scalar by vector
    /// \return scaled vector 
    Vector2D operator*(const double & scalar, Vector2D v);

    /// \brief magnitude of a vector
    /// \param v - vector
    /// \return magnitude 
    double mag(const Vector2D & v);

    /// \brief angle between 2 vectors
    /// \param v1 - vector1
    /// \param v2 - vector2
    /// \return angle  
    double angle(const Vector2D & v1, const Vector2D & v2);


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    std::istream & operator>>(std::istream & is, Vector2D & v);


    /// \brief normalize a Vector2D
    /// \param v - the vector to normalize
    /// \return a normalized vector in the same coordinate system
    NormalVec2D norm(const Vector2D & v);

    /// \brief output a  2 dimensional vector as [angular_component vel_xcomponent vel_ycomponent]
    /// os - stream to output to
    /// twist - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & twist);

    /// \brief input a 2 dimensional twist
    ///   You should be able to read vectors entered as two numbers
    ///   separated by a newline or a space, or entered as [xcomponent, ycomponent]
    /// is - stream from which to read
    /// twist [out] - output twist
    std::istream & operator>>(std::istream & is, Twist2D & twist);   
    

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D & trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param rot - the rotation, in radians
        Transform2D(const Vector2D & trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a adjoint to a Twist2D
        /// \param twist - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D twist) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \returns a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);
        
        /// \brief integrate a Twist
        /// \param twist - the twist to integrate
        /// \return transformation correspond to a twist for one time step
        Transform2D integrateTwist(const Twist2D & twist) const;

        /// \brief displacement of transform
        /// \return displacement data of transform
        TransformData2D displacement() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    private:
        // Initialize trans
        Transform2D(double theta, double ctheta, double stheta, double x, double y);
        // angle, sin, cos, x and y
        double theta, ctheta, stheta, x, y;    
    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// dtheta (degrees): 90 dx: 3 dy: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
}

#endif
