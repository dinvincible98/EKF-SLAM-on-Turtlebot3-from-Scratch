/// \file ekf_filter.hpp
/// \brief Extened Kalman filter header file

#ifndef EKF_FILTER_INCLUDE_GUARD_HPP
#define EKF_FILTER_INCLUDE_GUARD_HPP

#include<armadillo>
#include<cmath>
#include<vector>
#include<rigid2d/rigid2d.hpp>
#include<iosfwd>

namespace nuslam
{
    using arma::mat;
    using arma::vec;
    using arma::vec2;
    using arma::eps;
    using rigid2d::Twist2D;
    using rigid2d::Vector2D;
    using rigid2d::Transform2D;
    using rigid2d::almost_equal;
    using rigid2d::normalize_angle;


    /// \brief get random number
    std::mt19937 & get_random();

    /// \brief handle standard norm
    vec sampleStandardNormal(int n);

    /// \brief handle multivariate distribution
    vec sampleMultivariateDistribution(mat cov);

    struct Landmark
    {
        double x = 0.0;
        double y = 0.0;

        double range = 0.0;
        double bearing = 0.0;
    };

    class EKF
    {
    public:

        /// \brief constructor for EKF
        /// \param num_lm - number of landmark
        /// \param Q - motion noise
        /// \param R - sensor noise
        /// \param md_max - mahalanobis distance threshold for adding new landmark
        /// \param md_min - mahalanobis distance threshold for updating landmark  
        EKF(int num_landmark, std::vector<double> Q, std::vector<double>R, double md_max, double md_min);
        
        /// \brief update the state vector
        /// \param pose - x/y coordinates of landmark in robot frame
        /// \param twist - twist from odometery given wheel velocities
        void knownDataSLAM(const std::vector<Vector2D> &measures, const Twist2D &twsist);

        /// \brief unknown data slam
        /// \param pose - x/y coordinates of landmark in robot frame
        /// \param twist - twist from odometery given wheel velocities
        void unknownDataSLAM(const std::vector<Vector2D> &measures, const Twist2D &twist);

        /// \brief get current robot state
        /// \return transform from map to robot       
        Transform2D getRobotState() const;

        /// \brief get the estimate x and y location of landmark
        /// \param map - vector of landmark position 
        void getMap(std::vector<Vector2D> &map) const;

    private:

        /// \brief initialize state vector, state covariance matrix, motion and sensor model
        ///        noise, and process noise assuming we know where the robot is but dont known 
        ///        where the landmarks are
        void init_filter(std::vector<double>Q, std::vector<double>R);

        /// \brief estimate the robot pose based on odometry
        /// \param twist - twist from odometery given wheel velocities
        /// \param state_bar - estimate state vector
        void motionUpdate(const Twist2D &twist, vec &state_bar) const;

        /// \brief update the uncertainty in the robot pose and for landmark position
        /// \param twist - from odometery given wheel velocity
        /// \param signma_bar -predicted covariacne matrix
        void uncertaintyUpdate(const Twist2D &twist, mat &sigma_bar) const;

        /// \brief  compose the measurement jacobian
        /// \param j - correspondence id
        /// \param state_bar - estimated state vector
        /// \param H - the measurement jacobian
        void measurementJacobian(const int j, const vec &state_bar, mat &H) const;

        /// \brief perdicted range and bearing given the current state vector
        /// \param j - the correspondence id
        /// \param state_bar - estimated state vector
        /// \return the expected range and bearing of a landmark(r,b)
        vec2 predictMeasurement(const int j, const vec &state_bar) const;

        /// \brief Transform landmark x and y position into frame of the map
        /// \param pose - landmark(x,y) and (r,b) in frame of robot
        /// \param lm_pose - landmark pose
        void measureRobotToMap(const std::vector<Vector2D> &measures, std::vector<Landmark> &landmark_measures) const;

        /// \brief initialized a new landmark that has not been observed before
        /// \param lm - measurement of a landmark in the map frame
        /// \param j - correspondence id
        /// \param state_bar - estimate state vector 
        void newLandmark(const Landmark &landmark, const int j, vec &state_bar) const;

        int n;                                                         // max number of landmark, determine state size

        int state_size;                                                // size of state vector
    
        vec state;                                                     // state vector for robot and landmark

        mat state_cov;                                                 // state covariance for robot and landmark

        mat motion_noise;                                              // noise in motion model

        mat measurement_noise;                                         // noise in measurement model

        mat process_noise;                                             // process noise matrix

        std::vector<int> landmark_j;                                   // landmark j in state 

        double md_max,md_min;                                          // max/min mahalanobis distance threshold
    
        int N;                                                         // number of landmark(unknownData)         
        
        int LM;                                                        // number of landmark(unknownData)
    
    };


}// end namespace





#endif
/// end file