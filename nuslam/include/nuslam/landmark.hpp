/// \file landmark.hpp
/// \brief header file for landmarks detection

#ifndef LANDMARK_INCLUDE_GUARD_HPP
#define LANDMARK_INCLUDE_GUARD_HPP

#include<cmath>
#include<vector>
#include<iosfwd>
#include<rigid2d/rigid2d.hpp>

namespace nuslam
{
    using rigid2d::Vector2D;
    

    // \brief store each landmark
    struct  Cluster
    {
        std::vector<Vector2D> points;
        std::vector<double> z;

        double radius = 0.0;            // radius of cluster
        double x_hat = 0.0;             // centroid x
        double y_hat = 0.0;             // centroid y

        double x_bar = 0.0;
        double y_bar = 0.0;
        double z_bar = 0.0;
        
        /// \brief default constructor for cluster
        Cluster(){}
        /// \brief default constructor for cluster
        Cluster(const std::vector<Vector2D> &points): points(points){}
    };
    
    /// \brief distance between two points
    /// \param p1 - point 1
    /// \param p2 - point 2
    /// \return distance
    double pointDistance(const Vector2D p1,const Vector2D p2);

    /// \brief  compose angle in triangle using law of cosine
    /// \param a - length of side a
    /// \param b - length of side b
    /// \param c - length of side c
    /// \return angle between a and c
    double lawCosine(const double a, const double b, const double c);

    /// \brief convert laser range measurement to cartesian coordinates
    /// \param range - range measurement
    /// \param beam_angle - angle of beam
    Vector2D rangeToCartesian(double range, double beam_angle);

    /// \brief compute centroid of the cluster
    /// \return cluster - group of coordinates points    
    void centroid(Cluster &cluster);

    /// \brief shifts the centroid of the cluster to the center
    /// \return cluster - group of coordinates points
    void shiftCentroidToOrigin(Cluster &cluster);

    /// \brief find circle to the cluster
    /// \return cluster - group of coordinates points 
    void composeCircle(Cluster &cluster);



    /// \brief store laser properties
    struct Laserscan
    {
        double beam_min;
        double beam_max;
        double beam_delta;
        double range_min;
        double range_max;


        /// \brief set properties to laserscan
        /// \param beam_min - start angle of the scan
        /// \param beam_max - end angle of the scan
        /// \param beam_delta - increment scan angle
        /// \param range_min - scan min range 
        /// \param range_max - scan max range  
        Laserscan(double beam_min,double beam_max,double beam_delta,
                  double range_min,double range_max)
                  :beam_min(beam_min),
                   beam_max(beam_max),
                   beam_delta(beam_delta),
                   range_min(range_min),
                   range_max(range_max){}

    };

    /// \brief landmark detection and classification
    class Landmark
    {
    public:
        /// \brief constructs feature detector
        /// \param ls - laserscan
        /// \param epsilon - distance threshold between two points 
        Landmark(const Laserscan &ls, double epsilon);

        /// \brief detect circles in laserscan
        /// \brief beam_length - laser scan  
        void featureDetection(const std::vector<float> &beam_length);

        std::vector<Cluster> lm;                                // list of landmark  


    private:
        
        /// \brief converts range and bearing to cartesian coordinates
        /// \param end_points - 2D pointcloud
        /// \param beam_length - laserscan range 
        void laserEndPoints(std::vector<Vector2D> &end_points, const std::vector<float> &beam_length) const;

        /// \brief groups the laserscan into clusters
        /// \param end_points - 2D pointcloud 
        void clusterScan(const std::vector<Vector2D> &end_points);

        /// \brief classify a cluster as a circle or not
        /// \param cluster - group of candidate points
        /// \return true/false
        bool classifyCircles(const Cluster &cluster) const;




        double beam_min,beam_max,beam_delta;                        // laser beam_angles
        double range_min, range_max;                                // laser beam ranges
        
        double epsilon;                                             // distance threshold for clustering
        double radius_thresh;                                       // threshold for radius
        double angle_std;                                           // std of angle
        double mean_min, mean_max;                                  // min and max mean angle of circle 
        unsigned int num_points;                                    // min points per circle

    };




}















#endif
