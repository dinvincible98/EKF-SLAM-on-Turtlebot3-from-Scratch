/// \file landmark.cpp
/// \brief implementation file for landmark.hpp


#include<iostream>
#include<armadillo>
#include<nuslam/landmark.hpp>

namespace nuslam
{

    Vector2D rangeToCartesian(double range,double beam_angle)
    {
        Vector2D point;
        point.x = range * cos(beam_angle);
        point.y = range * sin(beam_angle);
        return point;
    }

    
    double pointDistance(const Vector2D p1, const Vector2D p2)
    {
        const auto dx = p1.x - p2.x;
        const auto dy = p1.y - p2.y;

        return std::sqrt(dx*dx + dy*dy);
    }

    double lawCosine(const double a, const double b, const double c)
    {
        const auto temp = (b*b - a*a - c*c) / (-2.0 * a * c);
        return std::acos(temp);
    }

    void centroid(Cluster &cluster)
    {
        // x center
        const auto x = std::accumulate(cluster.points.begin(),cluster.points.end(),0.0, \
                                      std::bind(std::plus<double>(), std::placeholders::_1, 
                                      std::bind(&Vector2D::x, std::placeholders::_2)));
    
        cluster.x_hat = x / cluster.points.size();

        // y center
        const auto y = std::accumulate(cluster.points.begin(),cluster.points.end(),0.0, \
                                      std::bind(std::plus<double>(), std::placeholders::_1, 
                                      std::bind(&Vector2D::y, std::placeholders::_2)));
    
        cluster.y_hat = y / cluster.points.size();
    }

    void shiftCentroidToOrigin(Cluster &cluster)
    {
        for (auto &point : cluster.points)
        {
            point.x -= cluster.x_hat;
            point.y -= cluster.y_hat;

            const auto xi = point.x;
            const auto yi = point.y;
            const auto zi = xi*xi + yi*yi;
            cluster.z.push_back(zi);
        }
        const auto x_bar = std::accumulate(cluster.points.begin(),cluster.points.end(),0.0, \
                                      std::bind(std::plus<double>(), std::placeholders::_1, 
                                      std::bind(&Vector2D::x, std::placeholders::_2)));
        cluster.x_bar = x_bar/static_cast<double>(cluster.points.size());
        
        const auto y_bar = std::accumulate(cluster.points.begin(),cluster.points.end(),0.0, \
                                      std::bind(std::plus<double>(), std::placeholders::_1, 
                                      std::bind(&Vector2D::y, std::placeholders::_2)));
        cluster.y_bar = y_bar/static_cast<double>(cluster.points.size());

        const auto z_bar =  std::accumulate(cluster.z.begin(), cluster.z.end(), 0.0);
        cluster.z_bar = z_bar/static_cast<double>(cluster.points.size());

    }

    void composeCircle(Cluster &cluster)
    {
        // compose Z matrix
        arma::mat Z(cluster.points.size(),4);
        
        for (unsigned int i=0;i<cluster.points.size();++i)
        {
            Z(i,0) = std::pow(cluster.points[i].x,2) + std::pow(cluster.points[i].y,2);
            Z(i,1) = cluster.points[i].x;
            Z(i,2) = cluster.points[i].y;
            Z(i,3) = 1.0;
        }

        // compose M matrix
        arma::mat M;
        M = arma::trans(Z) * Z;
        M /= cluster.points.size();

        // compose constraint matrix for the hyperaccurate algebraic fit
        arma::mat H(4,4,arma::fill::zeros);
        H(0,0) = 8.0 * cluster.z_bar;
        H(3,0) = 2.0;
        H(0,3) = 2.0;
        H(1,1) = 1.0;
        H(2,2) = 1.0; 

        // compose H^-1
        arma::mat Hinv(4,4,arma::fill::zeros);
        Hinv(0,3) = 0.5;
        Hinv(1,1) = 1.0;
        Hinv(2,2) = 1.0;
        Hinv(3,0) = 0.5;
        Hinv(3,3) = -2.0 * cluster.z_bar;

        // compute SVD of Z
        arma::mat U;
        arma::vec s;
        arma::mat V;
        
        arma::svd(U,s,V,Z);
        arma::mat sigma = arma::diagmat(s);

        // std::cout<<sigma<<std::endl;
        // std::cout<<"-----------------------------------"<<std::endl;
        // get smallest singular value
        const auto sigma_4 = s[3];

        arma::mat A(4,1);
        if (sigma_4 < 1e-12)
        {
            A = V.col(3); 
        }
        else
        {
            arma::mat Y = V * sigma * arma::trans(V);
            arma::mat Q = Y * Hinv * Y;

            // find smallest engivalue of Q
            arma::vec eigval;
            arma::mat eigvec;
            arma::eig_sym(eigval,eigvec,Q);

            // std::cout<<eigval<<std::endl;
            // std::cout<<"-----------------------------------"<<std::endl;
            // std::vector<double> temp;
            
            // find the corresponding idx of eigvector of the smallest postive eigvalue
            int count = 0;           
            for (unsigned int i=0;i<eigval.size();++i)
            {
                if (eigval(i)<=0)
                {
                    count++;
                    continue;
                }
                // temp.push_back(eigval(i));
            }
            // std::cout<<count<<std::endl;
            // std::cout<<"-----------------------------------"<<std::endl;         
            arma::mat A_star = eigvec.col(count); 

            A = arma::solve(Y,A_star);

        }
        const auto a = -A(1) / (2.0 * A(0)); 
        const auto b = -A(2) / (2.0 * A(0));
        const auto R_square = (A(1)*A(1) + A(2)*A(2) - 4.0*A(0)*A(3)) / (4.0 * A(0)*A(0));

        cluster.x_hat += a;
        cluster.y_hat += b;
        cluster.radius = std::sqrt(R_square);  

    }
    
    Landmark::Landmark(const Laserscan &ls, double epsilon)
                      : beam_min(ls.beam_min),
                        beam_max(ls.beam_max),
                        beam_delta(ls.beam_delta),
                        range_min(ls.range_min),
                        range_max(ls.range_max),
                        epsilon(epsilon),
                        radius_thresh(0.1),
                        angle_std(0.15),
                        mean_min(90.0),
                        mean_max(135.0),
                        num_points(4)
    {
    }

    void Landmark::featureDetection(const std::vector<float> &beam_length)
    {
        // convert scan to cartesiam coordinates
        std::vector<Vector2D> end_points;
        laserEndPoints(end_points,beam_length);

        // cluster points
        clusterScan(end_points);

        // std::cout<<lm.size()<<std::endl;
        // fit circles
        for(unsigned int i=0;i<lm.size();++i)
        {
            centroid(lm[i]);
            shiftCentroidToOrigin(lm[i]);
            composeCircle(lm[i]);
            // std::cout<<"x: "<<lm[i].x_hat<<std::endl;
            // std::cout<<"y: "<<lm[i].y_hat<<std::endl;
            // std::cout<<"r: "<<lm[i].radius<<std::endl;
            // std::cout<<"--------------------------------------"<<std::endl;
        }

        // classify circles
        for (unsigned int i=0;i<lm.size();++i)
        {
            // remove if it is not a circle
            if(lm[i].radius>radius_thresh)
            {
                lm.erase(lm.begin()+i);
                i--;
            }
            
            // check is any circle is left
            if (lm.empty())
            {
                return;
            }
            
            // // remove circle with large raidus 
            // if (lm[i].radius > radius_thresh)
            // {
            //     lm.erase(lm.begin() + i);
            //     i--;
            // }
        }
        std::cout<<"landmark num:"<<lm.size()<<std::endl;
        std::cout<<"############################################"<<std::endl;
    }


    void Landmark::laserEndPoints(std::vector<Vector2D> &end_points, const std::vector<float> &beam_length) const
    {
        auto beam_angle = beam_min;
        auto range = 0.0;
        
        // std::cout<<beam_angle<<std::endl;

        // std::cout<<beam_length.size()<<std::endl;
        for (unsigned int i=0;i<beam_length.size();++i)
        {
            range = beam_length[i];

            // std::cout<<range<<std::endl;
            if (range >= range_min && range < range_max)
            {
                // std::cout<<range<<std::endl;

                Vector2D point;
                point = rangeToCartesian(range,beam_angle);
                // std::cout<<"x: "<<point.x<<" y: "<<point.y<<std::endl;
                end_points.push_back(point);
            }

            // update beam angle
            beam_angle += beam_delta;

            // max angle is negative
            if(beam_max<0 && beam_angle<beam_max)
            {
                beam_angle = beam_min;
            }

            // max angle is postive
            else if(beam_max>=0 && beam_angle>beam_max)
            {
                beam_angle = beam_min;
            }
        }
        // std::cout<<"point size"<<end_points.size()<<std::endl;

    }
    
    void Landmark::clusterScan(const std::vector<Vector2D> &end_points)
    {
        // clear current landmark
        lm.clear();
        
        // store points in temp vector
        std::vector<Vector2D> temp_points;
        
        // compare previous and current points
        Vector2D cur_point, prev_point;
        cur_point = end_points[0];
        prev_point = cur_point;

        // std::cout<<cur_point<<std::endl;
        // std::cout<<end_points.size()<<std::endl;
        // iterates end points
        for (unsigned int i=0;i<end_points.size();++i)
        {
            cur_point = end_points[i];
            const auto dist = pointDistance(cur_point,prev_point);
            std::cout<<"dist: "<<dist<<std::endl;

            if (dist <= epsilon)
            {
                temp_points.push_back(end_points[i]);
                // std::cout<<end_points[i]<<std::endl;
            }

            else if (dist > epsilon)
            {
                // create new cluster
                Cluster cluster(temp_points);
                
                // add cluster to landmarks
                lm.push_back(cluster);

                // clear temp list
                temp_points.clear();
            }
            // update prev point
            prev_point = cur_point;
        }

        // add last cluster 
        Cluster cluster(temp_points);
        lm.push_back(cluster);
        // std::cout<<lm.back().x_hat<<std::endl;

        std::cout<<lm.size()<<std::endl;
        // compare last point to first point
        if(lm.size() > 1)
        {
            const auto dist = pointDistance(end_points.front(),end_points.back());

            if (dist <= epsilon)
            {
                lm.front().points.insert(lm.front().points.end(),
                                         lm.back().points.begin(),
                                         lm.back().points.end());

                lm.pop_back();
            }
        }

        // remove cluster less than three points
        for (unsigned int i=0;i<lm.size();++i)
        {
            // std::cout<<"before"<<lm[i].points.size()<<std::endl;
            if(lm[i].points.size() < num_points)
            {
                lm.erase(lm.begin() + i);
                i--;
            }
            // std::cout<<"after"<<lm[i].points.size()<<std::endl;
        }
        
        std::cout<<lm.size()<<std::endl;
        // for (unsigned int i=0;i<lm.size();++i)
        // {
        //     std::cout<<lm[i].points.at(i)<<std::endl;
        //     std::cout<<"-----------------------------------"<<std::endl;
        // }
        // std::cout<<lm[4].points.front()<<std::endl;
        // std::cout<<lm[4].points.back()<<std::endl;
        // std::cout<<"........................................"<<std::endl;
        // std::cout<<lm[1].x_hat<<std::endl;
        // std::cout<<lm[1].y_hat<<std::endl;           

    }

    bool Landmark::classifyCircles(const Cluster &cluster) const
    {
        const Vector2D p_start = cluster.points.front();
        const Vector2D p_end = cluster.points.back();

        // distance between p_start and p_end
        const auto b = pointDistance(p_start,p_end);

        // number of points between the end points (exclude start and end point)
        unsigned int num_inner_points = cluster.points.size()-2;

        // store angle in vector
        std::vector<double> angles;
        angles.reserve(num_inner_points);

        // iterates over all points between front and end points
        for (unsigned int i=1;i<cluster.points.size()-1;++i)
        {
            const Vector2D p = cluster.points[i];
            
            // distance from p_start to p
            const auto c = pointDistance(p_start,p);
            
            // distance from p to p_end
            const auto a = pointDistance(p,p_end);

            // angle
            angles.push_back(lawCosine(a,b,c));

        }
        
        // average angle
        auto mean_angle = std::accumulate(angles.begin(),angles.end(),0.0);
        mean_angle /= num_inner_points;

        // std of angles
        auto sum = 0.0;
        for (const auto angle:angles)
        {
            const auto delta = angle - mean_angle;
            sum += delta * delta;
        }
        const auto sigma = std::sqrt(sum/num_inner_points);

        // if is a circle
        if (sigma < angle_std && mean_angle>=rigid2d::deg2rad(mean_min) 
           && mean_angle<=rigid2d::deg2rad(mean_max))
        {
            return true;
        }
        
        return false; 

    }

} // end namespace


/// end file