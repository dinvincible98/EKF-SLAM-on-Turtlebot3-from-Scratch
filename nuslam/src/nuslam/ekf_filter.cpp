/// \file ekf_filter.cpp
/// \brief Extened Kalman Filter implementation filer for ekf_filter.hpp

#include<nuslam/ekf_filter.hpp>
#include<algorithm>
#include<ros/ros.h>
#include<ros/console.h>


namespace nuslam
{


std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    
    return mt;
}


vec sampleStandardNormal(int n)
{
    vec rand_vec = arma::zeros<vec>(n);
    for (auto i=0; i<n; ++i)
    {
        std::normal_distribution<double> d(0,1);
        rand_vec(i) = d(get_random());
    }
    return rand_vec;
}

vec sampleMultivariateDistribution(mat cov)
{
    // must be square
    int dim = cov.n_cols;
    vec rand_vec = sampleStandardNormal(dim);

    // cholesky decompose
    mat L = arma::chol(cov);

    return L * rand_vec;
}


EKF::EKF(int num_landmark,std::vector<double> Q,std::vector<double> R, double md_max,double md_min): 
                                                                                n(num_landmark),
                                                                                md_max(md_max),
                                                                                md_min(md_min),
                                                                                N(0),
                                                                                LM(0)
{
    // total size of state vector (robot + landmark) 
    state_size = 2 * n + 3;
    init_filter(Q,R);
}

void EKF::init_filter(std::vector<double>Q, std::vector<double>R)
{
    // initial state (0,0,0)
    state  = arma::zeros<vec>(state_size);
    
    // initial state covariance
    state_cov = arma::zeros<mat>(state_size,state_size);
    state_cov(0,0) = 0;
    state_cov(1,1) = 0;
    state_cov(2,2) = 0;

    // set landmark to a lager number
    for (auto i=0; i<2*n; i++)
    {
        auto row = i + 3;
        auto col = i + 3;
        state_cov(row,col) = INT_MAX;

    }

    // initial motion noise
    motion_noise = arma::zeros<mat>(3,3);
    motion_noise(0,0) = Q[0];                           // theta var
    motion_noise(1,1) = Q[1];                           // x var
    motion_noise(2,2) = Q[2];                           // y var

    // initial measurement noise (sensor_noise)
    measurement_noise = arma::zeros<mat>(2,2);
    measurement_noise(0,0) = R[0];                       // range var
    measurement_noise(1,1) = R[1];                       // bearing var

    // initial process noise
    process_noise = arma::zeros<mat> (state_size,state_size);
    process_noise(0,0) = motion_noise(0,0);
    process_noise(1,1) = motion_noise(1,1);
    process_noise(2,2) = motion_noise(2,2);

}

void EKF::unknownDataSLAM(const std::vector<Vector2D> & measures, const Twist2D & twist)
{
    // 1. motion model update
    vec state_bar = arma::zeros<vec>(state_size);
    motionUpdate(twist,state_bar);
    // std::cout<<"state_bar"<<state_bar<<std::endl;  
    
    // 2.propagate uncertainty
    mat sigma_bar = arma::zeros<mat> (state_size,state_size);
    uncertaintyUpdate(twist,sigma_bar);
    
    // 3. update state based on observations
    std::vector<Landmark> landmark_measures(measures.size());
    measureRobotToMap(measures, landmark_measures);
    // ROS_INFO_STREAM("lm size: "<<landmark_measures.size());

    for (unsigned int i=0;i<landmark_measures.size();++i)
    {
        Landmark lm = landmark_measures.at(i);
        
        // check if it is outside radius
        if(std::isnan(lm.x) && std::isnan(lm.y))
        {
            continue;
        }
        
        std::vector<double> dist;
        if(N==0)
        {
            dist.push_back(1e12);
        }
        else
        {
            dist.reserve(N);
        }
        // std::cout<<"dist size:"<<dist.size()<<std::endl;

        // 4. compute mahalanobis distance to all landmarks
        // compute  mahalanobis distance to each landmark
        for (int k=0;k<N;++k)
        {
            // predict measurement z_hat (range,bearing)
            vec2 z_hat = predictMeasurement(k, state_bar);
            
            // measurement jacobian
            mat H = arma::zeros<mat>(2,state_size);
            measurementJacobian(k, state_bar, H);

            // compute psi
            mat Psi = H * sigma_bar * arma::trans(H) + measurement_noise;
            
            // std::cout<<"Psi :"<<Psi<<std::endl;
            
            // compute expected measurement
            vec2 delta_z;
            delta_z(0) = lm.range - z_hat(0);
            delta_z(1) = normalize_angle(normalize_angle(lm.bearing) - normalize_angle(z_hat(1)));
            // std::cout<<"delta_z: "<<delta_z<<std::endl;

            // compute mahalanobis distance
            mat z_trans = arma::mat(2,1);
            z_trans(0,0) = delta_z(0);
            z_trans(1,0) = delta_z(1);
            // std::cout<<"z_trans: "<<z_trans<<std::endl;
            mat d = arma::mat(1,1);
            d = arma::trans(z_trans) * arma::inv(Psi) * delta_z;
            dist.push_back(d(0));

            // std::cout<<"dist: "<<d(0)<<std::endl;
        
            if(d(0) < 0)
            {
                throw std::invalid_argument("mahalanobis distance is negative");
            }

        }

        // 5. min mahalanobis distance
        auto j = std::min_element(dist.begin(),dist.end()) - dist.begin();
        // ROS_INFO_STREAM("j: "<<j);
        const auto dstar = dist.at(j);

        // ROS_INFO_STREAM("dstar: "<<dstar);
        
        if(dstar <= md_min || dstar>=md_max)
        {
            // d* below md_min, then update existing landmark
            if(dstar<=md_min)
            {
                ROS_INFO_STREAM("updating existing landmark: "<<j);
            }
            // d* is above md_max, then add new landmark
            else if(dstar>=md_max)
            {
                if((N+1) <= n)
                {
                    j = N;
                    ROS_INFO_STREAM("adding new landmark: "<<j);

                    newLandmark(lm, j, state_bar);

                    landmark_j.push_back(j);
                    
                    N++;
                }
            }
         

            // check if landmark has been added to state before update occurs
            if(std::find(landmark_j.begin(),landmark_j.end(),j) != landmark_j.end())
            {
                // update based on d*(j)
                // 6. perdicted measurement z_hat
                vec2 z_hat = predictMeasurement(j,state_bar);
                // std::cout<<"z_hat: "<<z_hat<<std::endl;
                
                // 7. measurement jacobian
                mat H = arma::zeros<mat> (2,state_size);
                measurementJacobian(j, state_bar, H);
                // std::cout<<"H: "<<H<<std::endl;

                // 8. Kalman Gain

                // std::cout<<"H size: "<<H.size()<<std::endl;

                // std::cout<<"sigma size: "<<sigma_bar.size()<<std::endl;

                mat temp = H * sigma_bar * arma::trans(H) + measurement_noise;
                // std::cout<<"temp: "<<std::endl;
                
                mat temp_inv = arma::inv(temp);

                mat K = arma::zeros<mat>(state_size,2);
                K = sigma_bar * arma::trans(H) * temp_inv;

                // std::cout<<"K: "<<K<<std::endl;
                
                // 9. update state vector 
                vec2 delta_z;
                delta_z(0) = lm.range - z_hat(0);
                delta_z(1) = normalize_angle(normalize_angle(lm.bearing) - normalize_angle(z_hat(1)));
                // std::cout<<delta_z<<std::endl;

                state_bar += K * delta_z;
                // std::cout<<"state_bar: "<<state_bar<<std::endl;

                // 10. update covariance sigma bar
                mat I = arma::eye<mat>(state_size,state_size);
                sigma_bar = (I - (K * H)) * sigma_bar;

            }   // end if landark added
        }   // end if update / add landmark
    }   // end outer loop
    
    // 11. update state vector and covariance
    state = state_bar;
    state_cov = sigma_bar;
}

void EKF::knownDataSLAM(const std::vector<Vector2D> &measures, const Twist2D &twist)
{
    // std::cout<<twist.w<<std::endl;
    // 1. motion model update
    vec state_bar = arma::zeros<vec>(state_size);
    motionUpdate(twist,state_bar);
    // std::cout<<state_bar<<std::endl;


    // 2.propagate uncertainty
    mat sigma_bar = arma::zeros<mat> (state_size,state_size);
    uncertaintyUpdate(twist,sigma_bar);
    // std::cout<<sigma_bar<<std::endl;

    // 3. update state based on observations
    // measurements comes in as (x,y) in robot frame
    // convert them to range and bearing
    // convert landmark position in robot frame and map frame

    // std::cout<<measures.size()<<std::endl;
    std::vector<Landmark> landmark_measures(measures.size());
    measureRobotToMap(measures, landmark_measures);

    // std::cout<<landmark_measures.size()<<std::endl;

    // for every landmark
    for (unsigned int i=0;i<landmark_measures.size();++i)
    {
        // new measurement
        Landmark lm = landmark_measures.at(i);

        // check if it is outside radius
        if(std::isnan(lm.x) && std::isnan(lm.y))
        {
            continue;
        }

        // find correspondence id is the idx the measurement comes in
        int j = i;
        // std::cout<<j<<std::endl;

        // landmark has not been sensed before
        if(std::find(landmark_j.begin(),landmark_j.end(),j) == landmark_j.end())
        {
            landmark_j.push_back(j);
            newLandmark(lm, j, state_bar);
        }
        // std::cout<<state_bar<<std::endl;
        // std::cout<<"-----------------------------------"<<std::endl;
        // // state_bar(3) = -1.0;
        // // state_bar(4) = 0.0;

        // 4. predict measurement z_hat (range,bearing)
        vec2 z_hat = predictMeasurement(j, state_bar);
        // std::cout<<z_hat<<std::endl;

        // 5. measurement jacobian
        mat H = arma::zeros<mat>(2,state_size);
        measurementJacobian(j, state_bar, H);
        // std::cout<<H<<std::endl;

        // 6. calculate Kalman gain
        mat temp = arma::zeros<mat>(2,2);
        temp = H * sigma_bar * arma::trans(H) + measurement_noise;
        
        mat temp_inv = arma::inv(temp);

        mat K = arma::zeros<mat>(state_size,2);
        K = sigma_bar * arma::trans(H) * temp_inv;

        // std::cout<<K<<std::endl;

        // 7. update posterior state vector
        // difference in measurement delta_z (range,bearing)
        vec2 delta_z;
        delta_z(0) = lm.range - z_hat(0);
        delta_z(1) = normalize_angle(normalize_angle(lm.bearing) - normalize_angle(z_hat(1)));
        // std::cout<<delta_z<<std::endl;

        state_bar += K * delta_z;
        // std::cout<<state_bar<<std::endl;

        // 8. update covariance sigma bar
        mat I = arma::eye<mat>(state_size,state_size);
        sigma_bar = (I - (K * H)) * sigma_bar;    
    
    }// end loop

    // 9. update state vector and covariance
    state = state_bar;
    state_cov = sigma_bar;
    // std::cout<<"theta: "<<state(0)<<std::endl;

}


void EKF::motionUpdate(const Twist2D &twist,vec &state_bar) const
{
    // set estimated state equal to current state and then update it
    state_bar = state;

    // sample noise
    // vec wt = sampleMultivariateDistribution(motion_noise);
    // ROS_INFO_STREAM("wt"<<wt);

    // update robot pose based on odometery
    if (almost_equal(twist.w,0.0))
    {
        // update theta
        state_bar(0) = normalize_angle(state_bar(0) + motion_noise(0,0));
        // update x
        state_bar(1) += twist.vx * std::cos(state_bar(0)) + motion_noise(1,1);
        // std::cout<<state_bar(1)<<std::endl;
        // update y
        state_bar(2) += twist.vx * std::sin(state_bar(0)) + motion_noise(2,2);   
    } 
    else
    {
        // update theta
        state_bar(0) = normalize_angle(state_bar(0) + twist.w + motion_noise(0,0));
        // std::cout<<"angular: "<<twist.w<<std::endl;
        // std::cout<<"theta :"<<rigid2d::rad2deg(state_bar(0))<<std::endl;

        // update x
        state_bar(1) += -(twist.vx/twist.w) * std::sin(state_bar(0)) + 
                        (twist.vx/twist.w) * std::sin(state_bar(0) + twist.w) + motion_noise(1,1);
        // update y
        state_bar(2) += (twist.vx/twist.w) * std::cos(state_bar(0)) - 
                        (twist.vx/twist.w) * std::cos(state_bar(0) + twist.w) + motion_noise(2,2);
    }
}

void EKF::uncertaintyUpdate(const Twist2D &twist, mat &sigma_bar) const
{
    mat I = arma::eye<mat>(state_size,state_size);
    // jacobian of motion model
    mat A = arma::zeros<mat> (state_size,state_size);

    if (almost_equal(twist.w,0.0))
    {
        A(1,0) = -twist.vx * std::sin(state(0));
        A(2,0) = twist.vx * std::cos(state(0));
    }
    else
    {
        A(1,0) = (-twist.vx/twist.w) * std::cos(state(0)) +
                 (twist.vx/twist.w) * std::cos(state(0) + twist.w);
        A(2,0) = (-twist.vx/twist.w) * std::sin(state(0)) +
                 (twist.vx/twist.w) * std::sin(state(0) + twist.w);
    }

    // add I to A
    A += I;
    // predicted covariance
    sigma_bar = A * state_cov * arma::trans(A) + process_noise;
    // ROS_INFO_STREAM("sigma_bar"<<sigma_bar);  

}

void EKF::measurementJacobian(const int j, const vec &state_bar, mat &H) const
{
    const auto jx = 2 * j + 3;
    const auto jy = 2 * j + 4;

    const auto delta_x = state_bar(jx) - state_bar(1);
    const auto delta_y = state_bar(jy) - state_bar(2);

    const auto delta_d = delta_x * delta_x + delta_y * delta_y;
    const auto sqrt_delta_d = std::sqrt(delta_d);

    // row1
    H(0,0) = 0.0;
    H(0,1) = -delta_x / sqrt_delta_d;
    H(0,2) = -delta_y / sqrt_delta_d;
    H(0,jx) = delta_x / sqrt_delta_d;
    H(0,jy) = delta_y / sqrt_delta_d;

    // row2
    H(1,0) = -1.0;
    H(1,1) = delta_y / delta_d;
    H(1,2) = -delta_x / delta_d;
    H(1,jx) = -delta_y / delta_d;
    H(1,jy) = delta_x / delta_d;

    // std::cout<<H<<std::endl;
}

vec2 EKF::predictMeasurement(const int j, const vec &state_bar) const
{
    // index of jth correspondence
    // first 3 indices are (theta,x,y)
    const auto jx = 2 * j + 3;
    const auto jy = 2 * j + 4;

    // change in x landmark to robot
    const auto delta_x = state_bar(jx) - state_bar(1);
    const auto delta_y = state_bar(jy) - state_bar(2);

    // // measurement noise
    // vec v = sampleMultivariateDistribution(measurement_noise);

    arma::vec2 z_hat;

    //predicted range
    z_hat(0) = std::sqrt(delta_x * delta_x + delta_y * delta_y) + measurement_noise(0,0);
    
    // predicted bearing
    z_hat(1) = normalize_angle(std::atan2(delta_y,delta_x) - normalize_angle(state_bar(0) + measurement_noise(1,1)));

    return z_hat;

}


void EKF::measureRobotToMap(const std::vector<Vector2D> &measures, std::vector<Landmark> &landmark_measures) const
{
    for (unsigned int i=0;i<measures.size();++i)
    {
        const auto mx = measures[i].x;
        const auto my = measures[i].y;

        // to polar coordinate range and bearing
        const auto range = std::sqrt(mx * mx + my * my);
        const auto bearing = std::atan2(my, mx);

        landmark_measures[i].range = range;
        landmark_measures[i].bearing = bearing;

        // frame robot -> map
        landmark_measures[i].x = state(1) + range * std::cos(bearing + state(0));
        landmark_measures[i].y = state(2) + range * std::sin(bearing + state(0));

        // std::cout<<"landmark x: "<<landmark_measures[i].x<<std::endl;
        // std::cout<<"landmark y: "<<landmark_measures[i].y<<std::endl;
    }
}

void EKF::newLandmark(const Landmark &landmark, const int j, vec &state_bar) const
{
    // index of jth correspondence
    // first 3 indices are (theta,x,y)
    const auto jx = 2 * j + 3;
    const auto jy = 2 * j + 4;
    // std::cout<<jx<<std::endl;
    // std::cout<<jy<<std::endl;

    // add to existing state
    state_bar(jx) = state_bar(1) + landmark.range * std::cos(landmark.bearing + state_bar(0));
    state_bar(jy) = state_bar(2) + landmark.range * std::sin(landmark.bearing + state_bar(0));
    // std::cout<<state_bar(jx)<<std::endl;
    // std::cout<<state_bar(jy)<<std::endl;
    // std::cout<<"-----------------------------------------"<<std::endl;
}

void EKF::getMap(std::vector<Vector2D> &map) const
{
    map.reserve(n);
    for (auto i=0;i<n;++i)
    {
        const auto jx = 2 * i + 3;
        const auto jy = 2 * i + 4;

        Vector2D marker;
        marker.x = state(jx);
        marker.y = state(jy);

        if (!almost_equal(marker.x, 0.0) and !almost_equal(marker.y, 0.0))
        {
        map.push_back(marker);
        }

    }
}

Transform2D EKF::getRobotState() const
{
    Vector2D V_mr(state(1),state(2));
    Transform2D T_mr(V_mr, state(0));
    return T_mr;
}

}// end namespace













///end file