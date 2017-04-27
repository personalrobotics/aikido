#include "tsr.hpp"
#include <vector>
#include <stdexcept>
#include <math.h>
#define _USE_MATH_DEFINES
#include <iostream>

TSR::TSR(const Eigen::Isometry3d& T0_w,
        const Eigen::Isometry3d& Tw_e,
        const Eigen::Matrix<double, 6, 2>& bounds,
        int seed)
:T0_w(T0_w), Tw_e(Tw_e), bounds(bounds), seed(seed){
    // assert min, max for bounds 
    for(int i=0;i<6;i++){
        if(bounds(i,0) > bounds(i,1)){
            throw std::invalid_argument("lower bound must be less than upper bound");
        }        
    }

    double lower = -M_PI;
    for(int i=3;i<6;i++){
        for(int j=0;j<2;j++){
            this->bounds(i,j) = fmod(bounds(i,j)-lower, 2*M_PI) + lower;
        }
    }
    generator = std::default_random_engine(seed);

};

const Eigen::Isometry3d TSR::sample(){
    std::vector<std::uniform_real_distribution<double> > distributions;
    for(int i=0;i<6;i++){
        distributions.push_back(std::uniform_real_distribution<double>(bounds(i,0), bounds(i,1)));
    }
    Eigen::Vector3d translation;
    for(int i=0;i<3;i++){
        translation(i) = distributions.at(i)(generator);
    }

    Eigen::Vector3d angles;
    for(int i=0;i<3;i++){
        angles(i) = distributions.at(i+3)(generator);
    }

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitZ())
               *Eigen::AngleAxisd(angles(1),Eigen::Vector3d::UnitY())
               *Eigen::AngleAxisd(angles(2),Eigen::Vector3d::UnitZ());

    Eigen::Isometry3d Tw_s;
    Tw_s.setIdentity();
    Tw_s.translation() = translation;
    Tw_s.linear() = rotation;

    const Eigen::Isometry3d T0_s(T0_w*Tw_s*Tw_e);
    return T0_s;
}
