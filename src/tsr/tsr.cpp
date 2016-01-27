#include "tsr.hpp"
#include <random>
#include <vector>

TSR::TSR(const Eigen::Isometry3d& T0_w,
        const Eigen::Isometry3d& Tw_e,
        const Eigen::Matrix<double, 6, 2>& bounds)
:T0_w(T0_w), Tw_e(Tw_e), bounds(bounds){
    // assert min, max for bounds 


    bounds.block(3,0,3,2) = wrap_to_interval(bounds.blodk(3,0,3,2));

};

const Eigen::Isometry3d TSR::sample(int seed){
    std::default_random_engine generator(seed); 
    std::vector<std::uniform_real_distribution<double> > distributions;
    for(int i=0;i<6;i++){
        distributions.push_back(std::uniform_real_distribution<double>(bound(i,0), bounds(i,1));
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
    rotation = Eigen::AngleAxisd(angles(0), Vector3d::UnitZ())
               *Eigen::AngleAxisd(angles(1),Vector3d::UnitY())
               *Eigen::AngleAxisd(angles(2),Vector3d::UnitZ());



    Eigen::Isometry3d Tw_s();
    Tw_s.translation() = translation;
    Tw_s.rotation() = rotation;

    const Isometry3d T0_s(T0_w*Tw_s*Tw_e);
    return T0_s;
}

void wrap_to_interval(Eigen::Matrix<double, 3, 2>& angle_bounds){

}