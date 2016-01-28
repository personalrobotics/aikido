#include <Eigen/Dense>
#include <random>

class TSR{
public:
    TSR(const Eigen::Isometry3d& T0_w,
        const Eigen::Isometry3d& Tw_e,
        const Eigen::Matrix<double, 6, 2>& bounds,
        int seed);

    
    TSR(const TSR&);
    virtual ~TSR(){};

    const Eigen::Isometry3d sample();

private:
    const Eigen::Isometry3d T0_w, Tw_e; 
    Eigen::Matrix<double, 6, 2> bounds;
    const int seed;
    std::default_random_engine generator; 

};