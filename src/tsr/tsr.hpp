#include <Eigen/Dense>

class TSR{
public:
    TSR(const Eigen::Isometry3d& T0_w,
        const Eigen::Isometry3d& Tw_e,
        const Eigen::Matrix<double, 6, 2>& bounds);

    
    TSR(const TSR&);
    virtual ~TSR();

    const Eigen::Isometry3d sample(int seed);

private:
    const Eigen::Isometry3d T0_w, Tw_e; 
    Eigen::Matrix<double, 6, 2>& bounds;
    void wrap_to_interval(Eigen::Matrix<double, 3, 2>& angle_bounds);
}