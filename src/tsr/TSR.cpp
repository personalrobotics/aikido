#include <aikido/tsr/TSR.hpp>

#include <stdexcept>
#include <math.h>
#include <vector>

using aikido::util::RNG;

namespace aikido {
namespace tsr {

//=============================================================================
TSR::TSR(const Eigen::Isometry3d& T0_w,
         const Eigen::Matrix<double, 6, 2>& Bw,
         const Eigen::Isometry3d& Tw_e)
  : mT0_w(T0_w), mTw_e(Tw_e), mBw(Bw)
{
  // Assertion checks for min, max on bounds 
  for(int i = 0; i < 6; i++)
  {
    if(mBw(i, 0) > mBw(i, 1))
    {
      throw std::invalid_argument(
        "Lower bound must be less than upper bound.");
    }
  }

  // Wrap angles to the interval [-PI, PI)
  double lower = -M_PI;
  for(int i = 3; i < 6; i++)
  {
    for(int j = 0; j < 2; j++)
    {
      mBw(i,j) = fmod(mBw(i, j) - lower, 2*M_PI) + lower;
    }
  }
};

//=============================================================================
const Eigen::Isometry3d TSR::sample(RNG& rng)
{
  std::vector<std::uniform_real_distribution<double> > distributions;
  for(int i = 0; i < 6; i++)
  {
    distributions.push_back(
      std::uniform_real_distribution<double>(mBw(i, 0), mBw(i, 1)));
  }

  Eigen::Vector3d translation;
  for(int i = 0; i < 3; i++)
  {
    translation(i) = distributions.at(i)(rng);
  }

  Eigen::Vector3d angles;
  for(int i = 0; i < 3; i++)
  {
    angles(i) = distributions.at(i+3)(rng);
  }

  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(angles(2), Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(angles(1), Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(angles(0), Eigen::Vector3d::UnitX());

  Eigen::Isometry3d Tw_s;
  Tw_s.setIdentity();
  Tw_s.translation() = translation;
  Tw_s.linear() = rotation;

  Eigen::Isometry3d T0_s(mT0_w * Tw_s * mTw_e);
  return T0_s;
}

} // namespace tsr
} // namespace aikido
