#include <aikido/sampleable/TSR.hpp>

#include <stdexcept>
#include <math.h>
#include <vector>

using aikido::util::RNG;

namespace aikido {
namespace sampleable{

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

  if (singlePointTSR())
  {
    mMaxSampleCount = 1;
  }else
  {
    mMaxSampleCount = SampleableRegion::INFTY;
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

  // TODO: decrease sample count if TSR is single point
  if (singlePointTSR())
  {
    mMaxSampleCount -= 1;
  }
  return T0_s;
}

//=============================================================================
bool TSR::isSatisfied(const Eigen::Isometry3d T0_s) const
{
  
  Eigen::Isometry3d Tw_s = mT0_w.inverse()*T0_s*mTw_e.inverse();

  // check if angles are within bounds
  Eigen::Matrix3d rotation = Tw_s.rotation();
  Eigen::Vector3d ea = rotation.eulerAngles(2, 1, 0); // ZYX

  for(int i = 3; i < 6; i++)
  {
    if (mBw(i,0) > ea[i-3] || mBw(i,1) < ea[i-3])
      return false;
  }

  // check if translation are within bounds
  Eigen::Vector3d translation = Tw_s.translation();

  for(int i = 0; i < 3; i++)
  {
    if (mBw(i,0) > translation[i] || mBw(i,1) < translation[i])
      return false;
  }

  return true;
}

//=============================================================================
bool TSR::canSample() const
{
  return mMaxSampleCount > 0;
}

//=============================================================================
int TSR::maxSampleCount() const
{
  return mMaxSampleCount;
}

//=============================================================================
bool TSR::singlePointTSR() const
{
  for(int i = 0; i < 6; i++)
  {
    if(mBw(i, 0) < mBw(i, 1))
    {
      return false;
    }
  }

  return true;

}

} // namespace sampleable
} // namespace aikido
