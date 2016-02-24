#include <aikido/sampleable/TSR.hpp>
#include <aikido/sampleable/TSRSampleGenerator.hpp>

#include <stdexcept>
#include <math.h>
#include <vector>

namespace aikido {
namespace sampleable{

//=============================================================================
TSR::TSR(std::unique_ptr<RNG> rng,
         const Eigen::Isometry3d& T0_w,
         const Eigen::Matrix<double, 6, 2>& Bw,
         const Eigen::Isometry3d& Tw_e)
: mRng(std::move(rng))
, mT0_w(T0_w)
, mTw_e(Tw_e)
, mBw(Bw)
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
}

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> TSR::sampler() const
{
  return std::unique_ptr<SampleGenerator<Eigen::Isometry3d>>
          (new TSRSampleGenerator(
            mT0_w, mBw, mTw_e, mRng->clone(mRng.get())));
}

//=============================================================================
  //TODO: 1) numerical precision
  //      2) for TSRs that cross +/-pi? --> do we still need this?
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

} // namespace sampleable
} // namespace aikido
