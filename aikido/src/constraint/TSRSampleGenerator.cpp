#include <aikido/constraint/TSR.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>

namespace aikido {
namespace constraint {

//=============================================================================
TSRSampleGenerator::TSRSampleGenerator(std::unique_ptr<util::RNG> _rng,
                     const Eigen::Isometry3d& _T0_w,
                     const Eigen::Matrix<double, 6, 2>& _Bw,
                     const Eigen::Isometry3d& _Tw_e)
: mRng(std::move(_rng))
, mT0_w(_T0_w)
, mBw(_Bw)
, mTw_e(_Tw_e)
{
  if (!mRng)
  {
    throw std::invalid_argument(
      "Random generator is empty.");
  }
}


//=============================================================================
boost::optional<Eigen::Isometry3d> TSRSampleGenerator::sample()
{

  // Check if all samples are exhausted.
  Eigen::Vector3d translation; 
  Eigen::Vector3d angles;

  std::vector<std::uniform_real_distribution<double> > distributions;
  for(int i = 0; i < 6; i++)
  {
    distributions.push_back(
      std::uniform_real_distribution<double>(mBw(i, 0), mBw(i, 1)));
  }

  for(int i = 0; i < 3; i++)
  {
    translation(i) = distributions.at(i)(*mRng);
  }

  for(int i = 0; i < 3; i++)
  {
    angles(i) = distributions.at(i+3)(*mRng);
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


//=============================================================================
bool TSRSampleGenerator::canSample() const
{
  return true;
}


//=============================================================================
int TSRSampleGenerator::getNumSamples() const
{
  return NO_LIMIT;
}

} // namespace constraint
} // namespace aikido
