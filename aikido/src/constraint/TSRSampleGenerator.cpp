#include <aikido/constraint/TSR.hpp>
#include <stdexcept>
#include <math.h>
#include <vector>
#include <random>

using aikido::statespace::SE3StateSpace;

namespace aikido {
namespace constraint {

//=============================================================================
TSRSampleGenerator::TSRSampleGenerator(
      std::unique_ptr<util::RNG> _rng,
      std::shared_ptr<SE3StateSpace> _stateSpace,
      const Eigen::Isometry3d& _T0_w,
      const Eigen::Matrix<double, 6, 2>& _Bw,
      const Eigen::Isometry3d& _Tw_e)
  : mRng(std::move(_rng))
  , mStateSpace(std::move(_stateSpace))
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
statespace::StateSpacePtr TSRSampleGenerator::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool TSRSampleGenerator::sample(statespace::StateSpace::State* _state)
{
  using statespace::SE3StateSpace;

  Eigen::Vector3d translation; 
  Eigen::Vector3d angles;

  std::vector<std::uniform_real_distribution<double> > distributions;
  for(int i = 0; i < 6; i++)
  {
    distributions.emplace_back(mBw(i, 0), mBw(i, 1));
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
  mStateSpace->setIsometry(static_cast<SE3StateSpace::State*>(_state), T0_s);

  return true;
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
