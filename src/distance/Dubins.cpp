#include <boost/math/constants/constants.hpp>
#include <aikido/distance/Dubins.hpp>
#include "aikido/planner/ompl/BackwardCompatibility.hpp"
#include <aikido/statespace/DubinsPath.hpp>

namespace aikido {
namespace distance {

using namespace aikido::statespace;

//=============================================================================
Dubins::Dubins(std::shared_ptr<statespace::SE2> _space)
  : mStateSpace(std::move(_space))
  , mTurningRadius(1.0)
  , mIsSymmetric(false)
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("_space is nullptr.");
}

//=============================================================================
Dubins::Dubins(std::shared_ptr<statespace::SE2> _space,
      double turningRadius,
      bool isSymmetric)
  : mStateSpace(std::move(_space))
  , mTurningRadius(turningRadius)
  , mIsSymmetric(isSymmetric)
{
  if (mStateSpace == nullptr)
    throw std::invalid_argument("_space is nullptr.");

  if (mTurningRadius < 0.0)
    throw std::invalid_argument("turning radius is lesser than zero");
}

//=============================================================================
statespace::ConstStateSpacePtr Dubins::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
double Dubins::distance(
    const StateSpace::State* _state1,
    const StateSpace::State* _state2) const
{
  
  auto state1 = static_cast<const SE2::State*>(_state1);
  auto state2 = static_cast<const SE2::State*>(_state2);

  aikido::common::DubinsPath path(*state1, *state2, mTurningRadius);

  return path.length();
}



} // namespace distance
} // namespace aikido
