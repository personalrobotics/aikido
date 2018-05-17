#include <aikido/distance/SO2Angular.hpp>

namespace aikido {
namespace distance {

//==============================================================================
SO2Angular::SO2Angular(std::shared_ptr<statespace::SO2> _space)
  : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr)
  {
    throw std::invalid_argument("SO2 is nullptr.");
  }
}

//==============================================================================
statespace::ConstStateSpacePtr SO2Angular::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
double SO2Angular::distance(
    const aikido::statespace::StateSpace::State* _state1,
    const aikido::statespace::StateSpace::State* _state2) const
{
  // Difference between angles
  double diff = mStateSpace->getAngle(
                    static_cast<const statespace::SO2::State*>(_state1))
                - mStateSpace->getAngle(
                      static_cast<const statespace::SO2::State*>(_state2));
  diff = std::fmod(std::fabs(diff), 2.0 * M_PI);
  if (diff > M_PI)
    diff -= 2.0 * M_PI;
  return std::fabs(diff);
}

} // namespace distance
} // namespace aikido
