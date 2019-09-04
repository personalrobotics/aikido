#ifndef AIKIDO_DISTANCE_ANGULARDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_ANGULARDISTANCEMETRIC_HPP_

#include "aikido/statespace/SO2.hpp"
#include "aikido/distance/DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Computes the shortest distance between two angles in SO(2)
class SO2Angular : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The SO2 this distance metric operates on
  explicit SO2Angular(std::shared_ptr<const statespace::SO2> _space);

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Computes shortest distance between two angles. (return value between 0 and
  /// pi)
  /// \param _state1 The first state (type: SO2::State)
  /// \param _state2 The second state (type: SO2::State)
  double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<const statespace::SO2> mStateSpace;
};

} // namespace distance
} // namespace aikido

#endif
