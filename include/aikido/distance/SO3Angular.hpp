#ifndef AIKIDO_DISTANCE_GEODESICDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_GEODESICDISTANCEMETRIC_HPP_

#include "../statespace/SO3.hpp"
#include "DistanceMetric.hpp"

namespace aikido {
namespace distance {

/// Implements a distance metric on SO(3)
class SO3Angular : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The SO3 this distance metric operates on
  explicit SO3Angular(std::shared_ptr<statespace::SO3> _space);

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Computes distance (in radians) between the two states
  /// \param _state1 The first state (type SO3::State)
  /// \param _state2 The second state (type SO3::State)
  double distance(
      const statespace::StateSpace::State* _state1,
      const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::SO3> mStateSpace;
};

} // namespace distance
} // namespace aikido

#endif
