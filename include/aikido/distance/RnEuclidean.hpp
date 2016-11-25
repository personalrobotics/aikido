#ifndef AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_

#include "DistanceMetric.hpp"
#include "../statespace/Rn.hpp"

namespace aikido
{
namespace distance
{
/// Implements a Euclidean distance metric
class RnEuclidean : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The Rn this metric operates on
  explicit RnEuclidean(
      std::shared_ptr<statespace::Rn> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes Euclidean distance between two states.
  /// \param _state1 The first state (type Rn::State)
  /// \param _state2 The second state (type Rn::State)
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::Rn> mStateSpace;
};
}
}

#endif
