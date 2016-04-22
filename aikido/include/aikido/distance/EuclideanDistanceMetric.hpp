#ifndef AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_

#include "DistanceMetric.hpp"
#include "../statespace/RealVectorStateSpace.hpp"

namespace aikido
{
namespace distance
{
/// Implements a Euclidean distance metric
class EuclideanDistanceMetric : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The RealVectorStateSpace this metric operates on
  explicit EuclideanDistanceMetric(
      std::shared_ptr<statespace::RealVectorStateSpace> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes Euclidean distance between two states.
  /// \param _state1 The first state (type RealVectorStateSpace::State)
  /// \param _state2 The second state (type RealVectorStateSpace::State)
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::RealVectorStateSpace> mStateSpace;
};
}
}

#endif
