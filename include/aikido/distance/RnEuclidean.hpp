#ifndef AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_
#define AIKIDO_DISTANCE_EUCLIDEANDISTANCEMETRIC_HPP_

#include "DistanceMetric.hpp"
#include "../statespace/Rn.hpp"

namespace aikido
{
namespace distance
{
/// Implements a Euclidean distance metric
template <int N>
class REuclidean : public DistanceMetric
{
public:
  /// Constructor.
  /// \param _space The Rn this metric operates on
  explicit REuclidean(std::shared_ptr<statespace::R<N>> _space);

  // Documentation inherited
  statespace::StateSpacePtr getStateSpace() const override;

  /// Computes Euclidean distance between two states.
  /// \param _state1 The first state (type Rn::State)
  /// \param _state2 The second state (type Rn::State)
  double distance(const statespace::StateSpace::State* _state1,
                  const statespace::StateSpace::State* _state2) const override;

private:
  std::shared_ptr<statespace::R<N>> mStateSpace;
};

using R0Euclidean = REuclidean<0>;
using R1Euclidean = REuclidean<1>;
using R2Euclidean = REuclidean<2>;
using R3Euclidean = REuclidean<3>;
using R6Euclidean = REuclidean<6>;
using RnEuclidean = REuclidean<Eigen::Dynamic>;

} // namespace distance
} // namespace aikido

#include "aikido/distance/detail/RnEuclidean-impl.hpp"

#endif
