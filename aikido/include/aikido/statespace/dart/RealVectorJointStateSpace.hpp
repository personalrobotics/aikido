#ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#include "../RealVectorStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// Wrap a joint with an arbitrary number of DOFs in a RealVectorStateSpace.
class RealVectorJointStateSpace
  : public RealVectorStateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<RealVectorJointStateSpace>
{
public:
  using RealVectorStateSpace::State;

  explicit RealVectorJointStateSpace(::dart::dynamics::Joint* _joint);

  // Documentation inherited.
  void convertPositionsToState(
    const Eigen::VectorXd& _positions,
    StateSpace::State* _state) const override;

  // Documentation inherited.
  void convertStateToPositions(
    const StateSpace::State* _state,
    Eigen::VectorXd& _positions) const override;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
