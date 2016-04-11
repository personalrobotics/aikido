#ifndef AIKIDO_STATESPACE_SO3JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SO3JOINTSTATESPACE_H_
#include "SO3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

/// Wrap a single DOF joint in a SO3StateSpace.
class SO3JointStateSpace
  : public SO3StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SO3JointStateSpace>
{
public:
  using SO3StateSpace::State;
  using JointStateSpace::SampleableConstraintPtr;

  explicit SO3JointStateSpace(dart::dynamics::BallJoint* _joint);

  // Documentation inherited.
  void getState(StateSpace::State* _state) const override;

  // Documentation inherited.
  void setState(const StateSpace::State* _state) const override;

  // Documentation inherited.
  SampleableConstraintPtr createSampleableConstraint(
    std::unique_ptr<util::RNG> _rng) const override;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_SO3JOINTSTATESPACE_H_
