#ifndef AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
#include "SE2StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

/// Wrap a PlanarJoint in a SE2StateSpace.
class SE2JointStateSpace
  : public SE2StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SE2JointStateSpace>
{
public:
  using SE2StateSpace::State;
  using SE2StateSpace::Isometry2d;
  using JointStateSpace::SampleableConstraintPtr;

  explicit SE2JointStateSpace(dart::dynamics::PlanarJoint* _joint);

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

#endif // ifndef AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
