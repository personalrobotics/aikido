#ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_JOINTSTATESPACE_H_
#include <dart/dynamics/dynamics.h>
#include "../StateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// StateSpace representing the position of a joint.
class JointStateSpace : public virtual StateSpace
{
public:
  explicit JointStateSpace(::dart::dynamics::Joint* _joint);

  virtual ~JointStateSpace() = default;

  /// The Joint whose StateSpace this class represents.
  ::dart::dynamics::Joint* getJoint() const;

  virtual void getState(
    const Eigen::VectorXd& _positions, StateSpace::State* _state) const = 0;

  virtual void setState(
    const StateSpace::State* _state, Eigen::VectorXd& _positions) const = 0;

  /// Gets the positions of the Joint and store them in _state.
  virtual void getState(StateSpace::State* _state) const;

  /// Sets the Joints's positions to the values stored in _state.
  virtual void setState(const StateSpace::State* _state) const;

protected:
  ::dart::dynamics::Joint* mJoint;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_JOINTSTATESPACE_H_
