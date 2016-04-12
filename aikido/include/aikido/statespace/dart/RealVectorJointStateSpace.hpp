#ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
#include "../RealVectorStateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {

/// Wrap a joint with an arbitrary number of DOFs in a RealVectorStateSpace.
class RealVectorJointStateSpace
  : public RealVectorStateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<RealVectorJointStateSpace>
{
public:
  using RealVectorStateSpace::State;

  explicit RealVectorJointStateSpace(dart::dynamics::Joint* _joint);

  // Documentation inherited.
  void getState(StateSpace::State* _state) const override;

  // Documentation inherited.
  void setState(const StateSpace::State* _state) const override;
};

} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_H_
