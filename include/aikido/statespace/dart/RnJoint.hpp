#ifndef AIKIDO_STATESPACE_DART_REALVECTORJOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_REALVECTORJOINTSTATESPACE_HPP_
#include "../Rn.hpp"
#include "JointStateSpace.hpp"
#include "aikido/statespace/detail/RnTraits.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c Rn for an arbitrary type of DART \c Joint with an
/// arbitrary number of <tt>DegreeOfFreedom</tt>s. This class treats the
/// joint's positions as a real vector space.
///
/// This may not be appropriate for all \c Joint types, e.g. \c FreeJoint is
/// best modelled as having an SE(3) state space. If you are not sure what type
/// of \c JointStateSpace to for a \c Joint you most likely should use
/// the \c createJointStateSpace helper function.
template <int N>
class RJoint
  : public R<N>
  , public JointStateSpace
  , public std::enable_shared_from_this<RJoint<N>>
{
public:
  static constexpr int DimensionAtCompileTime = N;

  using R<DimensionAtCompileTime>::State;

  using Vectord = typename R<DimensionAtCompileTime>::Vectord;

  using DartJoint = typename detail::RnTraits<N>::DartJoint;

  /// Create a real vector state space for \c _joint.
  ///
  /// \param _joint joint to create a state space for
  explicit RJoint(DartJoint* _joint);

  // Documentation inherited.
  void convertPositionsToState(
    const Eigen::VectorXd& _positions,
    StateSpace::State* _state) const override;

  // Documentation inherited.
  void convertStateToPositions(
    const StateSpace::State* _state,
    Eigen::VectorXd& _positions) const override;
};

using R0Joint = RJoint<0>;
using R1Joint = RJoint<1>;
using R2Joint = RJoint<2>;
using R3Joint = RJoint<3>;
using R6Joint = RJoint<6>;

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "aikido/statespace/dart/detail/RnJoint-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_REALVECTORJOINTSTATESPACE_HPP_
