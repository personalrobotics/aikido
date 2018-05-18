#ifndef AIKIDO_CONSTRAINT_DART_JOINTAVOIDANCESTRATEGY_HPP_
#define AIKIDO_CONSTRAINT_DART_JOINTAVOIDANCESTRATEGY_HPP_

#include "aikido/constraint/dart/IKRankingStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

class JointAvoidanceStrategy : public IKRankingStrategy
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] ikSolutions List of IK solutions to be ranked.
  JointAvoidanceStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const std::vector<statespace::StateSpace::ScopedState> ikSolutions);

protected:
  double evaluateIKSolution(
      statespace::StateSpace::State* solution) const override;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_FIFOSTRATEGY_HPP_
