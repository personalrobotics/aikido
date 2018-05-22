#ifndef AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

class JointAvoidanceConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] ikSolutions List of IK solutions to be ranked.
  JointAvoidanceConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const std::vector<statespace::StateSpace::ScopedState> ikSolutions);

protected:
  double evaluateIKSolution(
      statespace::StateSpace::State* solution) const override;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
