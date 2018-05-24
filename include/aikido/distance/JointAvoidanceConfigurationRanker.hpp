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
  JointAvoidanceConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton);

protected:
  double evaluateConfiguration(
      statespace::StateSpace::State* solution) const override;

  // TODO(avk): Maintain indices of unbounded positions instead?
  Eigen::VectorXd mPositionLowerLimits;
  Eigen::VectorXd mPositionUpperLimits;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
