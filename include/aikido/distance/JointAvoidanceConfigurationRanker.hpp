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
  /// Returns cost as negative of distance from position limits.
  double evaluateConfiguration(
      statespace::CartesianProduct::State* solution) const override;

  /// Vector of indices corresponding to unbounded lower position limits.
  std::vector<std::size_t> mUnboundedLowerLimitsIndices;

  /// Vector of indices corresponding to unbounded upper position limits.
  std::vector<std::size_t> mUnboundedUpperLimitsIndices;

  /// (Modified) State corresponding to the lower position limits.
  /// The positions at the indices in mUnboundedLowerLimitsIndices are modified
  /// to compute the distance from lower limits appropriately.
  statespace::CartesianProduct::State* mLowerLimitsState;

  /// (Modified) State corresponding to the upper position limits.
  /// The positions at the indices in mUnboundedUpperLimitsIndices are modified
  /// to compute the distance from upper limits appropriately.
  statespace::CartesianProduct::State* mUpperLimitsState;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
