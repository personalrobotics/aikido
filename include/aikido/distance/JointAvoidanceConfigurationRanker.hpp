#ifndef AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

/// Ranks configurations by their distance from joint limits. Configurations
/// further away from the limits are ranked higher. Only finite joint limits
/// are considered for distance computation.
class JointAvoidanceConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] weights Weights over joints to compute distance.
  /// Defaults to unit vector.
  JointAvoidanceConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::vector<double> weights = std::vector<double>());

protected:
  /// Set limits appropriately to account for infinite limits.
  void setupJointLimits();

  /// Returns cost as negative of distance from position limits.
  double evaluateConfiguration(
      const statespace::dart::MetaSkeletonStateSpace::State* solution)
      const override;

  /// Vector of indices corresponding to unbounded lower position limits.
  std::vector<std::size_t> mUnboundedLowerLimitsIndices;

  /// Vector of indices corresponding to unbounded upper position limits.
  std::vector<std::size_t> mUnboundedUpperLimitsIndices;

  /// (Modified) State corresponding to the lower position limits.
  /// The positions at the indices in \c mUnboundedLowerLimitsIndices are
  /// modified to match those in the input state to \c evaluateConfiguration()
  /// to compute the distance from finite lower joint limits only.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mLowerLimitsState;

  /// (Modified) State corresponding to the upper position limits.
  /// The positions at the indices in \c mUnboundedUpperLimitsIndices are
  /// modified to match those in the input state to \c evaluateConfiguration()
  /// to compute the distance from finite upper joint limits only.
  statespace::dart::MetaSkeletonStateSpace::ScopedState mUpperLimitsState;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_JOINTAVOIDANCECONFIGURATIONRANKER_HPP_
