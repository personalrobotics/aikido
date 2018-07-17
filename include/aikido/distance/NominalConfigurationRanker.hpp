#ifndef AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

/// Ranks configurations by their distance from a nominal configuration.
/// Configurations closer to the nominal configuration are ranked higher.
class NominalConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] nominalConfiguration Nominal configuration. The current
  /// configuration of \c metaSkeleton is considered if set to \c nullptr.
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const statespace::CartesianProduct::State* nominalConfiguration
      = nullptr);

protected:
  /// Returns cost as distance from the Nominal Configuration.
  double evaluateConfiguration(
      const statespace::dart::MetaSkeletonStateSpace::State* solution)
      const override;

  /// Nominal configuration used when evaluating a given configuration.
  const statespace::dart::MetaSkeletonStateSpace::State* mNominalConfiguration;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
