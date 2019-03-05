#ifndef AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

AIKIDO_DECLARE_POINTERS(NominalConfigurationRanker)

/// Ranks configurations by their distance from a nominal configuration.
/// Configurations closer to the nominal configuration are ranked higher.
class NominalConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] nominalConfiguration Nominal configuration.
  /// \param[in] weights Weights over the joints to compute distance.
  /// Defaults to unit vector.
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const statespace::CartesianProduct::State* nominalConfiguration,
      std::vector<double> weights = std::vector<double>());

  /// Constructor. The current configuration of \c metaSkeleton is
  /// considered as the nominal configuration.
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] weights Weights over the joints to compute distance.
  /// Defaults to unit vector.
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::vector<double> weights = std::vector<double>());

protected:
  /// Returns cost as distance from the Nominal Configuration.
  double evaluateConfiguration(
      const statespace::dart::MetaSkeletonStateSpace::State* solution)
      const override;

  /// Nominal configuration used when evaluating a given configuration.
  const statespace::dart::MetaSkeletonStateSpace::ScopedState mNominalConfiguration;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
