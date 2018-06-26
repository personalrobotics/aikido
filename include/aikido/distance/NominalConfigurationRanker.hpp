#ifndef AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"

namespace aikido {
namespace distance {

class NominalConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] nominalConfiguration Nominal Configuration. The current
  /// configuration is considered if set to nullptr.
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const statespace::CartesianProduct::State* nominalConfiguration);

protected:
  /// Returns cost as distance from the Nominal Configuration.
  double evaluateConfiguration(
      statespace::CartesianProduct::State* solution) const override;

  /// Nominal configuration used when evaluating a given configuration.
  const statespace::CartesianProduct::State* mNominalConfiguration;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
