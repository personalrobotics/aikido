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
  /// \param[in] nominalConfiguration Nominal Configuration
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const statespace::StateSpace::State* nominalConfiguration);

protected:
  /// Returns score as distance from the Nominal Configuration.
  double evaluateConfiguration(
      statespace::StateSpace::State* solution) const override;

  const statespace::StateSpace::State* mNominalConfiguration;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
