#ifndef AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_

#include "aikido/distance/ConfigurationRanker.hpp"
#include "aikido/distance/CartesianProductWeighted.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/statespace/CartesianProduct.hpp"

namespace aikido {
namespace distance {

class NominalConfigurationRanker : public ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] ikSolutions List of IK solutions to be ranked.
  NominalConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const std::vector<statespace::StateSpace::State*> ikSolutions);

protected:
  double evaluateIKSolution(
      statespace::StateSpace::State* solution) const override;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_NOMINALCONFIGURATIONRANKER_HPP_
