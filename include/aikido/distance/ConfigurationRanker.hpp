#ifndef AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_

#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace distance {

class ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  ConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton);

  /// Destructor
  virtual ~ConfigurationRanker() = default;

  /// Ranks/Sorts the vector of configurations in increasing order of costs.
  /// \param[in] configurations Vector of configurations to rank.
  /// \param[out] configurations Ranked vector of configurations.
  void rankConfigurations(
      std::vector<statespace::dart::MetaSkeletonStateSpace::ScopedState>&
          configurations);

protected:
  /// Returns the cost of the configuration.
  /// \param[in] solution Configuration to evaluate.
  virtual double evaluateConfiguration(
      const statespace::dart::MetaSkeletonStateSpace::State* solution)
      const = 0;

  /// Statespace of the skeleton.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Metaskeleton of the robot.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Distance Metric in this space
  distance::DistanceMetricPtr mDistanceMetric;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
