#ifndef AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_

#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace distance {

/// ConfigurationRanker is a base class for ranking configurations.
/// The rule for evaluating the costs of configurations to rank them
/// is specified by the concrete classes.
class ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  ConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton);

  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaSkeleton Metaskeleton of the robot.
  /// \param[in] weights Weights over the joints to compute distance.
  ConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::vector<double> weights);

  /// Destructor
  virtual ~ConfigurationRanker() = default;

  /// Ranks the vector of configurations in increasing order of costs.
  /// \param[in, out] configurations Vector of configurations to rank.
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
