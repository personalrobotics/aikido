#ifndef AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_

#include "aikido/distance/CartesianProductWeighted.hpp"
#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/distance/defaults.hpp"
#include "aikido/statespace/CartesianProduct.hpp"
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

  /// Returns the statespace.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the vector of ranked configurations.
  void rankConfigurations(
      std::vector<statespace::CartesianProduct::State*>& configurations);

protected:
  /// Returns the score of the configuration
  virtual double evaluateConfiguration(
      statespace::StateSpace::State* solution) const = 0;

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
