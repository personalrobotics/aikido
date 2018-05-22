#ifndef AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
#define AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_

#include <dart/dynamics/dynamics.hpp>
#include "aikido/statespace/ScopedState.hpp" // TODO: Remove this.
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace distance {

class ConfigurationRanker
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] ikSolutions List of IK solutions to be ranked.
  ConfigurationRanker(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const std::vector<statespace::StateSpace::State*> ikSolutions);

  /// Destructor
  virtual ~ConfigurationRanker() = default;

  /// Returns the statespace.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the vector of ranked IK solutions.
  std::vector<std::pair<statespace::StateSpace::State*, double>>&
  getRankedIKSolutions();

protected:
  /// Returns the score of the IK Solution
  virtual double evaluateIKSolution(
      statespace::StateSpace::State* solution) const = 0;

  /// Statespace of the skeleton.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Metaskeleton of the robot.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Vector to hold IK solutions and corresponding score.
  std::vector<std::pair<statespace::StateSpace::State*, double>> mIKSolutions;
};

} // namespace distance
} // namespace aikido

#endif // AIKIDO_DISTANCE_CONFIGURATIONRANKER_HPP_
