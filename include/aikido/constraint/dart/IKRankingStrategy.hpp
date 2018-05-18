#ifndef AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_
#define AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_

#include <dart/dynamics/dynamics.hpp>
#include "aikido/statespace/ScopedState.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint {
namespace dart {

class IKRankingStrategy
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] ikSolutions List of IK solutions to be ranked.
  IKRankingStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      const std::vector<statespace::StateSpace::State*> ikSolutions);

  /// Destructor
  virtual ~IKRankingStrategy() = default;

  /// Returns the statespace.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the vector of ranked IK solutions.
  statespace::StateSpace::State* rankedIKSolution(std::size_t index) const;

protected:
  /// Returns the score of the IK Solution
  virtual double evaluateIKSolution(
      statespace::StateSpace::State* solution) const;

  /// Statespace of the skeleton.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Metaskeleton of the robot.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Vector to hold IK solutions and corresponding score.
  std::vector<std::pair<statespace::StateSpace::State*, double>> mIKSolutions;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_