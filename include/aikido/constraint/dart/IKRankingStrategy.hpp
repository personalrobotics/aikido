#ifndef AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_
#define AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_

#include <dart/common/Memory.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/constraint/Sampleable.hpp"
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
  /// \param[in] numIKSolution Number of IK solutions to rank.
  IKRankingStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::size_t numIKSolutions);

  /// Destructor
  // TODO (avk): What is default mean?
  virtual ~IKRankingStrategy() = default;

  /// Returns the statespace.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the vector of ranked IK solutions.
  std::vector<std::pair<statespace::StateSpace::State*, double>>&
  getRankedIKSolutions();

  /// Add IK solution to Ranker
  /// Evaluates, creates a pair and stores it in the vector.
  void addIKSolution(statespace::StateSpace::State* solution);

private:
  /// Returns the score of the IK Solution
  virtual double evaluateIKSolution(statespace::StateSpace::State* solution)
      = 0;

  /// Statespace of the skeleton.
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Metaskeleton of the robot.
  ::dart::dynamics::ConstMetaSkeletonPtr mMetaSkeleton;

  /// Vector to hold IK solutions and corresponding score.
  std::vector<std::pair<statespace::StateSpace::State*, double>> mIKSolutions;

  /// Helper variable to keep track of number of IK Solutions held.
  std::size_t mIndex;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_
