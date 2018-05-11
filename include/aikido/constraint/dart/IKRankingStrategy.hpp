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
  /// \param[in] maxSolutionsToRank Maximum number of IK Solutions to rank.
  IKRankingStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaskeleton);

  /// Destructor
  // TODO (avk): What is default mean?
  virtual ~IKRankingStrategy() = default;

  /// Returns the statespace.
  statespace::ConstStateSpacePtr getStateSpace() const;

  /// Returns the skeleton.
  ::dart::dynamics::ConstMetaSkeletonPtr getMetaSkeleton() const;

  /// Adds IK solution to the ordered set.
  virtual void addIKSolution(statespace::StateSpace::State* solution) = 0;

  /// Returns (pops) the top ranked solution.
  virtual statespace::StateSpace::State* getIKSolution() = 0;

private:
  /// Statespace of the skeleton.
  statespace::dart::MetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;

  /// Metaskeleton of the robot.
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_
