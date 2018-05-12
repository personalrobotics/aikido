#ifndef AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_
#define AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_

#include <dart/common/Memory.hpp>
#include <dart/dynamics/dynamics.hpp>
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/dart/IKRankingStrategy.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint {
namespace dart {

class NominalConfigurationStrategy : IKRankingStrategy
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  NominalConfigurationStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaskeleton);

  /// Destructor
  // TODO (avk): What is default mean?
  ~NominalConfigurationStrategy();

  /// Adds IK solution to the ordered set.
  void addIKSolution(statespace::StateSpace::State* solution) override final;

  /// Returns (pops) the top ranked solution.
  statespace::StateSpace::State* getIKSolution() override final;

private:
  // TODO
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_
