#ifndef AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_
#define AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_

#include "aikido/constraint/dart/IKRankingStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

class NominalConfigurationStrategy : public IKRankingStrategy
{
public:
  /// Constructor
  ///
  /// \param[in] metaSkeletonStateSpace Statespace of the skeleton.
  /// \param[in] metaskeleton Metaskeleton of the robot.
  /// \param[in] numIKSolution Number of IK solutions to rank.
  NominalConfigurationStrategy(
      statespace::dart::ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
      ::dart::dynamics::ConstMetaSkeletonPtr metaSkeleton,
      std::size_t numIKSolutions = 1);

protected:
  double evaluateIKSolution(
      statespace::StateSpace::State* solution) const override;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_NOMINALCONFIGURATIONSTRATEGY_HPP_
