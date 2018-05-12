#include "aikido/constraint/dart/IKRankingStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using statespace::StateSpace::State;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
IKRankingStrategy::IKRankingStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");
}

//==============================================================================
statespace::ConstStateSpacePtr IKRankingStrategy::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
std::vector<std::pair<State*, double>> IKRankingStrategy::getRankedIKSolutions()
{
  return nullptr;
}

//==============================================================================
void IKRankingStrategy::addIKSolution(State* solution)
{
  auto evaluatedSolution = evaluateIKSolution(solution);
  mIKSolutions.emplace_back(evaluatedSolution);
}

//==============================================================================
std::pair<State*, double> IKRankingStrategy::evaluateIKSolution(State* solution)
{
  return std::pair<State*, double>(solution, 0.0);
}

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_IKRANKINGSTRATEGY_HPP_
