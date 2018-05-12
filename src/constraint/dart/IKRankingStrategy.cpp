#include "aikido/constraint/dart/IKRankingStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
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
std::vector<std::pair<statespace::StateSpace::State*, double>>
IKRankingStrategy::getRankedIKSolutions()
{
  std::vector<std::pair<statespace::StateSpace::State*, double>>
      rankedSolutions;
  return rankedSolutions;
}

//==============================================================================
void IKRankingStrategy::addIKSolution(statespace::StateSpace::State* solution)
{
  double score = evaluateIKSolution(solution);
  mIKSolutions.emplace_back(
      std::pair<statespace::StateSpace::State*, double>(solution, score));
}

} // namespace dart
} // namespace constraint
} // namespace aikido
