#include "aikido/constraint/dart/IKRankingStrategy.hpp"

namespace aikido {
namespace constraint {
namespace dart {

using statespace::dart::ConstMetaSkeletonStateSpacePtr;
using ::dart::dynamics::ConstMetaSkeletonPtr;

//==============================================================================
IKRankingStrategy::IKRankingStrategy(
    ConstMetaSkeletonStateSpacePtr metaSkeletonStateSpace,
    ConstMetaSkeletonPtr metaSkeleton,
    std::size_t numIKSolutions)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
  , mIndex(0.0)
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");

  mIKSolutions.resize(numIKSolutions);
}

//==============================================================================
statespace::ConstStateSpacePtr IKRankingStrategy::getStateSpace() const
{
  return mMetaSkeletonStateSpace;
}

//==============================================================================
std::vector<std::pair<statespace::StateSpace::State*, double>>&
IKRankingStrategy::getRankedIKSolutions()
{
  struct sortingFunction
  {
    bool operator()(
        const std::pair<statespace::StateSpace::State*, double>& left,
        const std::pair<statespace::StateSpace::State*, double>& right)
    {
      return left.second < right.second;
    }
  };

  std::sort(mIKSolutions.begin(), mIKSolutions.end(), sortingFunction());
  return mIKSolutions;
}

//==============================================================================
void IKRankingStrategy::addIKSolution(statespace::StateSpace::State* solution)
{
  if (mIndex == mIKSolutions.size())
    throw std::invalid_argument("Index exceeds maximum number of IK solutions");

  double score = evaluateIKSolution(solution);
  mIKSolutions.emplace_back(
      std::pair<statespace::StateSpace::State*, double>(solution, score));
  mIndex++;
}

} // namespace dart
} // namespace constraint
} // namespace aikido
