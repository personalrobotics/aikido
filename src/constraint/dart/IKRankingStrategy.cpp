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
    const std::vector<statespace::StateSpace::State*> ikSolutions)
  : mMetaSkeletonStateSpace(std::move(metaSkeletonStateSpace))
  , mMetaSkeleton(std::move(metaSkeleton))
{
  if (!mMetaSkeletonStateSpace)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  if (!mMetaSkeleton)
    throw std::invalid_argument("MetaSkeleton is nullptr.");

  if (ikSolutions.empty())
    throw std::invalid_argument("Vector of IK Solutions is empty.");

  mIKSolutions.resize(ikSolutions.size());
  mIKSolutions.shrink_to_fit();

  for (std::size_t i = 0; i < ikSolutions.size(); ++i)
  {
    auto state = ikSolutions[i];
    mIKSolutions[i]
        = std::pair<statespace::StateSpace::State*, double>(state, 0);
  }
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
  for (std::size_t i = 0; i < mIKSolutions.size(); ++i)
    mIKSolutions[i].second = evaluateIKSolution(mIKSolutions[i].first);

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

} // namespace dart
} // namespace constraint
} // namespace aikido
