#include <aikido/distance/RnEuclidean.hpp>

namespace aikido {
namespace distance {

//=============================================================================
template <int N>
RnEuclidean<N>::RnEuclidean(std::shared_ptr<statespace::Rn<N>> _space)
    : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("Rn is nullptr.");
  }
}

//=============================================================================
template <int N>
statespace::StateSpacePtr RnEuclidean<N>::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
template <int N>
double RnEuclidean<N>::distance(
    const statespace::StateSpace::State* _state1,
    const statespace::StateSpace::State* _state2) const
{
  auto v1 = mStateSpace->getValue(
      static_cast<const typename statespace::Rn<N>::State*>(_state1));
  auto v2 = mStateSpace->getValue(
      static_cast<const typename statespace::Rn<N>::State*>(_state2));
  return (v2 - v1).norm();
}

} // namespace distance
} // namespace aikido
