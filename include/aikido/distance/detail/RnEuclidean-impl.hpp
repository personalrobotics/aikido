#include "aikido/distance/RnEuclidean.hpp"

namespace aikido {
namespace distance {

//==============================================================================
extern template class REuclidean<0>;

extern template class REuclidean<1>;

extern template class REuclidean<2>;

extern template class REuclidean<3>;

extern template class REuclidean<6>;

extern template class REuclidean<Eigen::Dynamic>;

//==============================================================================
template <int N>
REuclidean<N>::REuclidean(std::shared_ptr<const statespace::R<N>> _space)
  : mStateSpace(std::move(_space))
{
  if (mStateSpace == nullptr)
  {
    throw std::invalid_argument("Rn is nullptr.");
  }
}

//==============================================================================
template <int N>
statespace::ConstStateSpacePtr REuclidean<N>::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
template <int N>
double REuclidean<N>::distance(
    const statespace::StateSpace::State* _state1,
    const statespace::StateSpace::State* _state2) const
{
  auto v1 = mStateSpace->getValue(
      static_cast<const typename statespace::R<N>::State*>(_state1));
  auto v2 = mStateSpace->getValue(
      static_cast<const typename statespace::R<N>::State*>(_state2));
  return (v2 - v1).norm();
}

} // namespace distance
} // namespace aikido
