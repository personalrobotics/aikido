#ifndef AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_

#include <cassert>
#include <functional>

#include "aikido/common/algorithm.hpp"

namespace aikido {
namespace common {

//==============================================================================
template <class T, class Compare>
const T& clamp(const T& v, const T& lo, const T& hi, Compare comp)
{
  assert(!comp(hi, lo));
  return comp(v, lo) ? lo : comp(hi, v) ? hi : v;
}

//==============================================================================
template <class T>
const T& clamp(const T& v, const T& lo, const T& hi)
{
  return clamp(v, lo, hi, std::less<T>());
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_
