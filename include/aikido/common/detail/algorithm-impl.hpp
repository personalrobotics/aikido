#ifndef AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_

#include "aikido/common/algorithm.hpp"

#include <cassert>
#include <cmath>
#include <functional>

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

//==============================================================================
template <typename T>
std::vector<T> linspace(T start, T end, std::size_t n, bool endpoint)
{
  std::vector<T> vals(n);

  if (n == 0u)
    return vals;

  if (n == 1u)
  {
    vals[0] = start;
    return vals;
  }

  std::size_t m = n - 1;
  if (!endpoint)
    m = n;

  const T h = (end - start) / static_cast<T>(m);

  T val = start;
  for (std::size_t i = 0u; i < m; ++i)
  {
    vals[i] = val;
    val += h;
  }

  if (endpoint)
    vals[m] = end;

  return vals;
}

//==============================================================================
template <typename T>
std::vector<T> arange(T start, T stop, T step)
{
  std::vector<T> values;
  for (T value = start; value < stop; value += step)
    values.push_back(value);

  return values;
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_ALGORITHM_IMPL_HPP_
