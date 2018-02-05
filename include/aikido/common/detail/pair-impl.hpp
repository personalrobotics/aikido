#ifndef AIKIDO_COMMON_DETAIL_PAIR_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_PAIR_IMPL_HPP_

#include <aikido/common/pair.hpp>

#include <boost/functional/hash.hpp>

namespace aikido {
namespace common {

//==============================================================================
template <typename T1, typename T2>
std::size_t PairHash::operator()(const std::pair<T1, T2>& pair) const
{
  std::size_t hash = 0;
  boost::hash_combine(hash, pair.first);
  boost::hash_combine(hash, pair.second);

  return hash;
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_PAIR_IMPL_HPP_
