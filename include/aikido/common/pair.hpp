#ifndef AIKIDO_COMMON_PAIR_HPP_
#define AIKIDO_COMMON_PAIR_HPP_

namespace aikido {
namespace common {

/// Implements a hash function for pairs of std::hash-able types.
struct PairHash
{

  /// Compute a hash for a pair of hashable types.
  /// \param[in] pair Pair to hash
  template <typename T1, typename T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const;
};

} // namespace common
} // namespace aikido

#include "aikido/common/detail/pair-impl.hpp"

#endif // AIKIDO_COMMON_PAIR_HPP_
