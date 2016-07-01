#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

#include <boost/iterator/iterator_facade.hpp>
#include <cassert>
#include <limits>
#include <tuple>

namespace aikido {
namespace util {

/// Generator for the Van der Corput sequence, a low-discripancy sequence
/// defined over a real interval. This sequence can be thought of as performing
/// successive level-order traversals of a binary search tree over the
/// interval
class VanDerCorput
{
public:
  // Implemented in detail/VanDerCorput-impl.hpp
  class const_iterator;

  /// Construts the Van der Corput sequence over an interval of length \c span
  /// that terminates when the maximum gap between samples is less than
  /// \c min_resolution. The sequence may be inclusive or exclusive of its end
  /// points; i.e. is an open or closed interval.
  ///
  /// \param span length of the interval
  /// \param include_endpoints whether or not to include the endpints
  /// \param min_resolution resolution at which to terminate
  VanDerCorput(
    double span = 1, bool include_endpoints = false,
    double min_resolution = 0.0);

  /// Returns an iterator to the first element of the sequence.
  ///
  /// \return iterator to the first element of the sequence
  const_iterator begin();

  /// Returns an iterator to the element following the last element of the
  /// sequence.
  /// 
  /// \return iterator followin the last element of the sequence
  const_iterator end();

  /// Returns the \c n-th element of the sequence (first element in the pair)
  /// and the current resolution (second element in the pair). Resolution is
  /// defined to be the maximum gap between samples thus far.
  ///
  /// \return pair of sample and maximum gap in samples
  std::pair<double, double> operator[](int n);

private:
  constexpr static int BASE { 2 };
  constexpr static int MAX { std::numeric_limits<int>::max() };

  std::pair<double, double> compute_vandercorput(int n) const;

  const double span;
  const bool include_endpoints;
  double min_resolution;
};

} // namespace util
} // namespace aikido

#include "detail/VanDerCorput-impl.hpp"

#endif
