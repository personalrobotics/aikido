#ifndef AIKIDO_UTIL_VANDERCORPUT_HPP_
#define AIKIDO_UTIL_VANDERCORPUT_HPP_

#include <cassert>
#include <limits>
#include <tuple>
#include <boost/iterator/iterator_facade.hpp>

namespace aikido {
namespace util {

/// Generator for the Van der Corput sequence, a low-discripancy sequence
/// defined over a real interval. This sequence can be thought of as performing
/// successive level-order traversals of a binary search tree over the
/// interval
class VanDerCorput
{
public:
  class const_iterator;

  /// Construts the Van der Corput sequence over an interval of length \c span
  /// that terminates when the maximum gap between samples is less than
  /// \c minResolution. The sequence may be inclusive or exclusive of its end
  /// point; i.e. is an open or closed interval.
  ///
  /// \param span length of the interval
  /// \param includeStartpoint whether or not to include the startpoint
  /// \param includeEndpoint whether or not to include the endpoint
  /// \param minResolution resolution at which to terminate
  VanDerCorput(
      double span = 1.0,
      bool includeStartpoint = false,
      bool includeEndpoint = false,
      double minResolution = 0.0);

  /// Returns an iterator to the first element of the sequence.
  ///
  /// \return iterator to the first element of the sequence
  const_iterator begin() const;

  /// Returns an iterator to the element following the last element of the
  /// sequence.
  ///
  /// \return iterator followin the last element of the sequence
  const_iterator end() const;

  /// Returns the \c n-th element of the sequence (first element in the pair)
  /// and the current resolution (second element in the pair). Resolution is
  /// defined to be the maximum gap between samples thus far.
  ///
  /// \return pair of sample and maximum gap in samples
  std::pair<double, double> operator[](int n) const;

private:
  constexpr static int BASE{2};
  constexpr static int MAX{std::numeric_limits<int>::max()};

  std::pair<double, double> computeVanDerCorput(int n) const;

  const double mSpan;
  const bool mIncludeStartpoint;
  const bool mIncludeEndpoint;
  double mMinResolution;
};

class VanDerCorput::const_iterator
    : public boost::iterator_facade<VanDerCorput::const_iterator,
                                    const double,
                                    boost::forward_traversal_tag,
                                    const double>
{
public:
  /// Dereference implementation for boost::iterator_facade
  double dereference() const;

  /// Increment implementation for boost::iterator_facade
  void increment();

  /// equal implementation for boost::iterator_facade
  bool equal(const VanDerCorput::const_iterator& other) const;

private:
  friend class VanDerCorput;

  /// Private constructor that should always be constructed from
  /// VanDerCorput::begin()
  const_iterator(const VanDerCorput* seq);

  const VanDerCorput* mSeq;
  int mN;
  bool mFinalIter;
  std::pair<double, double> mCurr;
};

} // namespace util
} // namespace aikido

#endif
