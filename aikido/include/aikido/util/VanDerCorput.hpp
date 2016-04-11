#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

#include <boost/iterator/iterator_facade.hpp>
#include <cassert>
#include <limits>
#include <tuple>

using std::pair;

namespace aikido
{
namespace util
{
/// An access-based generator for Van Der Corput sequences
class VanDerCorput
{
public:
  class const_iterator;

  /// Construct a VanDerCorput sequence with specific properties
  ///
  /// \param span the endpoint of the sequence
  /// \param includeStartpoint return 0.0 in sequence
  /// \param includeEndpoint return span in sequence
  /// \param minResolution the maximum distance between any
  ///        two points in the sequence before completion
  VanDerCorput(double span = 1, bool includeStartpoint = false,
               bool includeEndpoint = false, double minResolution = 0.0);

  /// Get VanDerCorput::const_iterator at first element
  const_iterator begin() const;

  /// Get VanDerCorput::const_iterator at element
  /// that satisfies minResolution
  const_iterator end() const;

  /// Get nth Van Der Corput sequence value
  pair<double, double> operator[](int n) const;

private:
  constexpr static int BASE{2};
  constexpr static int MAX = std::numeric_limits<int>::max();

  pair<double, double> computeVanDerCorput(int n) const;

  const double mSpan;
  const bool mIncludeStartpoint;
  const bool mIncludeEndpoint;
  double mMinResolution;
};

class VanDerCorput::const_iterator
    : public boost::iterator_facade<VanDerCorput::const_iterator, const double,
                                    boost::forward_traversal_tag, const double>
{
public:
  /// dereference implementation for boost::iterator_facade
  double dereference() const;
  /// increment implementation for boost::iterator_facade
  void increment();
  /// equal implementation for boost::iterator_facade
  bool equal(const VanDerCorput::const_iterator &other) const;

private:
  friend VanDerCorput;

  /// Private constructor
  /// should always be constructed from VanDerCorput::begin()
  explicit const_iterator(const VanDerCorput *seq);

  const VanDerCorput *mSeq;
  int mN;
  bool mFinalIter;
  pair<double, double> mCurr;
};

}  // util
}  // aikido
#endif
