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
class VanDerCorput
{
public:
  class const_iterator;

  VanDerCorput(double span = 1, bool includeStartpoint = false,
               bool includeEndpoint = false, double minResolution = 0.0);

  const_iterator begin() const;
  const_iterator end() const;

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
  double dereference() const;
  void increment();
  bool equal(const VanDerCorput::const_iterator &other) const;

private:
  friend VanDerCorput;

  explicit const_iterator(const VanDerCorput *seq);

  const VanDerCorput *mSeq;
  int mN;
  bool mFinalIter;
  pair<double, double> mCurr;
};

}  // util
}  // aikido
#endif
