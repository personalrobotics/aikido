#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

#include <boost/iterator/iterator_facade.hpp>
#include <cassert>
#include <limits>
#include <tuple>

namespace aikido
{
namespace util
{
using std::pair;

class VanDerCorput
{
public:
  class const_iterator;

  VanDerCorput(const double span = 1, const bool include_endpoints = false,
               const double min_resolution = 0.0);

  const_iterator begin();
  const_iterator end();

  pair<double, double> operator[](int n);

private:
  constexpr static int BASE{2};
  constexpr static int MAX = std::numeric_limits<int>::max();

  pair<double, double> compute_vandercorput(int n) const;

  const double span;
  const bool include_endpoints;
  double min_resolution;
};

class VanDerCorput::const_iterator
    : public boost::iterator_facade<VanDerCorput::const_iterator, const double,
                                    boost::forward_traversal_tag, const double>
{

public:
  double dereference() const { return curr.first; }

  void increment();
  bool equal(const VanDerCorput::const_iterator &other) const;

private:
  friend VanDerCorput;

  const_iterator(VanDerCorput *seq)
      : seq(seq)
      , n(-1)
      , final_iter(false)
  {
    assert(seq);
    increment();
  }

  VanDerCorput *seq;
  int n;
  bool final_iter;
  pair<double, double> curr;
};

}  // util
}  // aikido
#endif
