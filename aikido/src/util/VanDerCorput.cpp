#include <aikido/util/VanDerCorput.hpp>
#include <cmath>
#include <iostream>

using namespace aikido::util;
using std::pair;

VanDerCorput::VanDerCorput(const double span, const bool include_endpoints,
                           const double min_resolution_)
    : span(span)
    , include_endpoints(include_endpoints)
    , min_resolution(min_resolution_)
{
  if (min_resolution == 0.0) {
    min_resolution = std::numeric_limits<double>::epsilon();
  }
}

pair<double, double> VanDerCorput::operator[](int n)
{
  pair<double, double> val_res;

  if (n == VanDerCorput::MAX) {
    throw std::out_of_range("Indexed maximum integer.");
  }

  if (include_endpoints) {
    if (n == 0) {
      val_res.first = 0.0;
      val_res.second = span;
    } else if (n == 1) {
      val_res.first = 1.0;
      val_res.second = span;
    } else {
      val_res = compute_vandercorput(n - 1);
    }
  } else {
    val_res = compute_vandercorput(n + 1);
  }

  val_res.first *= span;
  return val_res;
}

pair<double, double> VanDerCorput::compute_vandercorput(int n) const
{
  // range: [1,int_max]
  double denom = 1;
  double resolution = 1;
  double ret = 0.0;

  // Treat Van Der Corput sequence like a binary tree.
  // Each node that completes a perfect tree
  // reduces the resolution by cutting the final remaining
  // segment of the last resolution size.
  // So to find the resolution...
  double power = std::ceil(std::log2(n + 1)) - 1;  // calc height of tree
  double next_power =
      std::ceil(std::log2(n + 2)) - 1;  // and height after next node is added.
  if (power == next_power) {            // If next node does not start new level
    resolution = 1. / (std::pow(2, power));  // not yet perfect tree
  } else {  // if next node does start new level
    resolution = 1. / (std::pow(2, power + 1));  // shrink resolution
  }

  while (n) {
    denom *= BASE;
    ret += (n % BASE) / denom;
    n /= BASE;
  }

  return std::make_pair(ret, resolution);
}

VanDerCorput::const_iterator VanDerCorput::begin()
{
  VanDerCorput::const_iterator itr{this};
  return itr;
}

VanDerCorput::const_iterator VanDerCorput::end()
{
  VanDerCorput::const_iterator itr{this};
  itr.n = VanDerCorput::MAX;
  return itr;
}

void VanDerCorput::const_iterator::increment()
{
  if (final_iter) {
    n = VanDerCorput::MAX;
  } else {
    ++n;
    curr = (*seq)[n];
    if (curr.second <= seq->min_resolution) {
      final_iter = true;
    }
  }
}

bool VanDerCorput::const_iterator::equal(
    const VanDerCorput::const_iterator &other) const
{
  return other.n == n && other.seq == seq;
}
