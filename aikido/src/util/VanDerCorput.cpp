#include <aikido/util/VanDerCorput.hpp>
#include <cmath>

using namespace aikido::util;

VanDerCorput::VanDerCorput(const double span, const bool include_endpoints)
    : span(span)
    , include_endpoints(include_endpoints)
    , n(0)
    , resolution(span)
{
}

double VanDerCorput::operator()()
{
  double ret = 0.0;
  DTuple val_res;

  if (include_endpoints) {
    if (n == 0) {
      ret = 0.0;
    } else if (n == 1) {
      ret = 1.0;
    } else {
      val_res = compute_vandercorput(n - 1);
      ret = val_res.first;
      resolution = val_res.second;
    }
  } else {
    val_res = compute_vandercorput(n + 1);
    ret = val_res.first;
    resolution = val_res.second;
  }

  ++n;
  return span * ret;
}

VanDerCorput::DTuple VanDerCorput::compute_vandercorput(int n) const
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
