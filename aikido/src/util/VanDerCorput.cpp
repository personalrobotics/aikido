#include <aikido/util/VanDerCorput.hpp>
#include <iostream>

using namespace aikido::util;

VanDerCorput::VanDerCorput(const int span, const bool include_endpoints)
    : span(span)
    , include_endpoints(include_endpoints)
    , n(0)
{
}

double VanDerCorput::operator()()
{
  double ret = 0.0;

  if (include_endpoints) {
    if (n == 0) {
      ret = 0.0;
    } else if (n == 1) {
      ret = 1.0;
    } else {
      ret = compute_vandercorput(n - 1);
    }
  } else {
    ret = compute_vandercorput(n + 1);
  }

  ++n;
  return span * ret;
}

double VanDerCorput::compute_vandercorput(int n)
{
  double denom = 1;
  double ret = 0.0;

  while (n) {
    denom *= BASE;
    ret += (n % BASE) / denom;
    n /= BASE;
  }

  return ret;
}
