#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

#include <tuple>

namespace aikido {
namespace util {

class VanDerCorput
{
public:
  VanDerCorput(const double span=1, const bool include_endpoints=false);

  double operator()();
  double current_resolution() const { return resolution; };

private:
  typedef std::pair<double, double> DTuple;
  constexpr static int BASE{2};

  DTuple compute_vandercorput(int n) const;

  const double span;
  const bool include_endpoints;
  int n;
  double resolution;
};

}  // util
}  // aikido
#endif
