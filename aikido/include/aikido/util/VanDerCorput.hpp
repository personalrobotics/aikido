#ifndef AIKIDO_VANDERCORPUT_HPP_
#define AIKIDO_VANDERCORPUT_HPP_

namespace aikido {
namespace util {

class VanDerCorput
{
public:
  VanDerCorput(const int span=1, const bool include_endpoints=false);

  double operator()();

private:
  constexpr static int BASE{2};

  double compute_vandercorput(int n);

  const int span;
  const bool include_endpoints;
  int n;
};

}  // util
}  // aikido
#endif
