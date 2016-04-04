#ifndef AIKIDO_STATESPACE_SE2STATESPACE_H
#define AIKIDO_STATESPACE_SE2STATESPACE_H
#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class SE2StateSpace : public StateSpace
{
public:
  class SE2State : public UtilState
  {
  public:
    // default is identity
    SE2State()
      : UtilState(Eigen::Vector3d::Zero())
    {
    }

    // q: (w, x, y)
    SE2State(Eigen::Vector3d _q)
      : UtilState(_q)
    {
    }

    Eigen::Isometry2d getIsometry() const;
  };

  class SE2Jacobian : public UtilJacobian
  {
  public:
    SE2Jacobian(Eigen::Matrix<double, Eigen::Dynamic, 3> _jac)
      : UtilJacobian(_jac)
    {
    }
  };

  SE2StateSpace() = default;

  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

  int getRepresentationDimension() const override;
};

using SE2State = SE2StateSpace::SE2State;
using SE2Jacobian = SE2StateSpace::SE2Jacobian;

} // namespace statespace
} // namespace aikido

#endif
