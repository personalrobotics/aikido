#ifndef AIKIDO_STATESPACE_SE3STATESPACE_H
#define AIKIDO_STATESPACE_SE3STATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"
#include <dart/math/MathTypes.h>

namespace aikido {
namespace statespace {

class SE3StateSpace : public StateSpace
{
public:
  class SE3State : public UtilState
  {
  public:
    // q = (w, v)
    SE3State(Eigen::Vector6d _q): UtilState(_q){};

    // default is identity
    SE3State(): UtilState(Eigen::Vector6d::Zero()){};

    Eigen::Isometry3d getIsometry() const;
  };

  class SE3Jacobian : public UtilJacobian
  {
  public:
    SE3Jacobian(Eigen::Matrix<double, Eigen::Dynamic, 6> _jac)
    : UtilJacobian(_jac){};
  };

  SE3StateSpace() = default;

  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

  int getRepresentationDimension() const override;
};

using SE3State = SE3StateSpace::SE3State;
using SE3Jacobian = SE3StateSpace::SE3Jacobian;

} // namespace statespace
} // namespace aikido

#endif
