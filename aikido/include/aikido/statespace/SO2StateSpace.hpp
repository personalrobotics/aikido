#ifndef AIKIDO_STATESPACE_SO2STATESPACE_H
#define AIKIDO_STATESPACE_SO2STATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class SO2StateSpace : public StateSpace {

public:
  SO2StateSpace(){};
  
  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

  int getRepresentationDimension() const override;

  class SO2State: public UtilState
  {
  public:
    // q: angle 
    SO2State(Eigen::VectorXd _q): UtilState(_q){};
    SO2State(double angle): UtilState(Eigen::VectorXd::Zero(1))
    {
      mQ(0) = angle;
    }
    SO2State(): SO2State(0){};

    Eigen::Isometry2d getIsometry() const;
  };

  class SO2Jacobian: public UtilJacobian
  {
  public:
    SO2Jacobian(Eigen::Matrix<double, Eigen::Dynamic, 1> _jac)
    : UtilJacobian(_jac){};
  };

};

using SO2State = SO2StateSpace::SO2State;
using SO2Jacobian = SO2StateSpace::SO2Jacobian;


}
}

#endif
