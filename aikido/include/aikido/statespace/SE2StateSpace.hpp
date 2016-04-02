#ifndef AIKIDO_STATESPACE_SE2STATESPACE_H
#define AIKIDO_STATESPACE_SE2STATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class SE2StateSpace : public StateSpace {
  
public:
  SE2StateSpace(){};

  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

  int getRepresentationDimension() const override;

  using SE2Jacobian = UtilJacobian;

  class SE2State: public UtilState
  {
  public:
    // q: (w, x, y)
    SE2State(Eigen::Vector3d _q): UtilState(_q){};

    // default is identity
    SE2State(): UtilState(Eigen::Vector3d::Zero()){};

    Eigen::Isometry2d getIsometry() const;
  };


};

using SE2State = SE2StateSpace::SE2State;


}
}

#endif
