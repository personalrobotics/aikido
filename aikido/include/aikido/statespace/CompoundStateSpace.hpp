#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

class CompoundStateSpace : public StateSpace {

public:
  CompoundStateSpace(std::vector<StateSpacePtr> _subspaces);
   
  void compose(const State& _state1, const State& _state2,
               State& _out) const override;

  int getRepresentationDimension() const override;

  class CompoundState: public UtilState
  {
  public:
    CompoundState(Eigen::VectorXd _q): UtilState(_q){};

    CompoundState(int _dim): UtilState(Eigen::VectorXd(_dim)){};
  };


  using CompoundJacobian = UtilJacobian;

private:

  std::vector<StateSpacePtr> mSubspaces;
  int mRepresentationDimension;

};

using CompoundState = CompoundStateSpace::CompoundState;


}
}

#endif
