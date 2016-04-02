#include <aikido/statespace/CompoundStateSpace.hpp>
#include <iostream>

namespace aikido {
namespace statespace {

CompoundStateSpace::CompoundStateSpace(std::vector<StateSpacePtr> _subspaces)
: mSubspaces(_subspaces)
{
  mRepresentationDimension = 0;
  for (auto supspace: mSubspaces)
  {
    mRepresentationDimension += supspace->getRepresentationDimension();
  }
}

void CompoundStateSpace::compose(const State& _state1, const State& _state2,
                                 State& _out) const
{

  CompoundState& out = static_cast<CompoundState&>(_out);

  const UtilState& state1 = static_cast<const UtilState&>(_state1);
  const UtilState& state2 = static_cast<const UtilState&>(_state2);
  
  int index = 0;
  for(int i = 0; i < mSubspaces.size(); ++i)
  {
    int dim = mSubspaces[i]->getRepresentationDimension();
    UtilState s1(dim), s2(dim), sOut(dim);
    s1.mQ = state1.mQ.block(index, 0, dim, 1);
    s2.mQ = state2.mQ.block(index, 0, dim, 1);

    mSubspaces[i]->compose(s1, s2, sOut);

    out.mQ.block(index, 0, dim, 1) = sOut.mQ;
    index += dim;
  

  }

}


int CompoundStateSpace::getRepresentationDimension() const
{
  return mRepresentationDimension;
}


}
}
