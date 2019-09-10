#include "aikido/planner/vectorfield/VectorField.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorField::VectorField(aikido::statespace::ConstStateSpacePtr stateSpace)
  : mStateSpace(stateSpace)
{
  // Do nothing
}

//==============================================================================
aikido::statespace::ConstStateSpacePtr VectorField::getStateSpace()
{
  return mStateSpace;
}

//==============================================================================
aikido::statespace::ConstStateSpacePtr VectorField::getStateSpace() const
{
  return mStateSpace;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
