#include <aikido/planner/vectorfield/VectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorField::VectorField(aikido::statespace::StateSpacePtr stateSpace)
  : mStateSpace(stateSpace)
{
  // Do nothing
}

//==============================================================================
aikido::statespace::StateSpacePtr VectorField::getStateSpace()
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
