#include <aikido/planner/vectorfield/ConfigurationSpaceVectorField.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

ConfigurationSpaceVectorField::ConfigurationSpaceVectorField(
    aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
    dart::dynamics::BodyNodePtr _bodyNode)
  : mStateSpace(_stateSpace)
  , mMetaSkeleton(_stateSpace->getMetaSkeleton())
  , mBodyNode(_bodyNode)
{
  // Do nothing
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
