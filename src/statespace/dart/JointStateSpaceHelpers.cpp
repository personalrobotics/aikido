#include <aikido/statespace/dart/JointStateSpaceHelpers.hpp>

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
std::unique_ptr<JointStateSpace> createJointStateSpace(
    ::dart::dynamics::Joint* _joint)
{
  auto space
      = common::DynamicCastFactory<detail::createJointStateSpaceFor_impl,
                                 common::DynamicCastFactory_raw_ptr,
                                 ::dart::dynamics::Joint,
                                 detail::SupportedJoints>::create(_joint);

  if (!space)
  {
    std::stringstream msg;
    msg << "Joint '" << _joint->getName() << "' has unsupported type '"
        << _joint->getType() << "'.";
    throw std::runtime_error(msg.str());
  }

  return space;
}

} // namespace dart
} // namespace statespace
} // namespace aikido
