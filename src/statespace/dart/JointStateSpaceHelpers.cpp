#include "aikido/statespace/dart/JointStateSpaceHelpers.hpp"

namespace aikido {
namespace statespace {
namespace dart {

//==============================================================================
std::unique_ptr<JointStateSpace> createJointStateSpace(
    const ::dart::dynamics::Joint* joint)
{
  auto space = common::DynamicCastFactory<
      detail::createJointStateSpaceFor_impl,
      common::DynamicCastFactory_raw_ptr,
      const ::dart::dynamics::Joint,
      detail::ConstSupportedJoints>::create(joint);

  if (!space)
  {
    std::stringstream msg;
    msg << "Joint '" << joint->getName() << "' has unsupported type '"
        << joint->getType() << "'.";
    throw std::runtime_error(msg.str());
  }

  return space;
}

} // namespace dart
} // namespace statespace
} // namespace aikido
