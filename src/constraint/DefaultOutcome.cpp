#include <aikido/constraint/DefaultOutcome.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
bool DefaultOutcome::isSatisfied() const
{
  return mSatisfiedFlag;
}

//==============================================================================
std::string DefaultOutcome::toString() const
{
  if (mSatisfiedFlag)
    return "true";

  return "false";
}

//==============================================================================
void DefaultOutcome::setSatisfiedFlag(bool satisfiedFlag)
{
  mSatisfiedFlag = satisfiedFlag;
}

} // namespace constraint
} // namespace aikido
