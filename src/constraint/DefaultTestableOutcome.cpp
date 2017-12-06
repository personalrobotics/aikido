#include <aikido/constraint/DefaultTestableOutcome.hpp>

namespace aikido {
namespace constraint {

//==============================================================================
bool DefaultTestableOutcome::isSatisfied() const
{
  return mSatisfiedFlag;
}

//==============================================================================
std::string DefaultTestableOutcome::toString() const
{
  if (mSatisfiedFlag)
    return "true";

  return "false";
}

//==============================================================================
void DefaultTestableOutcome::setSatisfiedFlag(bool satisfiedFlag)
{
  mSatisfiedFlag = satisfiedFlag;
}

} // namespace constraint
} // namespace aikido
