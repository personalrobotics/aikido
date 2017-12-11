#include <sstream>
#include <stdexcept>
#include <typeinfo>

namespace aikido {
namespace constraint {

//==============================================================================
template <class Child>
Child* dynamic_cast_if_present(TestableOutcome* outcome)
{
  if (!outcome)
    return nullptr;

  auto childPtr = dynamic_cast<Child*>(outcome);
  if (!childPtr)
  {
    std::stringstream message;
    message << "TestableOutcome pointer is not of type " << typeid(Child).name()
            << ".";
    throw std::invalid_argument(message.str());
  }

  return childPtr;
}

} // namespace constraint
} // namespace aikido
