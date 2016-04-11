#ifndef AIKIDO_PLANNING_VECTORFIELDPLANNEREXCEPTIONS_H_
#define AIKIDO_PLANNING_VECTORFIELDPLANNEREXCEPTIONS_H_
#include <string>
#include <stdexcept>
#include <dart/dynamics/SmartPointer.h>

namespace aikido {
namespace planning {

class VectorFieldTerminated : public std::runtime_error {
public:
  VectorFieldTerminated(std::string const &what_arg);
};

class DofLimitError : public VectorFieldTerminated {
public:
  DofLimitError(dart::dynamics::DegreeOfFreedom *dof,
    std::string const &what_arg);

  dart::dynamics::DegreeOfFreedom *dof() const;

private:
  dart::dynamics::DegreeOfFreedom *dof_;
};

} // namespace planning
} // namespace aikido

#endif // ifndef AIKIDO_PLANNING_VECTORFIELDPLANNEREXCEPTIONS_H_
