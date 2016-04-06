#ifndef MUUL_PROJECTION_VECTORFIELDPLANNEREXCEPTIONS_H_
#define MUUL_PROJECTION_VECTORFIELDPLANNEREXCEPTIONS_H_
#include <string>
#include <stdexcept>
#include <dart/dynamics/SmartPointer.h>

namespace muul {
namespace projection {

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

} // namespace projection
} // namespace muul

#endif // ifndef MUUL_PROJECTION_VECTORFIELDPLANNEREXCEPTIONS_H_
