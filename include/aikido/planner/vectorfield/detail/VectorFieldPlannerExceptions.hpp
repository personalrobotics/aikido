#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_H_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_H_
#include <string>
#include <stdexcept>
#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Define runtime error for the termination of vectorfield planner.
///
class VectorFieldTerminated : public std::runtime_error {
public:
  /// Constructor
  ///
  /// \param[in] _whatArg Error string
  VectorFieldTerminated(const std::string& _whatArg);
};

/// Define termination error of vectorfield planner due to DOF limit error.
///
class DofLimitError : public VectorFieldTerminated {
public:
  /// Constructor
  ///
  /// \param[in] _dof Degree of freedom
  /// \param[in] _whatArg Error string
  DofLimitError(dart::dynamics::DegreeOfFreedom *_dof,
    const std::string& _whatArg);

  /// Get degree of freedom
  ///
  /// \return Degree of freedom
  dart::dynamics::DegreeOfFreedom *dof() const;

private:
  dart::dynamics::DegreeOfFreedom *mDof;
};

} // namespace vectorfield
} // namespace aikido
} // namespace planner

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_H_
