#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Define runtime error for the termination of vectorfield planner.
class VectorFieldTerminated : public std::runtime_error
{
public:
  /// Constructor
  ///
  /// \param[in] _whatArg Error string
  VectorFieldTerminated(const std::string& _whatArg);
};

/// Define termination error of vectorfield planner due to DOF limit error.
class DofLimitError : public VectorFieldTerminated
{
public:
  /// Constructor
  ///
  /// \param[in] _dof Degree of freedom name
  /// \param[in] _whatArg Error string
  DofLimitError(const std::string& _dof, const std::string& _whatArg);

  /// Get degree of freedom name
  ///
  /// \return Degree of freedom name
  const std::string dof() const;

private:
  const std::string mDof;
};

} // namespace vectorfield
} // namespace aikido
} // namespace planner

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_
