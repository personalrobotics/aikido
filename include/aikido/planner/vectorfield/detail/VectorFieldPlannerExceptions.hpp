#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_

#include <stdexcept>
#include <string>
#include <dart/dynamics/dynamics.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// Define termination of vectorfield planner.
// When VectorFieldTerminated is throws, it means
// that planning is finished.
class VectorFieldTerminated : public std::exception
{
public:
  /// Constructor.
  ///
  /// \param[in] _whatArg Error string.
  VectorFieldTerminated(const std::string& whatArg);

  const char* what() const throw();

protected:
  std::string mWhatArg;
};

/// Define runtime error of vector field planner.
// When VectorFieldError is thrown, it means that
// planning encounters an error.
class VectorFieldError : public std::runtime_error
{
public:
  /// Constructor.
  ///
  /// \param[in] whatArg Error string.
  VectorFieldError(const std::string& whatArg);
};

/// Define termination error of vectorfield planner due to DOF limit error.
class DofLimitError : public VectorFieldError
{
public:
  /// Constructor.
  ///
  DofLimitError();
};

/// Define state in collision for the termination of vector field planner.
class ConstraintViolatedError : public VectorFieldError
{
public:
  /// Constructor.
  ///
  ConstraintViolatedError();
};

/// Define integration failure for the termination of vectorfield planner.
class IntegrationFailedError : public VectorFieldError
{
public:
  /// Constructor.
  ///
  IntegrationFailedError();
};

/// Define time limit error for the termination of vectorfield planner.
class TimeLimitError : public VectorFieldError
{
public:
  /// Constructor.
  ///
  TimeLimitError();
};

} // namespace vectorfield
} // namespace aikido
} // namespace planner

#endif // ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNEREXCEPTIONS_HPP_
