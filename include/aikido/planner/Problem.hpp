#ifndef AIKIDO_PLANNER_PROBLEM_HPP_
#define AIKIDO_PLANNER_PROBLEM_HPP_

#include <string>
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {

/// Base class for various planning problems.
class Problem
{
public:
  class Result;

  /// Constructor.
  explicit Problem(statespace::ConstStateSpacePtr stateSpace);

  /// Destructor.
  virtual ~Problem() = default;

  /// Returns name.
  virtual const std::string& getName() const = 0;

  /// Returns const state space.
  statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  /// State space associated with this problem.
  statespace::ConstStateSpacePtr mStateSpace;
};

/// Base class for planning result of various planning problems.
class Problem::Result
{
public:
  /// Destructor.
  virtual ~Result() = default;

  /// Sets message.
  void setMessage(const std::string& message);

  /// Returns message.
  const std::string& getMessage() const;

protected:
  /// Message.
  std::string mMessage;
};

} // namespace planner
} // namespace aikido

#endif
