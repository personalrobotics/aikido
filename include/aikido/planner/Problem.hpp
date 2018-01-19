#ifndef AIKIDO_PLANNER_PROBLEM_HPP_
#define AIKIDO_PLANNER_PROBLEM_HPP_

#include <string>
#include "aikido/statespace/smart_pointer.hpp"

namespace aikido {
namespace planner {

/// Base class for various planning problems.
class Problem
{
public:
  explicit Problem(statespace::ConstStateSpacePtr stateSpace);
  virtual ~Problem() = default;

  class Result;

  virtual const std::string& getName() const = 0;

  statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  statespace::ConstStateSpacePtr mStateSpace;
};

class Problem::Result
{
public:
  virtual ~Result() = default;
  void setMessage(const std::string& message);
  const std::string& getMessage() const;

protected:
  std::string mMessage;
};

} // namespace planner
} // namespace aikido

#endif
