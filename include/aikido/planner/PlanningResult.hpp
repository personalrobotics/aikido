#ifndef AIKIDO_PLANNER_PLANNINGRESULT_HPP_
#define AIKIDO_PLANNER_PLANNINGRESULT_HPP_

#include <string>

namespace aikido {
namespace planner {

class PlanningResult
{
public:
  /// Sets message.
  void setMessage(const std::string& message);

  /// Returns message.
  const std::string& getMessage() const;

public: // TODO: Change this to protected
  /// Message
  std::string mMessage;

  // TODO: Remove
  std::string message;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_PLANNINGRESULT_HPP_
