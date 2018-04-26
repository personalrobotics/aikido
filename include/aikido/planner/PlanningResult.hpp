#ifndef AIKIDO_PLANNER_PLANNINGRESULT_HPP_
#define AIKIDO_PLANNER_PLANNINGRESULT_HPP_

#include <string>

namespace aikido {
namespace planner {

// TODO(JS): This should be removed once all the legacy planning functions are
// converted to the new planning API.
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
