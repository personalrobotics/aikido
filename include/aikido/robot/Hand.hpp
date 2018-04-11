#ifndef AIKIDO_ROBOT_HAND_HPP_
#define AIKIDO_ROBOT_HAND_HPP_

#include <set>
#include <string>
#include <unordered_map>
#include <dart/dart.hpp>
#include "aikido/control/PositionCommandExecutor.hpp"
#include "aikido/io/yaml.hpp"
#include "aikido/robot/GrabMetadata.hpp"

namespace aikido {
namespace robot {

AIKIDO_DECLARE_POINTERS(Hand)

/// Abstract class for Hand interface.
class Hand
{
public:
  virtual ~Hand() = default;

  /// Grabs an object. Immediately executes.
  /// \param[in] bodyToGrab The object to grab
  virtual void grab(const dart::dynamics::SkeletonPtr& bodyToGrab) = 0;

  /// Ungrabs an object. Immediately ungrabs.
  /// \throws a runtime_error if fails.
  virtual void ungrab() = 0;

  /// Sets the hand to the corresponding preshape (from \c preshapesUri).
  /// \param[in] preshapeName Name of preshape (e.g. "open")
  virtual std::future<void> executePreshape(const std::string& preshapeName)
      = 0;

  /// \copydoc Robot::step
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Returns the metaskeleton corresponding to this hand.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() const = 0;

  /// Get the end-effector body node.
  /// \return DART body node of end-effector
  virtual dart::dynamics::BodyNode* getBodyNode() const = 0;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_HAND_HPP_
