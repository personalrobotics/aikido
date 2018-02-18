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

/// A base class for a hand used by manipulators
class Hand
{
public:
  virtual ~Hand() = default;

  /// Grabs an object. Immediately executes.
  /// \param bodyToGrab The object to grab
  /// \return bool for success
  virtual void grab(const dart::dynamics::SkeletonPtr& bodyToGrab) = 0;

  /// Ungrabs an object. Immediately ungrabs.
  /// Throws a runtime_error if fails.
  virtual void ungrab() = 0;

  /// Sets the hand to the corresponding preshape (from \c preshapesUri).
  /// \param preshapeName Name of preshape (e.g. "open")
  /// \throw a runtime_error if execution fails.
  virtual void executePreshape(const std::string& preshapeName) = 0;

  /// Executes the preshape trajectory upto timepoint..
  // \param timepoint Time to simulate to.
  virtual void step(const std::chrono::system_clock::time_point& timepoint) = 0;

  /// Returns the metaskeleton corresponding to this hand.
  virtual dart::dynamics::MetaSkeletonPtr getMetaSkeleton() = 0;
};

} // namespace robot
} // namespace aikido

#endif
