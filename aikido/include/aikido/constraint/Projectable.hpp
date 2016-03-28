#ifndef AIKIDO_CONSTRAINT_PROJECTABLE_H
#define AIKIDO_CONSTRAINT_PROJECTABLE_H

#include <Eigen/Dense>
#include "../state/State.hpp"
#include <boost/optional.hpp>

namespace aikido {
namespace constraint{

/// A projectable constraint.
class Projectable
{
public:

  /// True if this Projectable contains _s
  virtual bool contains(const state::StatePtr& _s) const = 0;

  /// Returns projection of _q in this constraint.
  virtual boost::optional<state::StatePtr> project(
  	const state::StatePtr& _s) = 0;
};

using ProjectablePtr = std::shared_ptr<const Projectable>;


} // constraint
} // aikido

#endif
