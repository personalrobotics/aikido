#ifndef AIKIDO_CONSTRAINT_PROJECTABLE_H
#define AIKIDO_CONSTRAINT_PROJECTABLE_H

#include <Eigen/Dense>
#include "../state/State.hpp"
#include <boost/optional.hpp>

namespace aikido {
namespace constraint{

class Projectable
{
public:

  /// True if this Projectable contains _s
  virtual bool contains(const state::State& _s) const = 0;

  /// Returns projection of _q in this constraint.
  virtual boost::optional<state::State> project(
  	const state::State& _s) const = 0;
};

using ProjectablePtr = std::shared_ptr<const Projectable&>;


} // constraint
} // aikido

#endif
