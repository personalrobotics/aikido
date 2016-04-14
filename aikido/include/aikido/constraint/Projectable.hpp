#ifndef AIKIDO_CONSTRAINT_PROJECTABLE_H
#define AIKIDO_CONSTRAINT_PROJECTABLE_H

#include <Eigen/Dense>
#include "../statespace/StateSpace.hpp"

namespace aikido {
namespace constraint{

/// A projectable constraint.
class Projectable
{
public:
  virtual ~Projectable() = default;

  /// Gets the StateSpace that this constraint operates on.
  virtual statespace::StateSpacePtr getStateSpace() const = 0;

  /// Projection _s to _out. Returns false if projection cannot be done. 
  virtual bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const = 0;
};

using ProjectablePtr = std::shared_ptr<const Projectable>;


} // constraint
} // aikido

#endif
