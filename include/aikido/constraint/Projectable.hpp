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
  /// \param _s state to be projected.
  /// \param _out resulting projection.
  virtual bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const = 0;

  /// Performs an in-place projection.
  /// By default, it creates a copy and then calls the two-parameter project.
  /// \param _s state to be projected and mutated.
  virtual bool project(statespace::StateSpace::State* _s) const;

};

using ProjectablePtr = std::shared_ptr<Projectable>;

} // namespace constraint
} // namespace aikido

#endif
