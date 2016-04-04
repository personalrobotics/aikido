#ifndef AIKIDO_STATESPACE_SE2STATESPACE_H
#define AIKIDO_STATESPACE_SE2STATESPACE_H
#include <Eigen/Geometry>
#include "CompoundStateSpace.hpp"

namespace aikido {
namespace statespace {

class SE2StateSpace : public CompoundStateSpace 
{
public:
  class State : public CompoundStateSpace::State
  {
  public:
    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a point in SE(2) from a transformation.
    explicit State(const Eigen::Isometry2d& _transform);

    /// Gets value as a transformation.
    Eigen::Isometry2d getIsometry() const;

    /// Sets value to a transformation.
    void setIsometry(const Eigen::Isometry2d& _transform);
  };

  SE2StateSpace();

  // Documentation inherited.
  StateSpace::State* allocateState() const override;

  // Documentation inherited.
  void freeState(StateSpace::State* _state) const override;
};

} // namespace statespace
} // namespace aikido

#endif
