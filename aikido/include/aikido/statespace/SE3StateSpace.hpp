#ifndef AIKIDO_STATESPACE_SE3STATESPACE_H
#define AIKIDO_STATESPACE_SE3STATESPACE_H
#include <Eigen/Geometry>
#include "CompoundStateSpace.hpp"

namespace aikido {
namespace statespace {

class SE3StateSpace : public CompoundStateSpace 
{
public:
  class State : public CompoundStateSpace::State
  {
  public:
    /// Constructs the identity element.
    State();

    ~State() = default;

    /// Constructs a point in SE(3) from a transformation.
    explicit State(const Eigen::Isometry3d& _transform);

    /// Gets value as a transformation.
    Eigen::Isometry3d getIsometry() const;

    /// Sets value to a transformation.
    void setIsometry(const Eigen::Isometry3d& _transform);
  };

  SE3StateSpace();

  // Documentation inherited.
  StateSpace::State* allocateState() const override;

  // Documentation inherited.
  void freeState(StateSpace::State* _state) const override;
};

} // namespace statespace
} // namespace aikido

#endif
