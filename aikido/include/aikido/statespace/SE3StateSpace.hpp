#ifndef AIKIDO_STATESPACE_SE3STATESPACE_H
#define AIKIDO_STATESPACE_SE3STATESPACE_H
#include <Eigen/Geometry>
#include "CompoundStateSpace.hpp"

namespace aikido {
namespace statespace {

template <class> class SE3StateHandle;

class SE3StateSpace : public virtual CompoundStateSpace 
{
public:
  class State : public CompoundStateSpace::State
  {
  protected:
    State() = default;
    ~State() = default;
  };

  using StateHandle = SE3StateHandle<State>;
  using StateHandleConst = SE3StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  SE3StateSpace();

  ScopedState createState() const;

  /// Gets value as a transformation.
  Eigen::Isometry3d getIsometry(const State* _state) const;

  /// Sets value to a transformation.
  void setIsometry(State* _state, const Eigen::Isometry3d& _transform) const;
};

} // namespace statespace
} // namespace aikido

#include "detail/SE3StateSpace.hpp"

#endif
