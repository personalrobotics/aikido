#ifndef AIKIDO_STATESPACE_SE2STATESPACE_H
#define AIKIDO_STATESPACE_SE2STATESPACE_H
#include <Eigen/Geometry>
#include "CompoundStateSpace.hpp"

namespace aikido {
namespace statespace {

template <class> class SE2StateHandle;

class SE2StateSpace : public virtual CompoundStateSpace 
{
public:
  class State : public CompoundStateSpace::State
  {
  protected:
    State() = default;
    ~State() = default;
  };

  using StateHandle = SE2StateHandle<State>;
  using StateHandleConst = SE2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  SE2StateSpace();

  ScopedState createState() const;

  /// Gets value as a transformation.
  Eigen::Isometry2d getIsometry(const State* _state) const;

  /// Sets value to a transformation.
  void setIsometry(State* _state, const Eigen::Isometry2d& _transform) const;
};

} // namespace statespace
} // namespace aikido

#include "detail/SE2StateSpace.hpp"

#endif
