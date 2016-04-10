#ifndef AIKIDO_STATESPACE_SE2STATESPACE_H
#define AIKIDO_STATESPACE_SE2STATESPACE_H
#include <Eigen/Geometry>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

template <class> class SE2StateHandle;

class SE2StateSpace : public virtual StateSpace  
{
public:
  class State : public StateSpace::State
  {
  public:
    using Isometry2d = Eigen::Transform<double, 2, 
                                        Eigen::Isometry, Eigen::DontAlign>;
    
    // Constructs identity element
    State();
    ~State() = default;

    /// Constructs a point in SE(2) from a transfomation.
    explicit State(const Isometry2d& _transform);

    /// Sets value to a transfomation.
    void setIsometry(const Isometry2d& _transform);

    /// Gets value to a transfomation.
    const Isometry2d& getIsometry() const;

  private:
    Isometry2d mTransform;

    friend class SE2StateSpace;

  };

  using StateHandle = SE2StateHandle<State>;
  using StateHandleConst = SE2StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using Isometry2d = State::Isometry2d;

  SE2StateSpace()=default;

  ScopedState createState() const;

  /// Gets value as a transformation.
  const Isometry2d& getIsometry(const State* _state) const;

  /// Sets value to a transformation.
  void setIsometry(State* _state, const Isometry2d& _transform) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

  // Documentation inherited.
  SampleableConstraintPtr createSampleableConstraint(
    std::unique_ptr<util::RNG> _rng) const override;
  
  // Documentation inherited.
  void compose(
    const StateSpace::State* _state1, const StateSpace::State* _state2,
    StateSpace::State* _out) const override;

  // Documentation inherited
  unsigned int getDimension() const override;

  // Documentation inherited
  double getMaximumExtent() const override;

  // Documentation inherited
  double getMeasure() const override;

  // Documentation inherited
  void enforceBounds(StateSpace::State* _state) const override;

  // Documentation inherited
  bool satisfiesBounds(const StateSpace::State* _state) const override;

  // Documentation inherited
  void copyState(StateSpace::State* _destination,
                 const StateSpace::State* _source) const override;

  // Documentation inherited
  double distance(const StateSpace::State* _state1,
                  const StateSpace::State* _state2) const override;

  // Documentation inherited
  bool equalStates(const StateSpace::State* _state1,
                   const StateSpace::State* _state2) const override;

  // Documentation inherited
  void interpolate(const StateSpace::State* _from,
                   const StateSpace::State* _to,
                   const double _t,
                   StateSpace::State* _State) const;

  // Documentation inherited. _tangent should be 3d twist (w, v).
  void expMap(
    const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

};

} // namespace statespace
} // namespace aikido

#include "detail/SE2StateSpace.hpp"

#endif
