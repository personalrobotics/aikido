#ifndef AIKIDO_STATESPACE_SE3STATESPACE_H
#define AIKIDO_STATESPACE_SE3STATESPACE_H
#include <Eigen/Geometry>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

template <class> class SE3StateHandle;

class SE3StateSpace : public virtual StateSpace 
{
public:
  class State : public StateSpace::State
  {
  public:
    using Isometry3d = Eigen::Transform<double, 3, 
                                        Eigen::Isometry, Eigen::DontAlign>;

    // Constructs Identity Element
    State();
    ~State() = default;

    /// Constructs a point in SE(3) from a transfomation.
    explicit State(const Isometry3d& _transform);

    /// Sets value to a transfomation.
    void setIsometry(const Isometry3d& _transform);

    /// Gets value to a transfomation.
    const Isometry3d& getIsometry() const;

  private:
    Isometry3d mTransform;

    friend class SE3StateSpace;
  };

  using StateHandle = SE3StateHandle<State>;
  using StateHandleConst = SE3StateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using Isometry3d = State::Isometry3d;

  SE3StateSpace() = default;

  ScopedState createState() const;

  /// Gets value as a transformation.
  const Isometry3d& getIsometry(const State* _state) const;

  /// Sets value to a transformation.
  void setIsometry(State* _state, const Isometry3d& _transform) const;

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

  // Documentation inherited. _tangent should be 6d twist (w, v).
  void expMap(const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

};

} // namespace statespace
} // namespace aikido

#include "detail/SE3StateSpace.hpp"

#endif
