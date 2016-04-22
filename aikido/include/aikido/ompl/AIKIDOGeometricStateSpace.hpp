#ifndef AIKIDO_OMPL_AIKIDOGEOMETRICSTATESPACE_HPP_
#define AIKIDO_OMPL_AIKIDOGEOMETRICSTATESPACE_HPP_

#include <ompl/base/StateSpace.h>
#include "../constraint/Sampleable.hpp"
#include "../constraint/TestableConstraint.hpp"
#include "../constraint/Projectable.hpp"
#include "../distance/DistanceMetric.hpp"
#include "../statespace/StateSpace.hpp"
#include "../statespace/GeodesicInterpolator.hpp"

namespace aikido {
namespace ompl {
/// The maximum distance between two states for them to still be considered
/// equal
constexpr double EQUALITY_EPSILON = 1e-7;

/// Wraps an aikido StateSpace into a space recognized by OMPL
class GeometricStateSpace : public ::ompl::base::StateSpace
{
public:
  /// Wraps an aikido::statespace::StateSpace::State in an OMPL StateType
  class StateType : public ::ompl::base::State
  {
  public:
    /// Constructor
    /// \param _st The state to wrap
    StateType(statespace::StateSpace::State *_st);

    statespace::StateSpace::State *mState;
  };

  /// Construct a state space
  /// \param _sspace The aikido::statespace::StateSpace to expose to OMPL
  /// \param _interpolator An aikido interpolator used by the interpolate method
  /// \param _dmetric The distance metric to use to compute distance between two
  /// states in the StateSpace
  /// \param sampler A state sampler used to sample new states in the StateSpace
  /// \param boundsConstraint A TestableConstraint used to determine whether
  /// states fall with in bounds defined on the space.
  /// \param boundsProjection A Projectable that can be used to project a state
  /// back within the valid boundary defined on the space.
  GeometricStateSpace(statespace::StateSpacePtr _sspace,
                      statespace::InterpolatorPtr _interpolator,
                      distance::DistanceMetricPtr _dmetric,
                      constraint::SampleableConstraintPtr _sampler,
                      constraint::TestableConstraintPtr _boundsConstraint,
                      constraint::ProjectablePtr _boundsProjection);

  /// Get the dimension of the space.
  unsigned int getDimension() const override;

  /// Get the maximum value a call to distance() can return (or an upper bound).
  /// For unbounded state spaces, this function can return infinity.
  double getMaximumExtent() const override;

  /// Get a measure of the space.
  double getMeasure() const override;

  /// Bring the state within the bounds of the state space using the
  /// boundsProjection defined in the constructor.
  /// \param _state The state to modify
  void enforceBounds(::ompl::base::State *_state) const override;

  /// Check if a state satisfieds the boundsConstraint defined in the
  /// constructor
  /// \param _state The state to check
  bool satisfiesBounds(const ::ompl::base::State *_state) const override;

  /// Copy the value of one state to another
  /// \param[out] _destination The state to copy to
  /// \param _source The state to copy from
  void copyState(::ompl::base::State *_destination,
                 const ::ompl::base::State *_source) const override;

  /// Computes distance between two states using the _dmetric defined in the
  /// constructor.
  /// \param _state1 The first state
  /// \param _state2 The second state
  double distance(const ::ompl::base::State *_state1,
                  const ::ompl::base::State *_state2) const override;

  /// Check state equality. The returns true if the distance between the states
  /// is 0.
  /// \param _state1 The first state
  /// \param _state2 The second state
  bool equalStates(const ::ompl::base::State *_state1,
                   const ::ompl::base::State *_state2) const override;

  /// Computes the state that lies at time t in [0, 1] on the segment
  /// that connects from state to to state.
  /// \param _from The state that begins the segment
  /// \param _to The state that ends the segment
  /// \param _t The interpolation parameter (between 0 and 1)
  /// \param[out] _state The result of the interpolation
  void interpolate(const ::ompl::base::State *_from,
                   const ::ompl::base::State *_to, const double _t,
                   ::ompl::base::State *_state) const override;

  /// Allocate an instance of the state sampler for this space.
  ::ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  /// Allocate a state that can store a point in the described space
  ::ompl::base::State *allocState() const override;

  /// Allocate a state constaining a copy of the aikido state
  /// \param _state The aikido state to copy and wrap in an OMPL state
  ::ompl::base::State *allocState(
      const statespace::StateSpace::State *_state) const;

  /// Free the memory of the allocated state. This also frees the memory of the
  /// wrapped aikido state.
  /// \param _state The state to free.
  void freeState(::ompl::base::State *_state) const override;

private:
  statespace::StateSpacePtr mStateSpace;
  statespace::InterpolatorPtr mInterpolator;
  distance::DistanceMetricPtr mDistance;
  constraint::SampleableConstraintPtr mSampler;
  constraint::TestableConstraintPtr mBoundsConstraint;
  constraint::ProjectablePtr mBoundsProjection;
};

using GeometricStateSpacePtr = std::shared_ptr<GeometricStateSpace>;
}
}
#endif
