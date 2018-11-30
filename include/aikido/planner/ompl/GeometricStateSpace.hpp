#ifndef AIKIDO_OMPL_AIKIDOGEOMETRICSTATESPACE_HPP_
#define AIKIDO_OMPL_AIKIDOGEOMETRICSTATESPACE_HPP_

#include <ompl/base/StateSpace.h>
#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/distance/DistanceMetric.hpp"
#include "aikido/planner/ompl/BackwardCompatibility.hpp"
#include "aikido/statespace/GeodesicInterpolator.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace planner {
namespace ompl {

AIKIDO_DECLARE_POINTERS(GeometricStateSpace)

/// The maximum distance between two states for them to be considered equal
constexpr double EQUALITY_EPSILON = 1e-7;

/// Wraps an aikido StateSpace into a space recognized by OMPL
class GeometricStateSpace : public ::ompl::base::StateSpace
{
public:
  /// Wraps an aikido::statespace::StateSpace::State in an OMPL StateType
  class StateType : public ::ompl::base::State
  {
  public:
    /// Constructor.
    /// \param[in] state The state to wrap
    explicit StateType(statespace::StateSpace::State* state);

    /// The wrapped aikido state
    statespace::StateSpace::State* mState;

    /// Indicates whether the state has been initialized to represent a valid
    /// state. This allows samplers to indicate failure by setting this flag to
    /// false when sampling fails. The StateValidityChecker should check this
    /// flag when determining if the state is valid.
    /// The value defaults to true.
    bool mValid;
  };

  /// Construct a state space
  /// \param[in] sspace The aikido::statespace::StateSpace to expose to OMPL
  /// \param[in] interpolator An aikido interpolator used by the interpolate
  /// method
  /// \param[in] dmetric The distance metric to use to compute distance between
  /// two states in the StateSpace
  /// \param[in] sampler A state sampler used to sample new states in the
  /// StateSpace
  /// \param[in] boundsConstraint A Testable used to determine whether
  /// states fall with in bounds defined on the space.
  /// \param[in] boundsProjection A Projectable that can be used to project a
  /// state back within the valid boundary defined on the space.
  /// \param[in] maxDistanceBetweenValidityChecks The maximum distance (under
  /// dmetric) between validity checking two successive points on a tree
  /// extension or an edge in a graph. Defines the "discreteness" of statespace.
  GeometricStateSpace(
      statespace::ConstStateSpacePtr sspace,
      statespace::ConstInterpolatorPtr interpolator,
      distance::DistanceMetricPtr dmetric,
      constraint::SampleablePtr sampler,
      constraint::ConstTestablePtr boundsConstraint,
      constraint::ProjectablePtr boundsProjection,
      double maxDistanceBetweenValidityChecks);

  /// Get the dimension of the space.
  unsigned int getDimension() const override;

  /// Get the maximum value a call to distance() can return (or an upper bound).
  /// For unbounded state spaces, this function can return infinity.
  double getMaximumExtent() const override;

#if OMPL_VERSION_AT_LEAST(1, 0, 0)
  /// Get a measure of the space.
  double getMeasure() const override;
#else
  double getMeasure() const;
#endif

  /// Bring the state within the bounds of the statespace using
  /// boundsProjection.
  /// \param[in] state The state to modify.
  void enforceBounds(::ompl::base::State* state) const override;

  /// Check if a state satisfies the boundsConstraint.
  /// \param[in] state The state to check.
  bool satisfiesBounds(const ::ompl::base::State* state) const override;

  /// Copy the value of one state to another.
  /// \param[out] destination The state to copy to.
  /// \param[in] source The state to copy from.
  void copyState(
      ::ompl::base::State* destination,
      const ::ompl::base::State* source) const override;

  /// Computes distance between two states using the dmetric.
  /// \param[in] state1 The first state.
  /// \param[in] state2 The second state.
  double distance(
      const ::ompl::base::State* state1,
      const ::ompl::base::State* state2) const override;

  /// Check state equality. Returns true if the distance between the states
  /// less than EQUALITY_EPSILON.
  /// \param[in] state1 The first state
  /// \param[in] state2 The second state
  bool equalStates(
      const ::ompl::base::State* state1,
      const ::ompl::base::State* state2) const override;

  /// Computes the state that lies at time t in [0, 1] on the segment
  /// that connects from state to to state.
  /// \param[in] from The state that begins the segment.
  /// \param[in] to The state that ends the segment.
  /// \param[in] t The interpolation parameter (between 0 and 1).
  /// \param[out] state The result of the interpolation.
  void interpolate(
      const ::ompl::base::State* from,
      const ::ompl::base::State* to,
      double t,
      ::ompl::base::State* state) const override;

  /// Allocate an instance of the state sampler for this space.
  ::ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  /// Allocate a state that can store a point in the described space
  ::ompl::base::State* allocState() const override;

  /// Allocate a state constaining a copy of the aikido state
  /// \param[in] state The aikido state to copy and wrap in an OMPL state
  ::ompl::base::State* allocState(
      const statespace::StateSpace::State* state) const;

  /// Free the memory of the allocated state. This also frees the memory of the
  /// wrapped aikido state.
  /// \param[in] state The state to free.
  void freeState(::ompl::base::State* state) const override;

  /// Return the Aikido StateSpace that this OMPL StateSpace wraps
  statespace::ConstStateSpacePtr getAikidoStateSpace() const;

  /// Return the interpolator used to interpolate between states in the space.
  aikido::statespace::ConstInterpolatorPtr getInterpolator() const;

  /// Return the bounds constraint for the statespace. Used to specify
  /// constraints to the OMPL planner.
  aikido::constraint::ConstTestablePtr getBoundsConstraint() const;

  /// Returns the collision checking resolution.
  double getMaxDistanceBetweenValidityChecks() const;

private:
  /// The AIKIDO statespace to be exposed to OMPL.
  statespace::ConstStateSpacePtr mStateSpace;

  /// An aikido interpolator to interpolate between states of the statespace.
  statespace::ConstInterpolatorPtr mInterpolator;

  /// Distance metric to compute distance between states in the statespace.
  distance::DistanceMetricPtr mDistance;

  /// State sampler used to sample states in the statespace.
  constraint::SampleablePtr mSampler;

  /// Constraint to determine if a state is within statespace bounds.
  constraint::ConstTestablePtr mBoundsConstraint;

  /// A Projectable that projects a state within valid bounds of statespace.
  constraint::ProjectablePtr mBoundsProjection;

  /// Collision checking resolution.
  double mMaxDistanceBetweenValidityChecks;
};

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif
