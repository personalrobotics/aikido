#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/AIKIDOStateSampler.hpp>
#include <aikido/constraint/Sampleable.hpp>
#include <dart/common/StlHelpers.h>
#include <boost/make_shared.hpp>

using dart::common::make_unique;

namespace aikido
{
namespace ompl
{
AIKIDOGeometricStateSpace::AIKIDOGeometricStateSpace(
    statespace::StateSpacePtr _sspace, distance::DistanceMetricPtr _dmetric,
    constraint::SampleableConstraintPtr _sampler,
    constraint::TestableConstraintPtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjection)
    : mStateSpace(std::move(_sspace))
    , mDistance(std::move(_dmetric))
    , mSampler(std::move(_sampler))
    , mBoundsConstraint(std::move(_boundsConstraint))
    , mBoundsProjection(std::move(_boundsProjection))
{
}

/// Get the dimension of the space (not the dimension of the surrounding ambient
/// space)
unsigned int AIKIDOGeometricStateSpace::getDimension() const
{
  return mStateSpace->getDimension();
}

/// Get the maximum value a call to distance() can return (or an upper bound).
/// For unbounded state spaces, this function can return infinity.
double AIKIDOGeometricStateSpace::getMaximumExtent() const
{
  return std::numeric_limits<double>::infinity();
}

/// Get a measure of the space (this can be thought of as a generalization of
/// volume)
double AIKIDOGeometricStateSpace::getMeasure() const
{
  throw std::runtime_error("getMeasure not implemented.");
}

/// Bring the state within the bounds of the state space.
/// For unbounded spaces this function can be a no-op.
void AIKIDOGeometricStateSpace::enforceBounds(::ompl::base::State *_state) const
{
  auto state = static_cast<const StateType *>(_state);
  auto out = mStateSpace->createState();
  mBoundsProjection->project(state->mState, out);
  mStateSpace->copyState(state->mState, out);
}

/// Check if a state is inside the bounding box.
/// For unbounded spaces this function can always return true.
bool AIKIDOGeometricStateSpace::satisfiesBounds(
    const ::ompl::base::State *_state) const
{
  auto state = static_cast<const StateType *>(_state);
  return mBoundsConstraint->isSatisfied(state->mState);
}

/// Copy a state to another.
void AIKIDOGeometricStateSpace::copyState(
    ::ompl::base::State *_destination, const ::ompl::base::State *_source) const
{
  auto dst = static_cast<StateType *>(_destination);
  auto sst = static_cast<const StateType *>(_source);
  mStateSpace->copyState(dst->mState, sst->mState);
}

/// Computes distance between two states. This function satisfies
/// the properties of a metric if isMetricSpace() is true, and its
/// return value will always be between 0 and getMaximumExtent()
double AIKIDOGeometricStateSpace::distance(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  auto state1 = static_cast<const StateType *>(_state1);
  auto state2 = static_cast<const StateType *>(_state2);

  return mDistance->distance(state1->mState, state2->mState);
}

/// Check state equality
bool AIKIDOGeometricStateSpace::equalStates(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  double dist = distance(_state1, _state2);
  return dist < 1e-7;
}

/// Computes the state that lies at time t in [0, 1] on the segment
/// that connects from state to to state. The memory location of state
/// is not required to be different from the memory of either from or to.
void AIKIDOGeometricStateSpace::interpolate(const ::ompl::base::State *_from,
                                            const ::ompl::base::State *_to,
                                            const double _t,
                                            ::ompl::base::State *_state) const
{
  auto from = static_cast<const StateType *>(_from);
  auto to = static_cast<const StateType *>(_to);
  auto state = static_cast<StateType *>(_state);
  mDistance->interpolate(from->mState, to->mState, _t, state->mState);
}

/// Allocate an instance of the state sampler for this space.
/// This sampler will be allocated with the sampler allocator that
/// was previously specified by setStateSamplerAllocator() or,
/// if no sampler allocator was specified, allocDefaultStateSampler() is called.
::ompl::base::StateSamplerPtr
AIKIDOGeometricStateSpace::allocDefaultStateSampler() const
{
  auto generator = mSampler->createSampleGenerator();
  auto stateSampler =
      boost::make_shared<AIKIDOStateSampler>(this, std::move(generator));

  return stateSampler;
}

/// Allocate a state that can store a point in the described space
::ompl::base::State *AIKIDOGeometricStateSpace::allocState() const
{
  auto ast = mStateSpace->allocateState();
  return new StateType(ast);
}

/// Allocate a state constaining a copy of the aikido state
::ompl::base::State *AIKIDOGeometricStateSpace::allocState(
    const aikido::statespace::StateSpace::State *_state) const
{
  auto ast = mStateSpace->allocateState();
  mStateSpace->copyState(ast, _state);
  return new StateType(ast);
}

/// Free the memory of the allocated state
void AIKIDOGeometricStateSpace::freeState(::ompl::base::State *_state) const
{
  auto st = static_cast<StateType *>(_state);
  mStateSpace->freeState(st->mState);
  delete st;
}
}
}
