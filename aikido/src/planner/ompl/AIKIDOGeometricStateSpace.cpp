#include <aikido/planner/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/planner/ompl/AIKIDOStateSampler.hpp>
#include <aikido/constraint/Sampleable.hpp>
#include <dart/common/StlHelpers.h>
#include <boost/make_shared.hpp>

using dart::common::make_unique;

namespace aikido {
namespace ompl {

//=============================================================================
GeometricStateSpace::StateType::StateType(statespace::StateSpace::State *_st)
    : mState(_st)
{
}

//=============================================================================
GeometricStateSpace::GeometricStateSpace(
    statespace::StateSpacePtr _sspace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleableConstraintPtr _sampler,
    constraint::TestableConstraintPtr _boundsConstraint,
    constraint::ProjectablePtr _boundsProjection)
    : mStateSpace(std::move(_sspace))
    , mInterpolator(std::move(_interpolator))
    , mDistance(std::move(_dmetric))
    , mSampler(std::move(_sampler))
    , mBoundsConstraint(std::move(_boundsConstraint))
    , mBoundsProjection(std::move(_boundsProjection))
{
  if (mStateSpace == nullptr) {
    throw std::invalid_argument("StateSpace is nullptr");
  }

  if (mInterpolator == nullptr) {
    throw std::invalid_argument("Interpolator is nullptr");
  }

  if (mInterpolator->getStateSpace() != mStateSpace) {
    throw std::invalid_argument("Interpolator does not match StateSpace");
  }

  if (mDistance == nullptr) {
    throw std::invalid_argument("DistanceMetric is nullptr");
  }

  if (mDistance->getStateSpace() != mStateSpace) {
    throw std::invalid_argument("DistanceMetric does not match StateSpace");
  }

  if (mSampler == nullptr) {
    throw std::invalid_argument("Sampler is nullptr");
  }

  if (mSampler->getStateSpace() != mStateSpace) {
    throw std::invalid_argument("Sampler does not match StateSpace");
  }

  if (mBoundsConstraint == nullptr) {
    throw std::invalid_argument("BoundsConstraint is nullptr");
  }

  if (mBoundsConstraint->getStateSpace() != mStateSpace) {
    throw std::invalid_argument("BoundConstraint does not match StateSpace");
  }

  if (mBoundsProjection == nullptr) {
    throw std::invalid_argument("BoundsProjection is nullptr");
  }

  if (mBoundsProjection->getStateSpace() != mStateSpace) {
    throw std::invalid_argument("BoundsProjection does not match StateSpace");
  }
}

//=============================================================================
unsigned int GeometricStateSpace::getDimension() const
{
  return mStateSpace->getDimension();
}

//=============================================================================
double GeometricStateSpace::getMaximumExtent() const
{
  return std::numeric_limits<double>::infinity();
}

//=============================================================================
double GeometricStateSpace::getMeasure() const
{
  throw std::runtime_error("getMeasure not implemented.");
}

//=============================================================================
void GeometricStateSpace::enforceBounds(::ompl::base::State *_state) const
{
  auto state = static_cast<const StateType *>(_state);
  auto temporaryState = mStateSpace->createState();
  mBoundsProjection->project(state->mState, temporaryState);
  mStateSpace->copyState(temporaryState, state->mState);
}

//=============================================================================
bool GeometricStateSpace::satisfiesBounds(
    const ::ompl::base::State *_state) const
{
  auto state = static_cast<const StateType *>(_state);
  return mBoundsConstraint->isSatisfied(state->mState);
}

//=============================================================================
void GeometricStateSpace::copyState(
    ::ompl::base::State *_destination, const ::ompl::base::State *_source) const
{
  auto dst = static_cast<StateType *>(_destination);
  auto sst = static_cast<const StateType *>(_source);
  mStateSpace->copyState(sst->mState, dst->mState);
}

//=============================================================================
double GeometricStateSpace::distance(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  auto state1 = static_cast<const StateType *>(_state1);
  auto state2 = static_cast<const StateType *>(_state2);

  return mDistance->distance(state1->mState, state2->mState);
}

//=============================================================================
bool GeometricStateSpace::equalStates(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  double dist = distance(_state1, _state2);
  return dist < EQUALITY_EPSILON;
}

//=============================================================================
void GeometricStateSpace::interpolate(const ::ompl::base::State *_from,
                                            const ::ompl::base::State *_to,
                                            const double _t,
                                            ::ompl::base::State *_state) const
{
  auto from = static_cast<const StateType *>(_from);
  auto to = static_cast<const StateType *>(_to);
  auto state = static_cast<StateType *>(_state);
  mInterpolator->interpolate(from->mState, to->mState, _t, state->mState);
}

//=============================================================================
::ompl::base::StateSamplerPtr
GeometricStateSpace::allocDefaultStateSampler() const
{
  auto generator = mSampler->createSampleGenerator();
  auto stateSampler =
      boost::make_shared<StateSampler>(this, std::move(generator));

  return stateSampler;
}

//=============================================================================
::ompl::base::State *GeometricStateSpace::allocState() const
{
  auto ast = mStateSpace->allocateState();
  return new StateType(ast);
}

//=============================================================================
::ompl::base::State *GeometricStateSpace::allocState(
    const aikido::statespace::StateSpace::State *_state) const
{
  auto newState = mStateSpace->allocateState();
  mStateSpace->copyState(_state, newState);
  return new StateType(newState);
}

//=============================================================================
void GeometricStateSpace::freeState(::ompl::base::State *_state) const
{
  auto st = static_cast<StateType *>(_state);
  mStateSpace->freeState(st->mState);
  delete st;
}
}
}
