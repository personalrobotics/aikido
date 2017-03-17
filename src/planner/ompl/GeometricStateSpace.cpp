#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/BackwardCompatibility.hpp>
#include <aikido/planner/ompl/StateSampler.hpp>
#include <aikido/constraint/Sampleable.hpp>
#include <dart/common/StlHelpers.hpp>
#include <boost/make_shared.hpp>

using dart::common::make_unique;

namespace aikido {
namespace planner {
namespace ompl {

//=============================================================================
GeometricStateSpace::StateType::StateType(statespace::StateSpace::State *_st)
    : mState(_st), mValid(true)
{
}

//=============================================================================
GeometricStateSpace::GeometricStateSpace(
    statespace::StateSpacePtr _sspace,
    statespace::InterpolatorPtr _interpolator,
    distance::DistanceMetricPtr _dmetric,
    constraint::SampleablePtr _sampler,
    constraint::TestablePtr _boundsConstraint,
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
  if (state == nullptr || state->mState == nullptr)
    throw std::invalid_argument("enforceBounds called with null state");

  if (!state->mValid)
    throw std::invalid_argument("enforceBounds called with invalid state");

  auto temporaryState = mStateSpace->createState();
  mBoundsProjection->project(state->mState, temporaryState);
  mStateSpace->copyState(temporaryState, state->mState);
}

//=============================================================================
bool GeometricStateSpace::satisfiesBounds(
    const ::ompl::base::State *_state) const
{
  auto state = static_cast<const StateType *>(_state);
  if( state == nullptr || state->mState == nullptr )
    return false;
  if (!state->mValid)
    return false;

  return mBoundsConstraint->isSatisfied(state->mState);
}

//=============================================================================
void GeometricStateSpace::copyState(
    ::ompl::base::State *_destination, const ::ompl::base::State *_source) const
{
  auto dst = static_cast<StateType *>(_destination);
  if (dst == nullptr || dst->mState == nullptr)
    throw std::invalid_argument("copyState called with null destination");
  auto sst = static_cast<const StateType *>(_source);
  if (sst == nullptr || sst->mState == nullptr)
    throw std::invalid_argument("copyState called with null source state");
  mStateSpace->copyState(sst->mState, dst->mState);
  dst->mValid = sst->mValid;
}

//=============================================================================
double GeometricStateSpace::distance(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  auto state1 = static_cast<const StateType *>(_state1);
  auto state2 = static_cast<const StateType *>(_state2);

  if (state1 == nullptr || state1->mState == nullptr)
    throw std::invalid_argument("distance called with null state1");
  if (state2 == nullptr || state2->mState == nullptr)
    throw std::invalid_argument("distance called with null state2");
  if (!state1->mValid)
    throw std::invalid_argument("distance called with invalid state1");
  if (!state2->mValid)
    throw std::invalid_argument("distance called with invaid state2");

  return mDistance->distance(state1->mState, state2->mState);
}

//=============================================================================
bool GeometricStateSpace::equalStates(
    const ::ompl::base::State *_state1,
    const ::ompl::base::State *_state2) const
{
  auto state1 = static_cast<const StateType *>(_state1);
  auto state2 = static_cast<const StateType *>(_state2);
  if (state1->mValid != state2->mValid)
    return false;

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
  if (from == nullptr || from->mState == nullptr)
    throw std::invalid_argument("interpolate called with null from state");
  auto to = static_cast<const StateType *>(_to);
  if (to == nullptr || to->mState == nullptr)
    throw std::invalid_argument("interpolate called with null to state");
  auto state = static_cast<StateType *>(_state);
  if (state == nullptr || state->mState == nullptr)
    throw std::invalid_argument("interpolate called with null out state");
  if (!from->mValid)
    throw std::invalid_argument("interpolate called with invalid from state");
  if (!to->mValid)
    throw std::invalid_argument("interpolate called with invalid to state");

  mInterpolator->interpolate(from->mState, to->mState, _t, state->mState);
}

//=============================================================================
::ompl::base::StateSamplerPtr
GeometricStateSpace::allocDefaultStateSampler() const
{
  auto generator = mSampler->createSampleGenerator();
  auto stateSampler =
      ompl_make_shared<StateSampler>(this, std::move(generator));

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
  if (_state != nullptr) {
    auto st = static_cast<StateType *>(_state);
    if (st->mState != nullptr) {
      mStateSpace->freeState(st->mState);
    }
    delete st;
  }
}

//=============================================================================
statespace::StateSpacePtr GeometricStateSpace::getAikidoStateSpace() const {
  return mStateSpace;
}
}
}
}
