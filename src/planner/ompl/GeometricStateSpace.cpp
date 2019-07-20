#include "aikido/common/memory.hpp"
#include <aikido/constraint/Sampleable.hpp>
#include <aikido/planner/ompl/BackwardCompatibility.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/StateSampler.hpp>

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
GeometricStateSpace::StateType::StateType(statespace::StateSpace::State* st)
  : mState(st), mValid(true)
{
}

//==============================================================================
GeometricStateSpace::GeometricStateSpace(
    statespace::ConstStateSpacePtr sspace,
    statespace::ConstInterpolatorPtr interpolator,
    distance::DistanceMetricPtr dmetric,
    constraint::SampleablePtr sampler,
    constraint::ConstTestablePtr boundsConstraint,
    constraint::ProjectablePtr boundsProjection,
    double maxDistanceBetweenValidityChecks)
  : mStateSpace(std::move(sspace))
  , mInterpolator(std::move(interpolator))
  , mDistance(std::move(dmetric))
  , mSampler(std::move(sampler))
  , mBoundsConstraint(std::move(boundsConstraint))
  , mBoundsProjection(std::move(boundsProjection))
  , mMaxDistanceBetweenValidityChecks(maxDistanceBetweenValidityChecks)
{
  if (mStateSpace == nullptr)
  {
    throw std::invalid_argument("StateSpace is nullptr");
  }

  if (mInterpolator == nullptr)
  {
    throw std::invalid_argument("Interpolator is nullptr");
  }

  if (mInterpolator->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument("Interpolator does not match StateSpace");
  }

  if (mDistance == nullptr)
  {
    throw std::invalid_argument("DistanceMetric is nullptr");
  }

  if (mDistance->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument("DistanceMetric does not match StateSpace");
  }

  if (mSampler == nullptr)
  {
    throw std::invalid_argument("Sampler is nullptr");
  }

  if (mSampler->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument("Sampler does not match StateSpace");
  }

  if (mBoundsConstraint == nullptr)
  {
    throw std::invalid_argument("BoundsConstraint is nullptr");
  }

  if (mBoundsConstraint->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument("BoundConstraint does not match StateSpace");
  }

  if (mBoundsProjection == nullptr)
  {
    throw std::invalid_argument("BoundsProjection is nullptr");
  }

  if (mBoundsProjection->getStateSpace() != mStateSpace)
  {
    throw std::invalid_argument("BoundsProjection does not match StateSpace");
  }

  if (mMaxDistanceBetweenValidityChecks <= 0)
  {
    throw std::invalid_argument("Resolution should be positive");
  }
}

//==============================================================================
unsigned int GeometricStateSpace::getDimension() const
{
  return mStateSpace->getDimension();
}

//==============================================================================
double GeometricStateSpace::getMaximumExtent() const
{
  return std::numeric_limits<double>::infinity();
}

//==============================================================================
double GeometricStateSpace::getMeasure() const
{
  throw std::runtime_error("getMeasure not implemented.");
}

//==============================================================================
void GeometricStateSpace::enforceBounds(::ompl::base::State* state) const
{
  auto sstate = static_cast<const StateType*>(state);
  if (sstate == nullptr || sstate->mState == nullptr)
    throw std::invalid_argument("enforceBounds called with null state");

  if (!sstate->mValid)
    throw std::invalid_argument("enforceBounds called with invalid state");

  auto temporaryState = mStateSpace->createState();
  mBoundsProjection->project(sstate->mState, temporaryState);
  mStateSpace->copyState(temporaryState, sstate->mState);
}

//==============================================================================
bool GeometricStateSpace::satisfiesBounds(
    const ::ompl::base::State* state) const
{
  auto sstate = static_cast<const StateType*>(state);
  if (sstate == nullptr || sstate->mState == nullptr)
    return false;
  if (!sstate->mValid)
    return false;

  return mBoundsConstraint->isSatisfied(sstate->mState);
}

//==============================================================================
void GeometricStateSpace::copyState(
    ::ompl::base::State* destination, const ::ompl::base::State* source) const
{
  auto dst = static_cast<StateType*>(destination);
  if (dst == nullptr || dst->mState == nullptr)
    throw std::invalid_argument("copyState called with null destination");
  auto sst = static_cast<const StateType*>(source);
  if (sst == nullptr || sst->mState == nullptr)
    throw std::invalid_argument("copyState called with null source state");
  mStateSpace->copyState(sst->mState, dst->mState);
  dst->mValid = sst->mValid;
}

//==============================================================================
double GeometricStateSpace::distance(
    const ::ompl::base::State* state1, const ::ompl::base::State* state2) const
{
  auto sstate1 = static_cast<const StateType*>(state1);
  auto sstate2 = static_cast<const StateType*>(state2);

  if (sstate1 == nullptr || sstate1->mState == nullptr)
    throw std::invalid_argument("distance called with null state1");
  if (sstate2 == nullptr || sstate2->mState == nullptr)
    throw std::invalid_argument("distance called with null state2");
  if (!sstate1->mValid)
    throw std::invalid_argument("distance called with invalid state1");
  if (!sstate2->mValid)
    throw std::invalid_argument("distance called with invaid state2");

  return mDistance->distance(sstate1->mState, sstate2->mState);
}

//==============================================================================
bool GeometricStateSpace::equalStates(
    const ::ompl::base::State* state1, const ::ompl::base::State* state2) const
{
  auto sstate1 = static_cast<const StateType*>(state1);
  auto sstate2 = static_cast<const StateType*>(state2);
  if (sstate1->mValid != sstate2->mValid)
    return false;

  double dist = distance(sstate1, sstate2);
  return dist < EQUALITY_EPSILON;
}

//==============================================================================
void GeometricStateSpace::interpolate(
    const ::ompl::base::State* from,
    const ::ompl::base::State* to,
    double t,
    ::ompl::base::State* state) const
{
  auto sfrom = static_cast<const StateType*>(from);
  if (sfrom == nullptr || sfrom->mState == nullptr)
    throw std::invalid_argument("interpolate called with null from state");
  auto sto = static_cast<const StateType*>(to);
  if (sto == nullptr || sto->mState == nullptr)
    throw std::invalid_argument("interpolate called with null to state");
  auto sstate = static_cast<StateType*>(state);
  if (sstate == nullptr || sstate->mState == nullptr)
    throw std::invalid_argument("interpolate called with null out state");
  if (!sfrom->mValid)
    throw std::invalid_argument("interpolate called with invalid from state");
  if (!sto->mValid)
    throw std::invalid_argument("interpolate called with invalid to state");

  mInterpolator->interpolate(sfrom->mState, sto->mState, t, sstate->mState);
}

//==============================================================================
::ompl::base::StateSamplerPtr GeometricStateSpace::allocDefaultStateSampler()
    const
{
  auto generator = mSampler->createSampleGenerator();
  auto stateSampler
      = ompl_make_shared<StateSampler>(this, std::move(generator));

  return stateSampler;
}

//==============================================================================
::ompl::base::State* GeometricStateSpace::allocState() const
{
  auto ast = mStateSpace->allocateState();
  return new StateType(ast);
}

//==============================================================================
::ompl::base::State* GeometricStateSpace::allocState(
    const aikido::statespace::StateSpace::State* state) const
{
  auto newState = mStateSpace->allocateState();
  mStateSpace->copyState(state, newState);
  return new StateType(newState);
}

//==============================================================================
void GeometricStateSpace::freeState(::ompl::base::State* state) const
{
  if (state != nullptr)
  {
    auto sstate = static_cast<StateType*>(state);
    if (sstate->mState != nullptr)
    {
      mStateSpace->freeState(sstate->mState);
    }
    delete sstate;
  }
}

//==============================================================================
statespace::ConstStateSpacePtr GeometricStateSpace::getAikidoStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
statespace::ConstInterpolatorPtr GeometricStateSpace::getInterpolator() const
{
  return mInterpolator;
}

//==============================================================================
constraint::ConstTestablePtr GeometricStateSpace::getBoundsConstraint() const
{
  return mBoundsConstraint;
}

//==============================================================================
double GeometricStateSpace::getMaxDistanceBetweenValidityChecks() const
{
  return mMaxDistanceBetweenValidityChecks;
}

} // ompl
} // planner
} // aikido
