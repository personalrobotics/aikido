#include <exception>
#include <Eigen/Dense>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include "aikido/planner/kinodynamics/dimt/Params.h"
#include "aikido/planner/kinodynamics/ompl/MyOptimizationObjective.hpp"

using Eigen::VectorXd;

//
// Return the cost of the state at this point
//
// @param s State to get the cost for
// @return Cost of the state
//
ompl::base::Cost ompl::base::MyOptimizationObjective::stateCost(const ompl::base::State *s) const
{
    if (sampler_ == nullptr or opt_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return opt_->stateCost(s);
}

//
// Return the cost of moving from s1 to s2
//
// @param s1 Start state
// @param s2 Goal state
// @return Cost of going from s1 to s2
//
::ompl::base::Cost ompl::base::MyOptimizationObjective::motionCost(const ompl::base::State *s1,
                                                                 const ompl::base::State *s2) const
{
    if (sampler_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return opt_->motionCost(s1, s2);
}

//
// Function to get the informed sampler pointer
//
// @param probDefn Problem definition pointer (OMPL)
// @param maxNumberCalls Maximum number of sampling calls
// @return Infromed sampler
//
::ompl::base::InformedSamplerPtr ompl::base::MyOptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls) const
{
    if (sampler_ == nullptr)
    {
        throw std::runtime_error("An informed sampler with optimization objective "
                                 "must be provided or set.");
    }
    return sampler_;
    //return ompl::base::OptimizationObjective::allocInformedStateSampler(probDefn, maxNumberCalls);
}


namespace ompl
{
    namespace base
    {
        Cost GeometricObjective::stateCost(const State *s) const
        {
            Eigen::VectorXd stateVec(param.dimensions);
            get_eigen_vector(s, stateVec);
            return Cost((startVec_ - stateVec).norm() + (goalVec_ - stateVec).norm());
        }

        Cost GeometricObjective::motionCost(const State *s1, const State *s2) const
        {
            Eigen::VectorXd s1Vec, s2Vec;
            get_eigen_vector(s1, s1Vec);
            get_eigen_vector(s2, s2Vec);
            return Cost((s1Vec - s2Vec).norm());
        }

        Cost GeometricObjective::combineCosts(Cost c1, Cost c2) const
        {
            return Cost(c1.value() + c2.value());
        }

        Cost DimtObjective::stateCost(const State *s) const
        {
            return Cost(dimt_->getMinTime(startState_, s) +
                        dimt_->getMinTime(s, goalState_));
        }

        Cost DimtObjective::motionCost(const State *s1, const State *s2) const
        {
            return Cost(dimt_->getMinTime(s1, s2));
        }

        Cost DimtObjective::getCostIfSmallerThan(const State* s1, const State *s2, Cost thresholdCost) const
        {
            return Cost(dimt_->getMinTimeIfSmallerThan(s1, s2, thresholdCost.value()));
        }
    }  // namespace base
}  // namespace ompl
