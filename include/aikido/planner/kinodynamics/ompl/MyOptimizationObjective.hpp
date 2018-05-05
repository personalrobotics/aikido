#ifndef AIKIDO_PLANNER_KINODYNAMICS_OMPL_MYOPTIMIZATIONOBJECTIVE_HPP_
#define AIKIDO_PLANNER_KINODYNAMICS_OMPL_MYOPTIMIZATIONOBJECTIVE_HPP_

#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <aikido/planner/kinodynamics/dimt/DoubleIntegratorMinimumTime.h>
#include <aikido/planner/kinodynamics/dimt/Params.h>
#include <aikido/planner/kinodynamics/ompl/OmplHelpers.hpp>
#include <aikido/planner/kinodynamics/sampler/Sampler.hpp>

using MotionCostFxn = std::function<double(Eigen::VectorXd, Eigen::VectorXd)>;
using StateCostFxn = std::function<double(Eigen::VectorXd)>;

///
/// Class that inherits from InformedSampler
///
namespace ompl {
namespace base {
class MyOptimizationObjective : public OptimizationObjective
{
private:
  /// Pointer to informed sampler
  MyInformedSamplerPtr sampler_ = nullptr;

  /// Base optimization objective that sets the cost function for
  /// both the informed sampler and anything using this optimization
  /// objective
  OptimizationObjectivePtr opt_ = nullptr;

protected:
  const ompl::base::State* startState_;
  const ompl::base::State* goalState_;

public:
  ///
  /// Constructor
  ///
  /// @param si Space Information
  /// @param sampler Informed Sampler
  /// @param sampleBatchSize How many samples to get each time a new
  /// batch of samples is gotten
  /// @param stateCostFn Cost function for a single point in space
  /// @param motionCostFn Cost function between two states
  ///
  MyOptimizationObjective(
      const SpaceInformationPtr& si,
      const MyInformedSamplerPtr sampler,
      const ompl::base::State* startState,
      const ompl::base::State* goalState)
    : OptimizationObjective(si)
    , sampler_(sampler)
    , opt_(sampler_->problem()->getOptimizationObjective())
    , startState_(startState)
    , goalState_(goalState)
  {
  }

  ///
  /// Return the cost of the state at this point
  ///
  /// @param s State to get the cost for
  /// @return Cost of the state
  ///
  virtual Cost stateCost(const State* s) const override;

  ///
  /// Return the cost of moving from s1 to s2
  ///
  /// @param s1 Start state
  /// @param s2 Goal state
  /// @return Cost of going from s1 to s2
  ///
  virtual Cost motionCost(const State* s1, const State* s2) const override;

  ///
  /// Function to get the informed sampler pointer
  ///
  /// @param probDefn Problem definition pointer (OMPL)
  /// @param maxNumberCalls Maximum number of sampling calls
  /// @return Infromed sampler
  ///
  virtual InformedSamplerPtr allocInformedStateSampler(
      const ProblemDefinitionPtr probDefn, unsigned int maxNumberCalls) const;

  ///
  /// Function to provide and informed sampler
  ///
  /// @param sampler
  ///
  virtual void setInformedStateSampler(const MyInformedSamplerPtr& sampler)
  {
    sampler_ = sampler;
  }
};

class GeometricObjective : public OptimizationObjective
{
protected:
  const ompl::base::State* startState_;
  const ompl::base::State* goalState_;
  Eigen::VectorXd startVec_;
  Eigen::VectorXd goalVec_;

public:
  ///
  /// Constructor
  ///
  /// @param si Space Information
  /// @param startState Start state of the problem
  /// @param goalState Goal state of the problem
  ///
  GeometricObjective(
      const SpaceInformationPtr& si,
      const ompl::base::State* startState,
      const ompl::base::State* goalState)
    : OptimizationObjective(si)
    , startState_(startState)
    , goalState_(goalState)
    , startVec_(param.dimensions)
    , goalVec_(param.dimensions)
  {
  }

  ///
  /// Return the cost of the state at this point
  ///
  /// @param s State to get the cost for
  /// @return Cost of the state
  ///
  virtual Cost stateCost(const State* s) const override;

  ///
  /// Return the cost of moving from s1 to s2
  ///
  /// @param s1 Start state
  /// @param s2 Goal state
  /// @return Cost of going from s1 to s2
  ///
  virtual Cost motionCost(const State* s1, const State* s2) const override;

  ///
  /// Combines cost
  ///
  /// @param c1 cost one
  /// @param c2 cost two
  /// @return Combined cost (c1 + c2)
  ///
  virtual Cost combineCosts(Cost c1, Cost c2) const override;
};

class DimtObjective : public OptimizationObjective
{
protected:
  const ompl::base::State* startState_;
  const ompl::base::State* goalState_;

private:
  const DIMTPtr dimt_;

public:
  ///
  /// Constructor
  ///
  /// @param si Space Information
  /// @param startState Start state of the problem
  /// @param goalState Goal state of the problem
  /// @param di Double Integrator model
  ///
  DimtObjective(
      const SpaceInformationPtr& si,
      const ompl::base::State* startState,
      const ompl::base::State* goalState,
      const DIMTPtr dimt)
    : OptimizationObjective(si), dimt_(dimt)
  {
    startState_ = startState;
    goalState_ = goalState;
  }

  ///
  /// Return the cost of the state at this point
  ///
  /// @param s State to get the cost for
  /// @return Cost of the state
  ///
  virtual Cost stateCost(const State* s) const override;

  ///
  /// Return the cost of moving from s1 to s2
  ///
  /// @param s1 Start state
  /// @param s2 Goal state
  /// @return Cost of going from s1 to s2
  ///
  virtual Cost motionCost(const State* s1, const State* s2) const override;

  ///
  /// Combines cost
  ///
  /// @param c1 cost one
  /// @param c2 cost two
  /// @return Combined cost (c1 + c2)
  ///
  virtual Cost combineCosts(Cost c1, Cost c2) const override
  {
    return Cost(std::max(c1.value(), c2.value()));
  }

  Cost getCostIfSmallerThan(
      const State* s1, const State* s2, Cost thresholdCost) const;
};
} // base
} // ompl

#endif // AIKIDO_PLANNER_KINODYNAMICS_OMPL_MYOPTIMIZATIONOBJECTIVE_HPP_
