#ifndef AIKIDO_TSR_GOAL_REGION_H_
#define AIKIDO_TSR_GOAL_REGION_H_

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "../constraint/TestableConstraint.hpp"
#include "../constraint/Sampleable.hpp"

namespace aikido
{
namespace ompl
{
/// Exposes a Testable/Sampleable constraint pair as a goal to OMPL planners
class GoalRegion : public ::ompl::base::GoalSampleableRegion
{
public:
  /// Constructor. Generate a goal region from a Testable and Sampleable
  /// constraint
  GoalRegion(const ::ompl::base::SpaceInformationPtr& _si,
             const constraint::TestableConstraintPtr& _goalTestable,
             std::unique_ptr<constraint::SampleGenerator> _generator);

  /// Sample a state in the goal region
  virtual void sampleGoal(::ompl::base::State* _state) const;

  /// Return the maximum number of samples that can
  ///  be asked for before repeating
  virtual unsigned int maxSampleCount() const;

  /// Return true of maxSampleCount() > 0
  virtual bool canSample() const;

  /// Return 0 if the goal is satisfied or inf otherwise
  virtual double distanceGoal(const ::ompl::base::State* _state) const;

  /// Return true if the state satisfies the goal constraints
  virtual bool isSatisfied(const ::ompl::base::State* _state) const;

private:
  constraint::TestableConstraintPtr mTestable;
  std::unique_ptr<constraint::SampleGenerator> mSampleGenerator;
};
}
}
#endif
