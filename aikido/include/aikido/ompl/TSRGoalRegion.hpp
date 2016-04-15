#ifndef AIKIDO_TSR_GOAL_REGION_H_
#define AIKIDO_TSR_GOAL_REGION_H_

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "../constraint/TSR.hpp"

namespace aikido
{
namespace ompl
{
/// Exposes a TSR as a goal to OMPL planners
class TSRGoalRegion : public ::ompl::base::GoalSampleableRegion
{
public:
  /// Constructor. Generate a goal region from a TSR
  TSRGoalRegion(const ::ompl::base::SpaceInformationPtr& _si,
                std::unique_ptr<constraint::TSR> _tsr);

  /// Sample a state in the goal region
  virtual void sampleGoal(::ompl::base::State* _state) const;

  /// Return the maximum number of samples that can
  ///  be asked for before repeating
  virtual unsigned int maxSampleCount() const;

  /// Return true of maxSampleCount() > 0
  virtual bool canSample() const;

  /// Compute the distance to the goal (heuristic)
  virtual double distanceGoal(const ::ompl::base::State* _state) const;

  /// Return true if the state satisfies the goal constraints
  virtual bool isSatisfied(const ::ompl::base::State* _state) const;

private:
  std::unique_ptr<constraint::TSR> mTSR;
  std::unique_ptr<constraint::SampleGenerator> mSampleGenerator;
};

}
}
#endif
