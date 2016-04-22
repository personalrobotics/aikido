#ifndef AIKIDO_OMPL_GOALREGION_HPP_
#define AIKIDO_OMPL_GOALREGION_HPP_

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
  /// Constructor. Generate a goal region from a Testable constraint
  ///  and a SampleGenerator.
  GoalRegion(const ::ompl::base::SpaceInformationPtr& _si,
             constraint::TestableConstraintPtr _goalTestable,
             std::unique_ptr<constraint::SampleGenerator> _generator);

  /// Sample a state in the goal region using the SampleGenerator defined on the class
  void sampleGoal(::ompl::base::State* _state) const override;

  /// Return the maximum number of samples that can
  ///  be asked for before repeating
  unsigned int maxSampleCount() const override;

  /// Return true of maxSampleCount() > 0
  bool couldSample() const override;

  /// Return 0 if the goal is satisfied or inf otherwise
  double distanceGoal(const ::ompl::base::State* _state) const override;

  /// Return true if the state satisfies the goal constraints
  bool isSatisfied(const ::ompl::base::State* _state) const override;

private:
  constraint::TestableConstraintPtr mTestable;
  std::unique_ptr<constraint::SampleGenerator> mSampleGenerator;
};
}
}
#endif
