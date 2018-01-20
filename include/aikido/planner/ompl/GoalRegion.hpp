#ifndef AIKIDO_OMPL_GOALREGION_HPP_
#define AIKIDO_OMPL_GOALREGION_HPP_

#include <ompl/base/goals/GoalSampleableRegion.h>
#include "aikido/constraint/smart_pointer.hpp"
#include "../../constraint/Sampleable.hpp"
#include "../../constraint/Testable.hpp"

namespace aikido {
namespace planner {
namespace ompl {

/// Exposes a Testable/Sampleable constraint pair as a goal to OMPL planners
class GoalRegion : public ::ompl::base::GoalSampleableRegion
{
public:
  /// Constructor. Generate a goal region from a Testable constraint
  ///  and a SampleGenerator.
  /// \param _si The SpaceInformation defining the planning instance where this
  /// goal is used
  /// \param _goalTestable A Testable that is satisfied for all states
  /// in this GoalRegion
  /// \param _generator A SampleGenerator that returns states in this GoalRegion
  GoalRegion(
      const ::ompl::base::SpaceInformationPtr& _si,
      constraint::TestablePtr _goalTestable,
      std::unique_ptr<constraint::SampleGenerator> _generator);

  /// Sample a state in the goal region using the SampleGenerator defined on the
  /// class
  /// \param[out] _state The sampled state
  void sampleGoal(::ompl::base::State* _state) const override;

  /// Return the maximum number of samples that can
  ///  be asked for before repeating
  unsigned int maxSampleCount() const override;

  /// Return true if the SampleGenerator can generate more samples
  bool couldSample() const override;

  /// Return 0 if the goal is satisfied or inf otherwise
  /// \param _state The state to evaluate
  double distanceGoal(const ::ompl::base::State* _state) const override;

  /// Return true if the state satisfies the goal constraints
  /// \param _state The state to evaluate
  bool isSatisfied(const ::ompl::base::State* _state) const override;

private:
  constraint::TestablePtr mTestable;
  std::unique_ptr<constraint::SampleGenerator> mSampleGenerator;
};

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif
