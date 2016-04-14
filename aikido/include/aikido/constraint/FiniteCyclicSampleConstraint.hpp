#ifndef AIKIDO_CONSTRAINT_FINITECYCLICSAMPLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_FINITECYCLICSAMPLECONSTRAINT_H

#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

// Constraint that always returns a finite set of samples.
// Its SampleGenerator will generate sample in the order of _states
// until all samples are exhausted.
class FiniteCyclicSampleConstraint : public SampleableConstraint
{
public:
  FiniteCyclicSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    statespace::StateSpace::State* _state);

  FiniteCyclicSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    std::vector<const statespace::StateSpace::State*> _states);

  virtual ~FiniteCyclicSampleConstraint();

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;

};


}
}

#endif
