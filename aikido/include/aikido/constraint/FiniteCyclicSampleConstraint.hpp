#ifndef AIKIDO_CONSTRAINT_FINITECYCLICSAMPLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_FINITECYCLICSAMPLECONSTRAINT_H

#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

// Constraint that turns a finite sampleable constraint into
// a cyclic sampleable constriant.
// It's generator  will generate samples in the same order as 
// the original sampleable constraint, but once the samples are exhausted,
// it will cycle through the samples, starting from the initial sample. 
// The original sampleable should be finite. 
class FiniteCyclicSampleConstraint : public SampleableConstraint
{
public:
  explicit FiniteCyclicSampleConstraint(
    SampleableConstraintPtr _sampleable);

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  SampleableConstraintPtr mSampleable;

};


}
}

#endif
