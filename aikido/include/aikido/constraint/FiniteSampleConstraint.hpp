#ifndef AIKIDO_CONSTRAINT_FINITESAMPLECONSTRAINT_H
#define AIKIDO_CONSTRAINT_FINITESAMPLECONSTRAINT_H

#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

// Constraint that always returns a finite set of samples.
// Its SampleGenerator will generate sample in the order of _states
// until all samples are exhausted.
class FiniteSampleConstraint : public SampleableConstraint
{
public:
  FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    statespace::StateSpace::State* _state);

  FiniteSampleConstraint(
    statespace::StateSpacePtr _stateSpace,
    std::vector<statespace::StateSpace::State*> _states);

  virtual ~FiniteSampleConstraint();

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

  std::unique_ptr<SampleGenerator> createCyclicSampleGenerator() const;

private:
  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;

};


// For internal use only.
class FiniteSampleGenerator : public SampleGenerator
{
public:

  FiniteSampleGenerator(const FiniteSampleGenerator&) = delete;
  FiniteSampleGenerator(FiniteSampleGenerator&& other) = delete;

  FiniteSampleGenerator& operator=(
    const FiniteSampleGenerator& other) = delete;
  FiniteSampleGenerator& operator=(
    FiniteSampleGenerator&& other) = delete;

  virtual ~FiniteSampleGenerator() = default; 

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  /// Documentation inherited.
  int getNumSamples() const override;

  /// Documentation inherited.
  bool canSample() const override;

private:

  // For internal use only.
  FiniteSampleGenerator(
    statespace::StateSpacePtr _stateSpace,
    std::vector<statespace::StateSpace::State*> _states);

  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  int mIndex;

  friend class FiniteSampleConstraint;
};


// For internal use only.
class FiniteCyclicSampleGenerator : public SampleGenerator
{
public:

  FiniteCyclicSampleGenerator(const FiniteCyclicSampleGenerator&) = delete;
  FiniteCyclicSampleGenerator(FiniteCyclicSampleGenerator&& other) = delete;

  FiniteCyclicSampleGenerator& operator=(
    const FiniteCyclicSampleGenerator& other) = delete;
  FiniteCyclicSampleGenerator& operator=(
    FiniteCyclicSampleGenerator&& other) = delete;

  virtual ~FiniteCyclicSampleGenerator() = default; 

  /// Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  /// Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  /// Documentation inherited.
  int getNumSamples() const override;

  /// Documentation inherited.
  bool canSample() const override;

private:

  // For internal use only.
  FiniteCyclicSampleGenerator(
    statespace::StateSpacePtr _stateSpace,
    std::vector<statespace::StateSpace::State*> _states);

  statespace::StateSpacePtr mStateSpace;
  std::vector<statespace::StateSpace::State*> mStates;
  int mIndex;

  friend class FiniteSampleConstraint;
};

}
}

#endif
