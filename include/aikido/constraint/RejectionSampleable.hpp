#ifndef AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP
#define AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP

#include "Sampleable.hpp"
#include "Testable.hpp"
#include "../statespace/StateSpace.hpp"



namespace aikido {
namespace constraint {

/// Rejection-based sampleable.
/// Takes a sampleable and a testable.
/// SampleGenerators generate samples from the sampleable
/// and return samples that pass the testable.
class RejectionSampleable : public Sampleable
{
public:

  /// Constructor.
  /// \param _stateSpace StateSpace in which both 
  ///        sampleable and testable operate.
  /// \prarm _sampleable Sampleable for robot configuration.
  /// \param _testable Testable for each configuration.
  /// \param _maxTrialPerSample Max number of trials to generate each sample.
  ///        If all _maxTrialPerSample fails to pass _testable,
  ///        SampleGenerator.sample(...) will return false.
  RejectionSampleable(
    statespace::StateSpacePtr _stateSpace,
    SampleablePtr _sampleable,
    TestablePtr _testable,
    int _maxTrialPerSample);


  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::StateSpacePtr mStateSpace;
  SampleablePtr mSampleable;
  TestablePtr mTestable;
  int mMaxTrialPerSample;
};

} // namespace constraint
} // namespace aikido

#endif 
