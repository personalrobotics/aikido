#ifndef AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_

#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/StateSpace.hpp"

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
  /// \param _sampleable Sampleable for robot configuration.
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
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::ConstStateSpacePtr mStateSpace;
  SampleablePtr mSampleable;
  TestablePtr mTestable;
  int mMaxTrialPerSample;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_
