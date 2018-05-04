#ifndef AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_

#include "../statespace/StateSpace.hpp"
#include "Sampleable.hpp"
#include "Testable.hpp"

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
  /// \param stateSpace StateSpace in which both
  ///        sampleable and testable operate.
  /// \param sampleable Sampleable for robot configuration.
  /// \param testable Testable for each configuration.
  /// \param maxTrialPerSample Max number of trials to generate each sample.
  ///        If all _maxTrialPerSample fails to pass _testable,
  ///        SampleGenerator.sample(...) will return false.
  RejectionSampleable(
      statespace::StateSpacePtr stateSpace,
      SampleablePtr sampleable,
      TestablePtr testable,
      int maxTrialPerSample);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

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

#endif // AIKIDO_CONSTRAINT_REJECTIONSAMPLEABLE_HPP_
