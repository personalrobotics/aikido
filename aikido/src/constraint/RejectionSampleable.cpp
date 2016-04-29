#include <aikido/constraint/RejectionSampleable.hpp>
#include <dart/common/StlHelpers.h>

using dart::common::make_unique;

namespace aikido {
namespace constraint {

// For internal use only.
class RejectionSampler : public SampleGenerator
{
public:

  RejectionSampler(
    statespace::StateSpacePtr _stateSpace,
    std::unique_ptr<SampleGenerator> _sampler,
    TestablePtr _testable,
    int _maxTrialPerSample);

  RejectionSampler(const RejectionSampler&) = delete;
  RejectionSampler(RejectionSampler&& other) = delete;

  RejectionSampler& operator=(const RejectionSampler& other) = delete;
  RejectionSampler& operator=(RejectionSampler&& other) = delete;

  virtual ~RejectionSampler() = default; 

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  bool canSample() const override;

  // Documentation inherited.
  int getNumSamples() const override;

private:  
  statespace::StateSpacePtr mStateSpace;
  std::unique_ptr<SampleGenerator> mSampler;
  TestablePtr mTestable;
  int mMaxTrialPerSample;

  friend class RejectionSampleable;
};

//=============================================================================
RejectionSampleable::RejectionSampleable(statespace::StateSpacePtr _stateSpace,
  SampleablePtr _sampleable, TestablePtr _testable, int _maxTrialPerSample)
: mStateSpace(std::move(_stateSpace))
, mSampleable(std::move(_sampleable))
, mTestable(std::move(_testable))
, mMaxTrialPerSample(_maxTrialPerSample)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (!mSampleable)
    throw std::invalid_argument("Sampleable is null.");

  if (!mTestable)
    throw std::invalid_argument("Testable is null.");

  if (mStateSpace != mSampleable->getStateSpace())
    throw std::invalid_argument("Sampleable's statespace "
      "does not match StateSpace.");

  if (mStateSpace != mTestable->getStateSpace())
    throw std::invalid_argument("Testable's statespace "
      "does not match StateSpace.");

  if (mMaxTrialPerSample <= 0)
    throw std::invalid_argument("MaxNumTrialsPerSample is not positive.");
}

//=============================================================================
statespace::StateSpacePtr RejectionSampleable::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
std::unique_ptr<SampleGenerator> RejectionSampleable::createSampleGenerator() const
{
  auto sampler = mSampleable->createSampleGenerator();
  return make_unique<RejectionSampler>(mStateSpace, std::move(sampler), 
    mTestable, mMaxTrialPerSample);
}

//=============================================================================
RejectionSampler::RejectionSampler(
  statespace::StateSpacePtr _stateSpace,
  std::unique_ptr<SampleGenerator> _sampler,
  TestablePtr _testable,
  int _maxTrialPerSample)
: mStateSpace(std::move(_stateSpace))
, mSampler(std::move(_sampler))
, mTestable(std::move(_testable))
, mMaxTrialPerSample(_maxTrialPerSample)
{
  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");

  if (!mSampler)
    throw std::invalid_argument("SampleGenerator is null.");

  if (!mTestable)
    throw std::invalid_argument("Testable is null.");

  if (mMaxTrialPerSample <= 0)
    throw std::invalid_argument("MaxNumTrialsPerSample is not positive.");
}

//=============================================================================
statespace::StateSpacePtr RejectionSampler::getStateSpace() const
{
  return mStateSpace;
}

//=============================================================================
bool RejectionSampler::sample(statespace::StateSpace::State* _state)
{
  if (!mSampler->canSample())
    return false;

  for (int i = 0; i < mMaxTrialPerSample; ++i)
  {
    bool success = mSampler->sample(_state);
    if (!success)
      continue;

    bool satisfied = mTestable->isSatisfied(_state);

    if (satisfied)
      return true;
  }

  return false;
}

//=============================================================================
bool RejectionSampler::canSample() const
{
  return mSampler->canSample();
}

//=============================================================================
int RejectionSampler::getNumSamples() const
{
  return mSampler->getNumSamples();
}

}
}
