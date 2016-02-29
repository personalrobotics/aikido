#include <aikido/sampleable/IKSampleable.hpp>

namespace aikido {
namespace sampleable{

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::VectorXd>> IKSampleableConstraint::sampler() const
{
  return IKSampleGeneratorUniquePtr(new IKSampleGenerator(mIsometry3dConstraintPtr->sampler(),
                                                          mIKPtr,
                                                          mRng->clone()));
}

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::VectorXd>> IKSampleableConstraint::sampler(int maxNumTrials) const
{
  return IKSampleGeneratorUniquePtr(new IKSampleGenerator(mIsometry3dConstraintPtr->sampler(),
                                                          mIKPtr,
                                                          mRng->clone(),
                                                          maxNumTrials));
}

} // namespace sampleable
} // namespace aikido
