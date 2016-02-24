#include <aikido/sampleable/IKAdapterSampleableConstraint.hpp>
#include <aikido/sampleable/IKAdapterSampleGenerator.hpp>

namespace aikido {
namespace sampleable{

//=============================================================================
std::unique_ptr<SampleGenerator<Eigen::VectorXd>> IKAdapterSampleableConstraint::sampler() const
{
  return IKAdapterSampleGenerator::UniquePtr(new IKAdapterSampleGenerator(
	  											  mIsometry3dConstraintPtr,
	  											  mIKPtr,
	  											  mRng->clone(mRng.get())));
}

//=============================================================================
bool IKAdapterSampleableConstraint::isSatisfied(const Eigen::VectorXd state) const
{
  mIKPtr->setPositions(state);

  // Get end-effector transform w.r.t. world origin
  Eigen::Isometry3d isometry = mIKPtr->getNode()->getWorldTransform();

  return mIsometry3dConstraintPtr->isSatisfied(isometry);
}

} // namespace sampleable
} // namespace aikido
