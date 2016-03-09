#ifndef AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H
#define AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H

#include "IkSampleableConstraint.hpp"
#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

// For internal use only.
class IKSampleGenerator : public SampleGenerator<Eigen::VectorXd>
{
public:
  IKSampleGenerator(const IKSampleGenerator&) = delete;
  IKSampleGenerator(IKSampleGenerator&& other) = delete;

  IKSampleGenerator& operator=(const IKSampleGenerator& other) = delete;
  IKSampleGenerator& operator=(IKSampleGenerator&& other) = delete;

  virtual ~IKSampleGenerator() = default; 

  // Documentation inherited.
  boost::optional<Eigen::VectorXd> sample() override;

  // Documentation inherited.
  bool canSample() const override;

  // Documentation inherited.
  int getNumSamples() const override;

private:  
  // For internal use only.
  IKSampleGenerator(
    std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
    const dart::dynamics::InverseKinematicsPtr& _ikPtr,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  dart::dynamics::InverseKinematicsPtr mIKPtr;
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> mIsometrySampler;
  int mMaxNumTrials;
  std::unique_ptr<util::RNG> mRng;

  friend class IKSampleableConstraint;
};

} // namespace constraint 
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_IKSAMPLEGENERATOR_H
