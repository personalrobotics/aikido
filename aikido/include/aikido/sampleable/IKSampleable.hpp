#ifndef AIKIDO_IK_SAMPLEABLE_H
#define AIKIDO_IK_SAMPLEABLE_H

#include "Sampleable.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace sampleable{

// Transforms Isometry3d region to IK region
class IKSampleableConstraint : public SampleableConstraint<Eigen::VectorXd>
{
public:

  IKSampleableConstraint(
    const std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>>
      _isometry3dConstraint,
    const dart::dynamics::InverseKinematicsPtr _ikPtr,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  IKSampleableConstraint(const IKSampleableConstraint& other);
  IKSampleableConstraint(IKSampleableConstraint&& other);
  IKSampleableConstraint& operator=(
    const IKSampleableConstraint& other);
  IKSampleableConstraint& operator=(IKSampleableConstraint&& other);

  virtual ~IKSampleableConstraint() = default;

  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> 
    createSampleGenerator() const override;
  
  /// Set rng seed.
  void setRNG(std::unique_ptr<util::RNG> rng);
  
private:
  std::unique_ptr<util::RNG> mRng;
  std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>> 
    mIsometry3dConstraintPtr;
  dart::dynamics::InverseKinematicsPtr mIKPtr;
  int mMaxNumTrials;

};


class IKSampleGenerator : public SampleGenerator<Eigen::VectorXd>
{
  friend class IKSampleableConstraint;
  
public:

  IKSampleGenerator(const IKSampleGenerator&) = delete;
  IKSampleGenerator(IKSampleGenerator&& other) = delete;
  IKSampleGenerator& operator=(const IKSampleGenerator& other) = delete;
  IKSampleGenerator& operator=(IKSampleGenerator&& other) = delete;
  virtual ~IKSampleGenerator() = default; 

  boost::optional<Eigen::VectorXd> sample() override;
  bool canSample() const override;
  int getNumSamples() const override;

private:  

  IKSampleGenerator(
    std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
    const dart::dynamics::InverseKinematicsPtr _ikPtr,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials);

  dart::dynamics::InverseKinematicsPtr mIKPtr;
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> mIsometrySampler;
  int mMaxNumTrials;
  std::unique_ptr<util::RNG> mRng;

};


using IKSampleGeneratorPtr = std::shared_ptr<const IKSampleGenerator>;
using IKSampleGeneratorUniquePtr = std::unique_ptr<IKSampleGenerator>;
using IKSampleableConstraintPtr = std::shared_ptr<const IKSampleableConstraint>;
using IKSampleableConstraintUniquePtr = std::unique_ptr<IKSampleableConstraint>;

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_IK_SAMPLEABLE_H

