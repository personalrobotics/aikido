#ifndef AIKIDO_IK_SAMPLEABLE_H
#define AIKIDO_IK_SAMPLEABLE_H

#include "Sampleable.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace sampleable{

class IKSampleGenerator : public SampleGenerator<Eigen::VectorXd>
{
public:

  IKSampleGenerator(
    std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
    dart::dynamics::InverseKinematicsPtr _ikPtr,
    std::unique_ptr<util::RNG> _rng,
    int _maxNumTrials)
  : mIsometrySampler(std::move(_isometrySampler))
  , mIKPtr(_ikPtr)
  , mRng(std::move(_rng))
  , mMaxNumTrials(_maxNumTrials)
  {
  };

  IKSampleGenerator(
    std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> _isometrySampler,
    dart::dynamics::InverseKinematicsPtr _ikPtr,
    std::unique_ptr<util::RNG> _rng)
  : mIsometrySampler(std::move(_isometrySampler))
  , mIKPtr(_ikPtr)
  , mRng(std::move(_rng))
  , mMaxNumTrials(5)
  {
  };

  IKSampleGenerator(const IKSampleGenerator&) = delete;
  IKSampleGenerator(IKSampleGenerator&& other) = delete;
  IKSampleGenerator& operator=(const IKSampleGenerator& other) = delete;
  IKSampleGenerator& operator=(IKSampleGenerator&& other) = delete;
  virtual ~IKSampleGenerator() = default; 

  boost::optional<Eigen::VectorXd> sample() override;
  bool canSample() const override;
  int getNumSamples() const override;

  dart::dynamics::InverseKinematicsPtr mIKPtr;
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> mIsometrySampler;
  int mMaxNumTrials;

private:  
  std::unique_ptr<util::RNG> mRng;

};


// transforms Isometry3d region to IK region
class IKSampleableConstraint : public SampleableConstraint<Eigen::VectorXd>
{
public:

  IKSampleableConstraint(
    std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>> _isometry3dConstraint,
    dart::dynamics::InverseKinematicsPtr _ikPtr,
    std::unique_ptr<util::RNG> _rng)
  : mIsometry3dConstraintPtr(_isometry3dConstraint)
  , mIKPtr(_ikPtr)
  , mRng(std::move(_rng))
  {
  };

  IKSampleableConstraint(const IKSampleableConstraint&) = default;
  IKSampleableConstraint(IKSampleableConstraint&& other) = default;
  IKSampleableConstraint& operator=(const IKSampleableConstraint& other) = default;
  IKSampleableConstraint& operator=(IKSampleableConstraint&& other) = default;
  virtual ~IKSampleableConstraint() = default;

  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> sampler() const override;
  /// Create a sampler that tries maxNumTrials times.
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> sampler(int maxNumTrials) const;

  std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>> mIsometry3dConstraintPtr;
  dart::dynamics::InverseKinematicsPtr mIKPtr;

private:
  std::unique_ptr<util::RNG> mRng;

};

using IKSampleGeneratorPtr = std::shared_ptr<const IKSampleGenerator>;
using IKSampleGeneratorUniquePtr = std::unique_ptr<IKSampleGenerator>;
using IKSampleableConstraintPtr = std::shared_ptr<const IKSampleableConstraint>;
using IKSampleableConstraintUniquePtr = std::unique_ptr<IKSampleableConstraint>;

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_IK_SAMPLEABLE_H

