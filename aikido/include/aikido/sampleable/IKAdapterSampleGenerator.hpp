#ifndef AIKIDO_IKADAPTER_SAMPLEGENERATOR_H_
#define AIKIDO_IKADAPTER_SAMPLEGENERATOR_H_

#include "IKAdapterSampleableConstraint.hpp"

namespace aikido {
namespace sampleable{

using SkeletonPtr = std::shared_ptr<::dart::dynamics::Skeleton>;

class IKAdapterSampleGenerator : public SampleGenerator<Eigen::VectorXd>
{
public:

  typedef std::shared_ptr<IKAdapterSampleGenerator> Ptr;
  typedef std::shared_ptr<IKAdapterSampleGenerator const> ConstPtr;
  typedef std::unique_ptr<IKAdapterSampleGenerator> UniquePtr;
  typedef std::unique_ptr<IKAdapterSampleGenerator const> UniqueConstPtr;

  IKAdapterSampleGenerator(
    Isometry3dConstraintPtr isometry3dConstraint,
    InverseKinematicsPtr ikPtr,
    std::unique_ptr<RNG> rng)
  : mIsometry3dConstraintPtr(isometry3dConstraint)
  , mIKPtr(ikPtr)
  , mRng(std::move(rng))
  {
  };

  IKAdapterSampleGenerator(const IKAdapterSampleGenerator&) = default;
  IKAdapterSampleGenerator(IKAdapterSampleGenerator&& other) = default;
  IKAdapterSampleGenerator& operator=(const IKAdapterSampleGenerator& other) = default;
  IKAdapterSampleGenerator& operator=(IKAdapterSampleGenerator&& other) = default;
  virtual ~IKAdapterSampleGenerator() = default; 

  optional<Eigen::VectorXd> sample() override;
  bool canSample() override;
  int numSamples() override;

  Isometry3dConstraintPtr mIsometry3dConstraintPtr;
  InverseKinematicsPtr mIKPtr;

private:  
  std::unique_ptr<RNG> mRng;

};


} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_IKADAPTER_SAMPLEGENERATOR_H_

