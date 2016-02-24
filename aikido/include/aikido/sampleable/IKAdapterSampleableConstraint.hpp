#ifndef AIKIDO_IKADAPTER_SAMPLEABLECONSTRAINT_H_
#define AIKIDO_IKADAPTER_SAMPLEABLECONSTRAINT_H_

#include "SampleableConstraint.hpp"

#include <Eigen/Dense>
#include <random>
#include <dart/dynamics/dynamics.h>
#include <memory>

namespace aikido {
namespace sampleable{

using ::dart::dynamics::InverseKinematicsPtr;
using Isometry3dConstraintPtr = std::shared_ptr<SampleableConstraint<Eigen::Isometry3d>>;

// transforms Isometry3d region to IK region
class IKAdapterSampleableConstraint : public SampleableConstraint<Eigen::VectorXd>
{
public:
  typedef std::shared_ptr<IKAdapterSampleableConstraint> Ptr;
  typedef std::shared_ptr<IKAdapterSampleableConstraint const> ConstPtr;

  IKAdapterSampleableConstraint(
    Isometry3dConstraintPtr isometry3dConstraint,
    InverseKinematicsPtr ikPtr,
    std::unique_ptr<RNG> rng)
  : mIsometry3dConstraintPtr(isometry3dConstraint)
  , mIKPtr(ikPtr)
  , mRng(std::move(rng))
  {
  };

  IKAdapterSampleableConstraint(const IKAdapterSampleableConstraint&) = default;
  IKAdapterSampleableConstraint(IKAdapterSampleableConstraint&& other) = default;
  IKAdapterSampleableConstraint& operator=(const IKAdapterSampleableConstraint& other) = default;
  IKAdapterSampleableConstraint& operator=(IKAdapterSampleableConstraint&& other) = default;
  virtual ~IKAdapterSampleableConstraint() = default;

  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> sampler() const override;
  bool isSatisfied(const Eigen::VectorXd state) const override;
  
  Isometry3dConstraintPtr mIsometry3dConstraintPtr;
  InverseKinematicsPtr mIKPtr;

private:
  std::unique_ptr<RNG> mRng;

};

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_IKADAPTER_SAMPLEABLECONSTRAINT_H_
