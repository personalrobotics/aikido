#ifndef AIKIDO_IKADAPTER_H_
#define AIKIDO_IKADAPTER_H_

#include "SampleableRegion.hpp"
#include "TSR.hpp"

#include <Eigen/Dense>
#include <random>
#include <dart/dynamics/dynamics.h>
#include <memory>

using ::dart::dynamics::InverseKinematicsPtr;

namespace aikido {
namespace sampleable{

using SkeletonPtr = std::shared_ptr<::dart::dynamics::Skeleton>;
using TSRPtr = std::shared_ptr<TSR>;

// transforms TSR sample to IK sample
class IKAdapter : public SampleableRegion<Eigen::VectorXd>
{
public:

  IKAdapter(const TSRPtr& _tsrPtr, const InverseKinematicsPtr& _ikPtr);
  IKAdapter(const IKAdapter&) = default;
  IKAdapter(IKAdapter&& other) = default;
  IKAdapter& operator=(const IKAdapter& other) = default;
  IKAdapter& operator=(IKAdapter&& other) = default;
  virtual ~IKAdapter() = default;

  const Eigen::VectorXd sample(aikido::util::RNG& rng) override;
  bool isSatisfied(const Eigen::VectorXd state) const override;
  bool canSample() const override;
  int maxSampleCount() const override;

  const TSRPtr& mTSRPtr;
  const InverseKinematicsPtr& mIKPtr;



private:
  double _sample(aikido::util::RNG& rng, double lower, double upper) const;
};

} // namespace sampleable
} // namespace aikido

#endif // AIKIDO_IKADAPTER_H_
