#ifndef AIKIDO_TSR_TSR_H_
#define AIKIDO_TSR_TSR_H_

#include "SampleableConstraint.hpp"
#include <Eigen/Dense>

namespace aikido {
namespace sampleable{

using util::RNGWrapper;

/// No point-tsr allowed. 
class TSR : public SampleableConstraint<Eigen::Isometry3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TSR(std::unique_ptr<RNG> rng,
      const Eigen::Isometry3d& T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& Bw = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& Tw_e = Eigen::Isometry3d::Identity());
  TSR(const TSR&) = default;
  TSR(TSR&& other) = default;
  TSR& operator=(const TSR& other) = default;
  TSR& operator=(TSR&& other) = default;
  virtual ~TSR() = default;

  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> sampler() const override;

  // True if T0_s is in this TSR.
  bool isSatisfied(const Eigen::Isometry3d T0_s) const override;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<RNG> mRng;

};

} // aikido
} // sampleable

#endif // AIKIDO_TSR_TSR_H_
