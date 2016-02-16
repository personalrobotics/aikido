#ifndef AIKIDO_TSR_TSR_H_
#define AIKIDO_TSR_TSR_H_

#include "SampleableRegion.hpp"
#include <Eigen/Dense>
#include <random>
#include <memory>

namespace aikido {
namespace sampleable{

class TSR : public SampleableRegion<Eigen::Isometry3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TSR(const Eigen::Isometry3d& T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& Bw = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& Tw_e = Eigen::Isometry3d::Identity());

  TSR(const TSR&) = default;
  TSR(TSR&& other) = default;
  TSR& operator=(const TSR& other) = default;
  TSR& operator=(TSR&& other) = default;
  virtual ~TSR() = default;

  /// Return a transform sampled from this TSR.
  ///
  /// This function uses the provided RNG to create a sample `Tw_s` from the
  /// `Bw` bounds matrix of this TSR, and returns the result:
  /// `T0_w * Tw_s * Tw_e`.
  ///
  /// \param[in] rng Random number generator from which to sample
  /// \return a transform within the bounds of this TSR.
  const Eigen::Isometry3d sample(aikido::util::RNG& rng) override;

  bool isSatisfied(const Eigen::Isometry3d T0_s) const override;
  bool canSample() const override;
  int maxSampleCount() const override;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:

  /// True if TSR contains single point
  bool singlePointTSR() const;

  /// infinite if bounds have volume. 
  int mMaxSampleCount;
};

} // namespace sampleableregion
} // namespace aikido

#endif // AIKIDO_TSR_TSR_H_
