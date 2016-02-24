#ifndef AIKIDO_TSR_SAMPLEGENERATOR_H_
#define AIKIDO_TSR_SAMPLEGENERATOR_H_

#include "TSR.hpp"
#include <Eigen/Dense>
#include <random>
#include <memory>

namespace aikido {
namespace sampleable{

class TSRSampleGenerator : public SampleGenerator<Eigen::Isometry3d>
{
public:

  TSRSampleGenerator(
      const Eigen::Isometry3d& T0_w,
      const Eigen::Matrix<double, 6, 2>& Bw,
      const Eigen::Isometry3d& Tw_e,
      std::unique_ptr<RNG> rng)
  : mT0_w(T0_w)
  , mTw_e(Tw_e)
  , mBw(Bw)
  , mRng(std::move(rng))
  {
  };

  TSRSampleGenerator(const TSRSampleGenerator&) = default;
  TSRSampleGenerator(TSRSampleGenerator&& other) = default;
  TSRSampleGenerator& operator=(const TSRSampleGenerator& other) = default;
  TSRSampleGenerator& operator=(TSRSampleGenerator&& other) = default;
  virtual ~TSRSampleGenerator() = default; 

  /// Return a transform sampled from this TSR.
  ///
  /// This function uses the provided RNG to create a sample `Tw_s` from the
  /// `Bw` bounds matrix of this TSR, and returns the result:
  /// `T0_w * Tw_s * Tw_e`.
  ///
  /// \param[in] rng Random number generator from which to sample
  /// \return a transform within the bounds of this TSR.
  optional<Eigen::Isometry3d> sample() override;

  bool canSample() override;
  int numSamples() override;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<RNG> mRng;

};

} // namespace sampleableregion
} // namespace aikido

#endif // AIKIDO_TSR_SAMPLEGENERATOR_H_

