#ifndef AIKIDO_TSR_TSR_H_
#define AIKIDO_TSR_TSR_H_

#include "Sampleable.hpp"
#include <Eigen/Dense>

namespace aikido {
namespace sampleable{

class TSR : public SampleableConstraint<Eigen::Isometry3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TSR(std::unique_ptr<util::RNG> _rng,
      const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const TSR& other);
  TSR(TSR&& other);
  TSR& operator=(const TSR& other);
  TSR& operator=(TSR&& other);
  virtual ~TSR() = default;

  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>> sampler() const override;

  /// Checks if current state is valid. Throws invalid argument exception if not.
  void validate();
  
  /// Wraps mBw's angles.
  void wrap();

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<util::RNG> mRng;

};


class TSRSampleGenerator : public SampleGenerator<Eigen::Isometry3d>
{
public:

  TSRSampleGenerator(std::unique_ptr<util::RNG> _rng,
                     const Eigen::Isometry3d& _T0_w,
                     const Eigen::Matrix<double, 6, 2>& _Bw,
                     const Eigen::Isometry3d& _Tw_e)
  : mRng(std::move(_rng))
  , mT0_w(_T0_w)
  , mBw(_Bw)
  , mTw_e(_Tw_e)
  {
  };

  TSRSampleGenerator(const TSRSampleGenerator&) = delete;
  TSRSampleGenerator(TSRSampleGenerator&& other) = delete;
  TSRSampleGenerator& operator=(const TSRSampleGenerator& other) = delete;
  TSRSampleGenerator& operator=(TSRSampleGenerator&& other) = delete;
  virtual ~TSRSampleGenerator() = default; 

  /// Return a transform sampled from this TSR.
  ///
  /// This function uses the provided RNG to create a sample `Tw_s` from the
  /// `Bw` bounds matrix of this TSR, and returns the result:
  /// `T0_w * Tw_s * Tw_e`.
  ///
  /// \param[in] rng Random number generator from which to sample
  /// \return a transform within the bounds of this TSR.
  boost::optional<Eigen::Isometry3d> sample() override;

  bool canSample() const override;
  int getNumSamples() const override;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

private:
  std::unique_ptr<util::RNG> mRng;

};

using TSRPtr = std::shared_ptr<const TSR>;
using TSRUniquePtr = std::unique_ptr<TSR>;
using TSRSampleGeneratorPtr = std::shared_ptr<const TSRSampleGenerator>;
using TSRSampleGeneratorUniquePtr = std::unique_ptr<TSRSampleGenerator>;
using TSRSamplerUniquePtr = std::unique_ptr<SampleGenerator<Eigen::Isometry3d>>;

} // sampleable
} // aikido

#endif // AIKIDO_TSR_TSR_H_
