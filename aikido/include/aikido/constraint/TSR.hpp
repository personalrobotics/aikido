#ifndef AIKIDO_CONSTRAINT_TSR_H_
#define AIKIDO_CONSTRAINT_TSR_H_

#include "Sampleable.hpp"
#include <Eigen/Dense>
#include "Projectable.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint {

class TSR;
class TSRSampleGenerator;
class TSRConstraint;


using TSRPtr = std::shared_ptr<TSR>;
using TSRConstPtr = std::shared_ptr<const TSR>;
using TSRUniquePtr = std::unique_ptr<TSR>;
using TSRSamplerUniquePtr = std::unique_ptr<SampleGenerator<Eigen::Isometry3d>>;


class TSR : public SampleableConstraint<Eigen::Isometry3d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TSR(std::unique_ptr<util::RNG> _rng,
      const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw 
        = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e
        = Eigen::Isometry3d::Identity());

  TSR(const Eigen::Isometry3d& _T0_w = Eigen::Isometry3d::Identity(),
      const Eigen::Matrix<double, 6, 2>& _Bw
       = Eigen::Matrix<double, 6, 2>::Zero(),
      const Eigen::Isometry3d& _Tw_e = Eigen::Isometry3d::Identity());

  TSR(const TSR& other);
  TSR(TSR&& other);

  TSR& operator=(const TSR& other);
  TSR& operator=(TSR&& other);

  virtual ~TSR() = default;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator<Eigen::Isometry3d>>
    createSampleGenerator() const override;

  /// Throws an invalid_argument exception if this TSR is invalid.
  void validate() const;

  /// Set the random number generator used by SampleGenerators for this TSR.
  void setRNG(std::unique_ptr<util::RNG> rng);

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

  // Documentation inherited.
  bool canSample() const override;

  // Documentation inherited.
  int getNumSamples() const override;

private:
  // For internal use only.
  TSRSampleGenerator(std::unique_ptr<util::RNG> _rng,
                     const Eigen::Isometry3d& _T0_w,
                     const Eigen::Matrix<double, 6, 2>& _Bw,
                     const Eigen::Isometry3d& _Tw_e);
  
  std::unique_ptr<util::RNG> mRng;

  /// Transformation from origin frame into "wiggle" frame.
  Eigen::Isometry3d mT0_w;

  /// Bounds on "wiggling" in `x, y, z, roll, pitch, yaw`.
  Eigen::Matrix<double, 6, 2> mBw;

  /// Transformation from "wiggle" frame into end frame.
  Eigen::Isometry3d mTw_e;

  friend class TSR;
};


class TSRConstraint : public Differentiable, 
                      public Projectable
{

public: 
  TSRConstraint(const TSRPtr& _tsr, int _maxIteration=1000);
  // :
   
  /// Size of constraints.
  size_t getConstraintDimension() const override;

  /// Value of constraints at _s.
  /// _s should be SE3State or CompoundState containing single SE3State.
  /// Returns 0-vector if _s is in TSR.
  Eigen::VectorXd getValue(const state::StatePtr& _s) const;

  /// Jacobian of constraints at _s.
  /// _s should be SE3State or CompoundState containing single SE3State.
  /// Returns SE3JacobianPtr for SE3State and CompoundJacobian for CompoundState.
  state::JacobianPtr getJacobian(const state::StatePtr& _s) const override;

  /// Returns a vector containing each constraint's type.
  std::vector<ConstraintType> getConstraintTypes() const override;


  /// True if this Projectable contains _s
  bool contains(const state::StatePtr& _s) const override;

  /// Returns projection of _q in this constraint.
  boost::optional<state::StatePtr> project(const state::StatePtr& _s) override;


private:

  /// Takes SE3 or CompoundStatePtr and return SE3StatePtr.
  state::SE3StatePtr convertToSE3(const state::StatePtr& _s) const;

  TSRPtr mTsr;
  int mMaxIteration;

};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_TSR_H_
