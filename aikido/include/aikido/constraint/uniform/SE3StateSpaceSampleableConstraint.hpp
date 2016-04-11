#ifndef AIKIDO_STATESPACE_SE3STATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_SE3STATESPACESAMPLEABLECONSTRAINT_H_
#include <array>
#include "../../statespace/SE3StateSpace.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {


class SE3StateSpaceSampleGenerator
  : public constraint::SampleGenerator
{
public:
  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool sample(statespace::StateSpace::State* _state) override;

  // Documentation inherited.
  int getNumSamples() const override;

  // Documentation inherited.
  bool canSample() const override;

private:
  SE3StateSpaceSampleGenerator(
    std::shared_ptr<statespace::SE3StateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::Vector3d& _lowerTranslationLimits,
    const Eigen::Vector3d& _upperTranslationLimits);

  std::shared_ptr<statespace::SE3StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::array<
    std::uniform_real_distribution<double>, 3> mTranslationDistributions;
  std::uniform_real_distribution<double> mOrientationDistribution;

  friend class SE3StateSpaceSampleableConstraint;
};


class SE3StateSpaceSampleableConstraint
  : public constraint::SampleableConstraint
{
public:
  SE3StateSpaceSampleableConstraint(
    std::shared_ptr<statespace::SE3StateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::Vector3d& _lowerTranslationLimits,
    const Eigen::Vector3d& _upperTranslationLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
  std::shared_ptr<statespace::SE3StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  Eigen::Vector3d mLowerLimits;
  Eigen::Vector3d mUpperLimits;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_SE3STATESPACESAMPLEABLECONSTRAINT_H_
