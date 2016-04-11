#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
#include "RealVectorStateSpace.hpp"
#include "../constraint/Sampleable.hpp"

namespace aikido {
namespace statespace {


class RealVectorStateSpaceSampleGenerator
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
  RealVectorStateSpaceSampleGenerator(
    std::shared_ptr<statespace::RealVectorStateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  std::shared_ptr<statespace::RealVectorStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<std::uniform_real_distribution<double>> mDistributions;

  friend class RealVectorStateSpaceSampleableConstraint;
};


class RealVectorStateSpaceSampleableConstraint
  : public constraint::SampleableConstraint
{
public:
  RealVectorStateSpaceSampleableConstraint(
    std::shared_ptr<statespace::RealVectorStateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::RealVectorStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  Eigen::VectorXd mLowerLimits;
  Eigen::VectorXd mUpperLimits;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
