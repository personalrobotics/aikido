#ifndef AIKIDO_STATESPACE_SE2STATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_SE2STATESPACESAMPLEABLECONSTRAINT_H_
#include <array>
#include "SE2StateSpace.hpp"
#include "../constraint/Sampleable.hpp"

namespace aikido {
namespace statespace {


class SE2StateSpaceSampleGenerator
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
  SE2StateSpaceSampleGenerator(
    std::shared_ptr<statespace::SE2StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  std::shared_ptr<statespace::SE2StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::uniform_real_distribution<double> mDistributionX;
  std::uniform_real_distribution<double> mDistributionY;
  std::uniform_real_distribution<double> mDistributionAngle;

  friend class SE2StateSpaceSampleableConstraint;
};


class SE2StateSpaceSampleableConstraint
  : public constraint::SampleableConstraint
{
public:
  SE2StateSpaceSampleableConstraint(
    std::shared_ptr<statespace::SE2StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::SE2StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_SE2STATESPACESAMPLEABLECONSTRAINT_H_
