#ifndef AIKIDO_STATESPACE_SO2STATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_SO2STATESPACESAMPLEABLECONSTRAINT_H_
#include "../../statespace/SO2StateSpace.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {

class SO2StateSpaceSampleGenerator
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
  SO2StateSpaceSampleGenerator(
    std::shared_ptr<statespace::SO2StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  std::shared_ptr<statespace::SO2StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::uniform_real_distribution<double> mDistribution; 

  friend class SO2StateSpaceSampleableConstraint;
};


class SO2StateSpaceSampleableConstraint
  : public constraint::SampleableConstraint
{
public:
  SO2StateSpaceSampleableConstraint(
    std::shared_ptr<statespace::SO2StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::SO2StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_SO2STATESPACESAMPLEABLECONSTRAINT_H_
