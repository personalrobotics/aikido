#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACESAMPLEABLECONSTRAINT_H_
#include <vector>
#include "../../statespace/CompoundStateSpace.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {


class CompoundStateSpaceSampleGenerator
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
  CompoundStateSpaceSampleGenerator(
    std::shared_ptr<statespace::CompoundStateSpace> _space,
    std::unique_ptr<util::RNG> _rng,
    std::vector<std::unique_ptr<constraint::SampleGenerator>> _delegates);

  std::shared_ptr<statespace::CompoundStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<std::unique_ptr<constraint::SampleGenerator>> mDelegates;

  friend class CompoundStateSpaceSampleableConstraint;
};


class CompoundStateSpaceSampleableConstraint
  : public constraint::SampleableConstraint
{
public:
  CompoundStateSpaceSampleableConstraint(
    std::shared_ptr<statespace::CompoundStateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
  std::vector<constraint::SampleableConstraintPtr> mDelegates;
};


} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_COMPOUNDSTATESPACESAMPLEABLECONSTRAINT_H_
