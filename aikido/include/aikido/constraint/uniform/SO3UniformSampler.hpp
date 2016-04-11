#ifndef AIKIDO_STATESPACE_SO3STATESPACESAMPLEABLECONSTRAINT_H_
#define AIKIDO_STATESPACE_SO3STATESPACESAMPLEABLECONSTRAINT_H_
#include "../../statespace/SO3StateSpace.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace statespace {

class SO3UniformSampler
  : public constraint::SampleableConstraint
{
public:
  SO3UniformSampler(
    std::shared_ptr<statespace::SO3StateSpace> _space,
    std::unique_ptr<util::RNG> _rng);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::SO3StateSpace> mSpace;
  std::unique_ptr<util::RNG> mRng;
};

} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_SO3STATESPACESAMPLEABLECONSTRAINT_H_
