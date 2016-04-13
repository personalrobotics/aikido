#ifndef AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_H_
#define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_H_
#include <vector>
#include "Sampleable.hpp"
#include "../statespace/CompoundStateSpace.hpp"

namespace aikido {
namespace constraint {

class SampleableSubSpace : public SampleableConstraint
{
public:
  SampleableSubSpace(
    std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
    std::vector<std::shared_ptr<SampleableConstraint>> _constraints);

  statespace::StateSpacePtr getStateSpace() const override;

  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<std::shared_ptr<SampleableConstraint>> mConstraints;
};

} // namespace constraint
} // namespace aikido

#endif // define AIKIDO_CONSTRAINT_SAMPLEABLESUBSPACE_H_
