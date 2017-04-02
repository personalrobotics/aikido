#ifndef AIKIDO_STATESPACE_CONSTANTSAMPLER_H_
#define AIKIDO_STATESPACE_CONSTANTSAMPLER_H_
#include "../../statespace/Rn.hpp"
#include "../Sampleable.hpp"

namespace aikido {
namespace constraint {

/// ConstantSampler for RealVectorStates.
/// Stub sampler for WeldJoint or any fixed constant state space.
class ConstantSampler
  : public constraint::Sampleable
{
public:

  /// Constructor.
  /// \param _space Space in which this constraint operates.
  /// \param _value Value to return when sampled.
  ConstantSampler(
    std::shared_ptr<statespace::Rn> _space,
    const Eigen::VectorXd& _value);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

private:
  std::shared_ptr<statespace::Rn> mSpace;
  Eigen::VectorXd mValue;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_STATESPACE_CONSTANTSAMPLER_H_
