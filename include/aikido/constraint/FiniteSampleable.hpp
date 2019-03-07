#ifndef AIKIDO_CONSTRAINT_FINITESAMPLEABLE_HPP_
#define AIKIDO_CONSTRAINT_FINITESAMPLEABLE_HPP_

#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "Sampleable.hpp"

namespace aikido {
namespace constraint {

/// Constraint that always returns a finite set of samples.
/// Its SampleGenerator will generate sample
/// until all samples are exhausted.
class FiniteSampleable : public Sampleable
{
public:
  /// Constructor for single-sample constraint.
  /// \param _stateSpace StateSpace in which _state belongs.
  /// \param _state The only sample in this constraint.
  FiniteSampleable(
      statespace::StateSpacePtr _stateSpace,
      const aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState&
          _state);

  /// Constructor for multiple samples.
  /// \param _stateSpace StateSpace in which _states belong.
  /// \param _states Samples in this constraint.
  ///        SampleGenerator will generate samples in this order.
  FiniteSampleable(
      statespace::StateSpacePtr _stateSpace,
      const std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::
                            ScopedState>& _states);

  FiniteSampleable(const FiniteSampleable& other) = delete;
  FiniteSampleable(FiniteSampleable&& other) = delete;

  FiniteSampleable& operator=(const FiniteSampleable& other) = delete;
  FiniteSampleable& operator=(FiniteSampleable&& other) = delete;

  virtual ~FiniteSampleable();

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  std::unique_ptr<SampleGenerator> createSampleGenerator() const override;

private:
  statespace::ConstStateSpacePtr mStateSpace;
  std::vector<aikido::statespace::dart::MetaSkeletonStateSpace::ScopedState>
      mStates;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_FINITESAMPLEABLE_HPP_
