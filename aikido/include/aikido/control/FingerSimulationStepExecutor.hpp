#ifndef AIKIDO_CONTROL_FINGERSIMULATIONSTEPEXECUTOR_HPP_
#define AIKIDO_CONTROL_FINGERSIMULATIONSTEPEXECUTOR_HPP_
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace control {

/// Trajectory executor for one finger. 
/// Assumes that fingers are underactuated: primal joint is actuated,
/// distal joint moves with certain mimic ratio (set to be Barret hand ratio).
class FingerSimulationStepExecutor
{
public:
  /// Constructor.
  /// \param _fingers fingers to be controlled by this Executor.
  explicit FingerSimulationStepExecutor(::dart::dynamics::ChainPtr _finger);

  /// Move primal joint by _stepAngle and distalJoint by _stepAngle*mimicRatio.
  /// If joint limit is reached, it will be set to the limit.
  /// \param _stepAngle Increment/decrement for dof pose.
  /// \param _distalOnly Move only distal joint by its mimic ratio.
  void execute(double _stepAngle, bool _distalOnly);

  static double getMimicRatio();

private:
  ::dart::dynamics::ChainPtr mFinger; 
  constexpr static double mimicRatio = 0.333;  
};

} // control
} // aikido

#endif
