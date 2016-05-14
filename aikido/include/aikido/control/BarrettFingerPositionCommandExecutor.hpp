#ifndef AIKIDO_CONTROL_BARRETFINGERPOSITIONCOMMANDEXECUTOR_HPP_
#define AIKIDO_CONTROL_BARRETFINGERPOSITIONCOMMANDEXECUTOR_HPP_
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace control {

/// Trajectory executor for one finger. 
/// Assumes that fingers are underactuated: primal joint is actuated,
/// distal joint moves with certain mimic ratio (set to be Barret hand ratio).
class BarrettFingerPositionCommandExecutor
{
public:
  /// Constructor.
  /// \param _fingers fingers to be controlled by this Executor.
  /// \param _primal Index of primal dof
  /// \param _distal Index of distal dof 
  explicit BarrettFingerPositionCommandExecutor(
    ::dart::dynamics::ChainPtr _finger, int _primal, int _distal);

  /// Move primal joint by _stepAngle and distalJoint by _stepAngle*mimicRatio.
  /// If joint limit is reached, it will be set to the limit.
  /// \param _stepAngle Increment/decrement for dof pose.
  /// \param _distalOnly Move only distal joint by its mimic ratio.
  void execute(double _stepAngle, bool _distalOnly);

  /// Returns mimic ratio, i.e. how much the distal joint moves relative to 
  /// the primal joint. 
  /// \return mimic ratio.
  static double getMimicRatio();

private:
  ::dart::dynamics::ChainPtr mFinger; 
  constexpr static double mimicRatio = 0.333; 

  int mPrimal;
  int mDistal;
   
};

} // control
} // aikido

#endif
