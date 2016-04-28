#ifndef AIKIDO_PLANNER_OMPL_MOTIONVALIDATOR_HPP_
#define AIKIDO_PLANNER_OMPL_MOTIONVALIDATOR_HPP_

#include <ompl/base/MotionValidator.h>

namespace aikido {
namespace planner {
namespace ompl {
/// Implement an OMPL MotionValidator.  This class checks the validity
///  of path segments between states.
class MotionValidator : public ::ompl::base::MotionValidator
{
public:
  /// Constructor.
  /// \param _si The SpaceInformation describing the planning space where this
  /// MotionValidator will be used
  /// \param _maxDistBtwValidityChecks The maximum distance (under the distance
  /// metric defined on the planning StateSpace) between two points on the
  /// segment checked for validity
  MotionValidator(const ::ompl::base::SpaceInformationPtr& _si,
                  const double& _maxDistBtwValidityChecks);

  /// Check if the path between two states, _s1 and _s2, is valid.  This
  /// function assumes _s1 is valid.
  /// \param _s1 The state at the start of the segment
  /// \param _s2 The state at the end of the segment
  bool checkMotion(const ::ompl::base::State* _s1,
                   const ::ompl::base::State* _s2) const override;

  // Check if the path between two states is valid. Also compute the last state
  // that was valid and the time of that state.  The time is used to
  // parameterize the motion from _s1 to _s2, _s1 being at t=0 and _s2 being at
  // t=1. The function assumes _s1 is valid.
  /// \param _s1 The state at the start of the segment
  /// \param _s2 The state at the end of the segment
  /// \param[out] _lastValid The last valid state on the segment and the segment
  /// time of that state (between 0 and 1)
  bool checkMotion(
      const ::ompl::base::State* _s1, const ::ompl::base::State* _s2,
      std::pair<::ompl::base::State*, double>& _lastValid) const override;

private:
    double mSequenceResolution;
};
}
}
}
#endif
