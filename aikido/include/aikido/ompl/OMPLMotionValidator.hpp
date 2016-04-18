#ifndef OMPL_MOTION_VALIDATOR_H_
#define OMPL_MOTION_VALIDATOR_H_

#include <ompl/base/MotionValidator.h>

namespace aikido
{
namespace ompl
{
/// Implement an OMPL MotionValidator.  This class checks the validity
///  of path segments between states (local planner).
class OMPLMotionValidator : public ::ompl::base::MotionValidator
{
public:
  /// Constructor.
  OMPLMotionValidator(const ::ompl::base::SpaceInformationPtr& _si,
                      const double& _maxDistBtwValidityChecks);

  /// Check if the path between two states, _s1 and _s2, is valid.  This
  /// function assumes _s1 is valid.
  bool checkMotion(const ::ompl::base::State* _s1,
                   const ::ompl::base::State* _s2) const override;

  // Check fi the path between two states is valid. Also compute the last state
  // that was valid and the time of that state.  The time is used to
  // parameterize the motion from _s1 to _s2, _s1 beging at t=0 and _s2 being at
  // t=1. The function assumes _s1 is valid.
  bool checkMotion(
      const ::ompl::base::State* _s1, const ::ompl::base::State* _s2,
      std::pair<::ompl::base::State*, double>& _lastValid) const override;

private:
    double mSequenceResolution;
};
}
}
#endif
