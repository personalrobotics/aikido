#ifndef AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_
#define AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_

#include <chrono>

#include "aikido/planner/kunzretimer/KunzRetimer.hpp"
#include "aikido/planner/parabolic/ParabolicSmoother.hpp"

namespace aikido {
namespace planner {

/// Enum for which postprocessor is being used.
enum class PostProcessorType
{
  HAUSER,
  KUNZ
};

/// Trajectory postprocessing parameters. This struct can be used with the
/// `ConcreteRobot` class, and let's the user specify the exact processor and
//  and parameters to use.
struct PostProcessorParams
{
  /// Convenience constructor for when using Hauser.
  PostProcessorParams(
      bool enableShortcut = true,
      bool enableBlend = true,
      double shortcutTimelimit = 0.6,
      double blendRadius = 0.4,
      int blendIterations = 1,
      double feasibilityCheckResolution = 1e-3,
      double feasibilityApproxTolerance = 1e-3)
    : mPostProcessorType(PostProcessorType::HAUSER)
    , mHauserParams{enableShortcut,
                    enableBlend,
                    shortcutTimelimit,
                    blendRadius,
                    blendIterations,
                    feasibilityCheckResolution,
                    feasibilityApproxTolerance}
  {
    // Do nothing.
  }

  /// Convenience constructor for when using Kunz.
  PostProcessorParams(double maxDeviation = 0.1, double timeStep = 0.01)
    : mPostProcessorType(PostProcessorType::KUNZ)
    , mKunzParams{maxDeviation, timeStep}
  {
    // Do nothing.
  }

  /// Which postprocessor is being used.
  PostProcessorType mPostProcessorType;
  /// Params used if `mPostProcessorType` indicates we are using Hauser.
  parabolic::Params mHauserParams;
  /// Params used if `mPostProcessorType` indicates we are using Kunz.
  kunzretimer::Params mKunzParams;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_POSTPROCESSORPARAMS_HPP_
