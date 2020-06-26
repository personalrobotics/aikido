#ifndef AIKIDO_ROBOT_ROBOTPARAMS_HPP_
#define AIKIDO_ROBOT_ROBOTPARAMS_HPP_

#include <chrono>

namespace aikido {
namespace robot {

/// Enum for which postprocessor is being used.
enum class PostProcessorType
{
  HAUSER,
  KUNZ
};

/// Hauser postprocessor parameters.
struct HauserParams
{
  bool mEnableShortcut;
  bool mEnableBlend;
  double mShortcutTimelimit;
  double mBlendRadius;
  int mBlendIterations;
  double mFeasibilityCheckResolution;
  double mFeasibilityApproxTolerance;
};

/// Kunz postprocessor parameters.
struct KunzParams
{
  double mMaxDeviation;
  double mTimeStep;
};

/// Trajectory postprocessing parameters.
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
  HauserParams mHauserParams;
  /// Params used if `mPostProcessorType` indicates we are using Kunz.
  KunzParams mKunzParams;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_ROBOTPARAMS_HPP_
