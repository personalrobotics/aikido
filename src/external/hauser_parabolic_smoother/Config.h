#ifndef PARABOLIC_RAMP_CONFIG_H
#define PARABOLIC_RAMP_CONFIG_H

#include <assert.h>

///assertion function
#define PARABOLIC_RAMP_ASSERT(x)

///print an error
#define PARABOLIC_RAMP_PERROR(...)

///print a notification
#define PARABOLIC_RAMP_PLOG(...)

namespace ParabolicRamp {

  ///tolerance for time equality
  const static Real EpsilonT = 1e-6;

  ///tolerance for position equality
  const static Real EpsilonX = 1e-5;

  ///tolerance for velocity equality
  const static Real EpsilonV = 1e-5;

  ///tolerance for acceleration equality
  const static Real EpsilonA = 1e-6;

  ///self validity check level:
  ///- 0 no checking
  ///- 1 moderate checking 
  ///- 2 full checking
  const static int gValidityCheckLevel = 2;

  ///verbosity level:
  ///- 0 all messages off
  ///- 1 brief messages
  ///- 2 detailed messages
  const static int gVerbose = 2;

  ///whether or not to pause on serious errors
  const static bool gErrorGetchar = false;

  ///whether or not errors are logged to disk
  const static bool gErrorSave = false;

} //namespace ParabolicRamp

#endif
