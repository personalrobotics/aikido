/*****************************************************************************
 *
 * Copyright (c) 2013, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#ifndef MINTOS_CONFIG_H
#define MINTOS_CONFIG_H

namespace Mintos {

  /** @file Config.h 
   * @brief Global configuration settings.
   */

///If true, uses double precision math
#define MATH_DOUBLE 1

///If true, will turn on debugging for time scaling's initial point
const static bool gDebugTimeScalingInitialPoint = false;

///OptimizeTimeScaling will pring warnings if the path's velocity /
///acceleration exceeds vWarningThresohld/aWarningThreshold, respectively.
const static double vWarningThreshold = 100, aWarningThreshold = 1000;

///ConstrainedInterpolator will print messages upon failure if this is set
///to >=1.
const static int gConstrainedInterpolateVerbose = 0;

///TimeScaling functions will print messages on failure if this is >=1
///and will provide basic information on optimization progress if this
///is >= 2
const static int gTimeScalingVerbose = 0;

///The number of iterations for SLP
const static int gSLPMaxIters = 100;

///SLP converges on X if the change in the X variable decreases below this
///threshold.
const static double gSLPXTolerance = 1e-5;

///SLP converges on F if the the change in the objective function decreases
///below this threshold
const static double gSLPFTolerance = 1e-7;

} //namespace Mintos

#endif
