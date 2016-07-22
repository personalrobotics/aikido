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

#ifndef MINTOS_H
#define MINTOS_H

#include <mintos/ConstrainedInterpolator.h>
#include <mintos/TimeScaling.h>

namespace Mintos {

/** @brief Generates a constrained interpolator between a and b with
 * the given tolerance and time-optimizes it. 
 *
 * The space pointer can be NULL, in which case Cartesian space is used.
 * The constraint pointer can be NULL, in which case a regular bezier
 * interpolator is used.
 */
bool InterpolateAndTimeOptimize(const Config& a,const Config& b,
				GeodesicManifold* space,
				VectorFieldFunction* constraint,
				Real constraintTolerance,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output);

/** @brief Generates a constrained interpolator between all of the 
 * given milestones with constraint error up to the given tolerance,
 * then time-optimizes it. 
 *
 * The space pointer can be NULL, in which case Cartesian space is used.
 * The constraint pointer can be NULL, in which case a regular bezier
 * interpolator is used.
 */
bool InterpolateAndTimeOptimize(const vector<Config>& configs,
				GeodesicManifold* space,
				VectorFieldFunction* constraint,
				Real constraintTolerance,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output);

/** @brief Generates an unconstrained interpolator between all of the 
 * given milestones then time-optimizes it with the given timing resolution. 
 */
bool InterpolateAndTimeOptimize(const vector<Config>& configs,
				Real timingResolution,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output);


} //namespace Mintos

#endif
