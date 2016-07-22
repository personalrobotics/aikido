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


#ifndef MINTOS_CONSTRAINED_INTERPOLATOR_H
#define MINTOS_CONSTRAINED_INTERPOLATOR_H

#include <mintos/math/function.h>
#include <mintos/spline/GeneralizedBezierCurve.h>
#include <mintos/optimization/Newton.h>

namespace Mintos {

  /** @file ConstrainedInterpolator.h 
   * @brief Definitions for interpolation on a constrained manifold. 
   */

  using namespace std;
  using namespace Math;
  using namespace Spline;
  typedef Vector Config;

/** @brief Construct a polyline between a and b such that each point is near
 * the constraint C(x)=0.
 *
 * The method uses a recursive bisection technique, where the midpoint of
 * each segment is projected to the constraint until termination.  If
 * checkConstraints is true, the feasibility of each projected
 * point is checked.
 *
 * The projection uses a Newton-Raphson solver, capped at maxNewtonIters
 * iterations.  It ensures that each milestone satisfies ||C(x[k])|| <= ftol,
 * and d(x[k],x[k+1])<=xtol.
 *
 * maxGrowth defines the maximum extra distance that the path through a projected configuration can
 * add to the total length of the path.  That is, when going from x1 to x2, the projected midpoint
 * xm is checked so that d(x1,xm) + d(xm,x2) <= (1+maxGrowth)d(x1,x2).
 * To ensure convergence this parameter should be < 1 (default 0.9).
 */
class ConstrainedInterpolator
{
 public:
  ConstrainedInterpolator(GeodesicManifold* space,VectorFieldFunction* constraint);
  virtual ~ConstrainedInterpolator() {}
  bool Make(const Config& a,const Config& b,vector<Config>& path,bool checkConstraints=false);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);

  GeodesicManifold* space;
  VectorFieldFunction* constraint;
  Config xmin,xmax; ///< if set, uses bounds in the newton solver
  VectorFieldFunction* inequalities;  ///< if set, uses a nonlinear constraint in the newton solver
  int maxNewtonIters;
  Real ftol,xtol;
  Real maxGrowth;

  //temp: solver
  Optimization::NewtonRoot solver;
};

/** @brief Constructs a piecewise polynomial path between a and b such that
 * each point is near the constraint C(x)=0.
 *
 * Interpolation is accomplished via a cubic bezier curve.  This is slightly
 * faster than taking a regular ConstrainedInterpolator and then post-smoothing
 * it via a Bezier curve.
 *
 * The method uses a recursive bisection technique, where the midpoint of each segment
 * is projected to the constraint, and bisection continues until a given resolution
 * is met.  If checkConstraints is true, the feasibility of each projected
 * point is also checked.
 *
 * The projection uses a Newton-Raphson solver, capped at maxNewtonIters iterations.
 * It ensures that each milestone satisfies ||C(x[k])|| <= ftol, and
 * d(x[k],x[k+1])<=xtol.
 *
 * maxGrowth defines the maximum extra distance that the path through a projected
 * configuration can add to the total length of the path.  That is, when going from
 * x1 to x2, the projected midpoint
 * xm is checked so that d(x1,xm), d(xm,x2) <= (1+maxGrowth)/2 d(x1,x2).
 * To ensure convergence this parameter should be < 1 (default 0.9).
 */
class SmoothConstrainedInterpolator
{
 public:
  SmoothConstrainedInterpolator(GeodesicManifold* space,VectorFieldFunction* constraint);
  virtual ~SmoothConstrainedInterpolator() {}
  bool Make(const Config& a,const Config& b,
	    GeneralizedCubicBezierSpline& path,
	    bool checkConstraints=false);
  bool Make(const Config& a,const Vector& da,const Config& b,const Vector& db,
	    GeneralizedCubicBezierSpline& path,
	    bool checkConstraints=false);
  virtual void ConstraintValue(const Config& x,Vector& v);
  virtual bool Project(Config& x);
  virtual bool ProjectVelocity(const Config& x,Config& v);

  GeodesicManifold* space;
  VectorFieldFunction* constraint;
  Config xmin,xmax; ///< if set, uses bounds in the newton solver
  VectorFieldFunction* inequalities;  ///< if set, uses a nonlinear constraint in the newton solver
  int maxNewtonIters;
  Real ftol,xtol;
  Real maxGrowth;

  //temp: solver
  Optimization::NewtonRoot solver;
};

/** @brief Interpolates through several points smoothly using the given interpolator.
 * The output durations are such that point i corresponds precisely to path
 * parameter i.
 */
bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,
			    const vector<Vector>& pts,
			    GeneralizedCubicBezierSpline& path);


/** @brief Interpolates through several points smoothly using the given interpolator.
 * The output durations are such that point i corresponds precisely to path
 * parameter i.
 * The endpoint derivatives are set to dq0 and dq1, respectively.
 */
bool MultiSmoothInterpolate(SmoothConstrainedInterpolator& interp,
			    const vector<Vector>& pts,
			    const Vector& dq0,const Vector& dq1,
			    GeneralizedCubicBezierSpline& path);

/** @brief Adds a smooth path to the given point at the end of the provided path.
 * The path extension has duration suffixDuration.
 */
bool AppendInterpolate(SmoothConstrainedInterpolator& interp,
		       const Vector& pt,Real suffixDuration,
		       GeneralizedCubicBezierSpline& path);

} //namespace Mintos

#endif
