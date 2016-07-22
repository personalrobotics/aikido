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

#ifndef GENERALIZED_BEZIER_CURVE_H
#define GENERALIZED_BEZIER_CURVE_H

#include <mintos/GeodesicSpace.h>
#include <iostream>
#include <vector>

namespace Mintos {
namespace Spline {

/** @brief A generalized Bezier curve that uses the CSpace's Interpolate routine to generate
 * the smooth interpolation.
 *
 * The members x0,...,x3 need to be set up as valid configs in the given space.
 * The De Casteljau algorithm is used to generate the output.
 */
class GeneralizedCubicBezierCurve
{
 public:
  GeneralizedCubicBezierCurve(Mintos::GeodesicManifold* space=NULL);
  ///Evaluate the bezizer curve at point u
  void Eval(Real u,Config& x) const;
  ///Only works properly if the space is cartesian or manifold is set
  void Deriv(Real u,Config& dx) const;
  ///Only works properly if the space is cartesian
  void Accel(Real u,Config& ddx) const;
  ///Evaluates the length of the "frame" from x0->x1->x2->x3
  Real OuterLength() const;

  ///These work only with cartesian spaces
  void GetBounds(Vector& xmin,Vector& xmax) const;
  void GetDerivBounds(Vector& vmin,Vector& vmax,Vector& amin,Vector& amax) const;
  void GetDerivBounds(Real u1,Real u2,Vector& vmin,Vector& vmax,Vector& amin,Vector& amax) const;

  ///Helper: sets x1 and x2 to smoothly interpolate from previous/next configs.
  void SetSmoothTangents(const Config* prev=NULL,const Config* next=NULL);
  ///Helper: sets x1 and x2 to smoothly interpolate from previous/next configs,
  ///where dtprev is the amount of time elapsed from prev to x0, 1 is the duration
  ///of this curve, and dtnext is  the amount of time elapsed from x3 to next
  void SetSmoothTangents(const Config* prev,const Config* next,Real dtprev,Real dtnext);
  ///Helper: sets x1 and x2 to "naturally" interpolate the given tangents.
  ///To specify a non-constrained tangent, set the vector to be empty
  void SetNaturalTangents(const Vector& dx0,const Vector& dx1);

  ///These may be slightly cheaper than Eval(0.5) and Deriv(0.5) for cartesian spaces
  void Midpoint(Vector& x) const;
  void MidpointDeriv(Vector& v) const;
  void MidpointTimeDeriv(Real duration,Vector& v) const;

  Mintos::GeodesicManifold* space;
  Config x0,x1,x2,x3;
};


/** @ingroup MotionPlanning
 * @brief A Bezier spline with segment durations.
 */
class GeneralizedCubicBezierSpline
{
public:
  int ParamToSegment(Real u,Real* segparam=NULL) const;
  int Eval(Real u,Vector& x) const;
  int Deriv(Real u,Vector& dx) const;
  int Accel(Real u,Vector& ddx) const;
  Real TotalTime() const;
  const Config& Start() const { return segments.front().x0; }
  const Config& End() const { return segments.back().x3; }
  void Append(const GeneralizedCubicBezierCurve& seg,Real duration);
  void Concat(const GeneralizedCubicBezierSpline& s);
  void TimeScale(Real scale);
  ///gets the piecewise linear path corresponding to each segment endpoint
  void GetPiecewiseLinear(std::vector<Real>& times,std::vector<Config>& milestones) const;
  bool Save(std::ostream& out) const;
  bool Load(std::istream& in,Mintos::GeodesicManifold* space=NULL);

  std::vector<GeneralizedCubicBezierCurve> segments;
  std::vector<Real> durations;
};

} //namespace Spline
} //namespace Mintos

#endif
