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

#ifndef MINTOS_CONTACT_TIME_SCALING_H
#define MINTOS_CONTACT_TIME_SCALING_H

#include "TimeScaling.h"
#include <mintos/math3d/primitives.h>
#include <mintos/math/matrix.h>

namespace Mintos { 

  using namespace Math3D;

/** @brief A base class for a time scaling with colocation point constraints.
 * Subclasses must fill in dsmax, ds2ddsConstraintNormals, and
 * ds2ddsConstraintOffsets before Optimize is called.
 *
 * Each dynamic constraint is specified as 
 * dot(ds2ddsConstraintNormals[i],((s')^2,s') <= ds2ddsConstraintOffsets[i].
 *
 * Output is given in traj.
 *
 * Users will usually use TorqueTimeScaling, ZMPTimeScaling, or
 * ContactTimeScaling unless they wish to implement special constraints.
 */
class CustomTimeScaling
{
 public:
  CustomTimeScaling();
  ///Sets up the path
  void SetPath(const GeneralizedCubicBezierSpline& path,
	       const vector<Real>& colocationParams);
  ///Sets up the path -- multiple section version.  If path sections are
  ///timed, the colocationParams are given in the total range. Otherwise,
  ///the colocationParams are given in the range [0,1].
  void SetPath(const vector<GeneralizedCubicBezierSpline>& paths,
	       const vector<Real>& colocationParams);
  ///Sets up the path to start and stop at zero velocity
  void SetStartStop();
  ///Sets up velocity bounds
  void SetVelocityBounds(const Vector& vmin,const Vector& vmax);
  ///Sets up acceleration bounds
  void SetAccelerationBounds(const Vector& amin,const Vector& amax);
  ///Runs the optimizer with the custom constraints
  bool Optimize();

  ///Output trajectory
  TimeScaledBezierCurve traj;
  ///colocation grid
  vector<Real> paramDivs;
  ///index from grid to section, if multiple sections are given
  vector<int> paramSections;
  ///colocation points
  vector<Vector> xs,dxs,ddxs;
  ///time scaling maxima.  Defaults to Inf
  vector<Real> dsmax;
  ///Constraint planes a^T (ds^2,dds) <= b in the
  ///(squared-rate, acceleration) plane
  vector<vector<Vector2> > ds2ddsConstraintNormals;
  vector<vector<Real> > ds2ddsConstraintOffsets;
};

/** @brief Hooks for evaluating the system's dynamics equation.
 *
 * EvalJacobian only needs to be overloaded for ContactTimeScaling.
 */
class DynamicsEquationBase
{
 public:
  ///Multiply the inertia matrix Bddq = B(q)*ddq
  virtual void MulInertia(const Vector& q,const Vector& ddq,Vector& Bddq) = 0;
  ///Evaluate the Coriolis term C(q,dq)
  virtual void EvalCoriolis(const Vector& q,const Vector& dq,Vector& C) = 0;
  ///Evaluate the gravity term G(q)
  virtual void EvalGravity(const Vector& q,Vector& G) = 0;
  ///Evaluates the Jacobian J(q) of the point pt attached to link i (pt and
  ///jacobian are given in target coordinates)
  virtual void EvalJacobian(const Vector3& pt,int i,int target,const Vector& q,Matrix& J) {}
};

/** @brief A time scaling with torque constraints |t| <= tmax.
 * Assuming fixed base manipulator.
 *
 * t = B(q)q'' + C(q,q') + G(q)
 * if v is the tangent, a is the curvature, then q' = ds*v, q'' = v*dds + a*ds^2
 * t = ds^2 (B(q)a+C(q,v)) + dds B(q)v + G(q)
 *
 * The torqueLimitShift and torqueLimitScale terms let you adjust the
 *  robot's torque limits to account for errors during execution.
 */
class TorqueTimeScaling : public CustomTimeScaling
{
 public:
  TorqueTimeScaling(DynamicsEquationBase* dynamics);
  void Init(const GeneralizedCubicBezierSpline& path,const vector<Real>& colocationParams,const Vector& torquemin,const Vector& torquemax);

  DynamicsEquationBase* dynamics;
};


/** @brief A single contact point, i.e. a position, normal, and friction
 * coefficient.
 */
struct ContactPoint
{
  Vector3 x;
  Vector3 n;
  Real kFriction;
};

/** @brief A formation of robot-environment or robot-robot contacts.
 * The list of contacts contacts[i] is associated with links[i]
 * (pointing from the opposing body into the robot).
 *
 * Usually the opposing body is the environment, but if targets is
 * non-empty, it gives the index of the body for which contact is made.
 * In this case, the contact coordinates are given relative to the target.
 */
struct ContactFormation
{
  std::vector<int> links;
  std::vector<std::vector<ContactPoint> > contacts;
  std::vector<int> targets;
};


/** @brief A time scaling with torque/contact constraints
 *
 * Uses dynamic equation B(q)*q''+C(q,q')+G(q) = t+J^T(q)*f
 *   |t| <= tmax
 *   f \in FC
 * with f being the contact forces, J being the jacobian of the contact
 * points, tmax being the torque limits, and FC being the (polygonalized)
 * friction cones.
 *
 * Can shift/scale torque limits, reduce the friction coefficients, and
 * add a robustness margin to the friction cones.
 */
class ContactTimeScaling : public CustomTimeScaling
{
 public:
  ContactTimeScaling(DynamicsEquationBase* dynamics);
  ///Assigns each path in the paths vector the corresponding stance in the
  ///stances vector.
  ///Discretizes friction cones into pyramids with numFCEdges edges.
  bool Init(const vector<GeneralizedCubicBezierSpline>& paths,const vector<Real>& colocationParams,
	    const vector<ContactFormation>& stances,
	    const Vector& tmin,const Vector& tmax,
	    const Vector& amin,const Vector& amax,
	    const Vector& vmin,const Vector& vmax,
	    int numFCEdges = 4);

  DynamicsEquationBase* dynamics;
  Real forceRobustness;   ///< >= 0, indicates the absolute margin for forces to be contained within the friction cone
};


} //namespace Mintos

#endif
