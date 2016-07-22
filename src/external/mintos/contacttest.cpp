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

#include <mintos/ConstrainedInterpolator.h>
#include <mintos/ContactTimeScaling.h>
#include "Timer.h"
#include "src/primitives.h"
using namespace Mintos;
using namespace Mintos::Spline;
using namespace Math3D;


/** A 2D body with q=(x,y,theta).
 */
class DynamicsEquation2DBody : public DynamicsEquationBase
{
 public:
  Real mass,inertia;

  DynamicsEquation2DBody()
    :mass(1.0),inertia(1.0/12.0)
  {}
  virtual void MulInertia(const Vector& q,const Vector& ddq,Vector& Bddq) 
  { Bddq.resize(3); Bddq(0)=ddq(0)*mass; Bddq(1)=ddq(1)*mass; Bddq(2)=ddq(2)*inertia; }
  virtual void EvalCoriolis(const Vector& q,const Vector& dq,Vector& C)
  { C.resize(3); C.setZero(); }
  virtual void EvalGravity(const Vector& q,Vector& G)
  { G.resize(3); G.setZero(); G(1) = 9.8*mass; }
  virtual void EvalJacobian(const Vector3& pt,int i,int target,const Vector& q,Matrix& J)
  {
    Assert(target==-1);
    J.resize(3,3);
    J.setZero();
    J(0,0) = 1.0;
    J(1,1) = 1.0;
    J(0,2) = -pt.y;
    J(1,2) = pt.x;
  }
};


void TestContactBox()
{
  DynamicsEquation2DBody myDynamicsEquation;

  //a 3-unit side to side motion
  GeneralizedCubicBezierSpline path;
  path.segments.resize(1);
  path.durations.resize(1);
  path.segments[0].x0.resize(3);
  path.segments[0].x3.resize(3);
  path.segments[0].x3(0) = 3.0;
  path.segments[0].SetSmoothTangents();
  path.durations[0] = 1;
  //set very high accel and velocity bounds -- numerical errors may occur with
  //infinite bounds
  Vector amin(3,-1000),amax(3,1000),vmin(3,-1000),vmax(3,1000);
  Vector tmin(3),tmax(3);
  //first testing regular torques on all axes
  tmin.set(-20.0);
  tmax.set(20.0);

  //TODO: set up colocation points as desired -- this uses 100 points evenly spaced in the path domain
  Real pathDomainMax = path.TotalTime();
  vector<Real> colocationPoints(100);
  for(size_t i=0;i<colocationPoints.size();i++) 
    colocationPoints[i] = pathDomainMax*Real(i)/Real(colocationPoints.size());

  //with torque constraints
  {
    //create the time scaling
    TorqueTimeScaling scaling(&myDynamicsEquation);
    //initialize the path and torque constraints
    scaling.Init(path,colocationPoints,tmin,tmax);
    //constrains the start and end velocity
    scaling.SetStartStop();
    //optionally constrains the joint velocity and acceleration
    scaling.SetVelocityBounds(vmin,vmax);
    scaling.SetAccelerationBounds(amin,amax);
    if(scaling.Optimize()) {
      //solution is stored in scaling.traj
      printf("Torque-only solution found: duration %g\n",scaling.traj.EndTime());
    }
    else {
      printf("Torque-only time scaling failed.\n");
    }
  }

  //with torque and friction constraints -- here the contact points must
  //support the block.  This is sort of a weird constraint because it
  //overcomes the restriction on sliding contacts with an implementation trick.
  //moreover the friction forces can help the block accelerate, not just
  //decelerate.
  { 
    tmin.setZero();
    tmax.setZero();
    tmin(0) = -20.0;
    tmax(0) = 20.0;
    //set up two low-friction contacts
    ContactFormation formation;
    formation.links.resize(1);
    formation.contacts.resize(1);
    formation.links[0] = 0;
    formation.contacts[0].resize(2);
    formation.contacts[0][0].x.set(-0.5,-0.5,0);
    formation.contacts[0][0].n.set(0,1,0);
    formation.contacts[0][0].kFriction = 0.01;
    formation.contacts[0][1].x.set(0.5,0.5,0);
    formation.contacts[0][1].n.set(0,1,0);
    formation.contacts[0][1].kFriction = 0.01;

    vector<GeneralizedCubicBezierSpline> sections(1,path);
    vector<ContactFormation> stances(1,formation);
    
    //create the time scaling
    ContactTimeScaling scaling(&myDynamicsEquation);
    //initialize the path and all constraints
    scaling.Init(sections,colocationPoints,stances,
		 tmin,tmax,amin,amax,vmin,vmax);
    scaling.SetStartStop();
    if(scaling.Optimize()) {
      //solution is stored in scaling.traj
      printf("F/T solution found: duration %g\n",scaling.traj.EndTime());
    }
    else {
      printf("F/T time scaling failed.\n");
    }
  }  
}

int main(int argc,char** argv)
{
  TestContactBox();
  return 0;
}
