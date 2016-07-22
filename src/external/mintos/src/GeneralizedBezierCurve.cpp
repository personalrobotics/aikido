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
#include <mintos/spline/GeneralizedBezierCurve.h>
#include <mintos/math/interpolate.h>
using namespace std;
using namespace Mintos::Spline;
using namespace Mintos;

GeneralizedCubicBezierCurve::GeneralizedCubicBezierCurve(GeodesicManifold* _space)
  :space(_space)
{}

void GeneralizedCubicBezierCurve::SetNaturalTangents(const Vector& dx0,const Vector& dx1)
{
  const static double third = 1.0/3.0;
  if(dx0.empty() && dx1.empty()) {
    if(space==NULL) {
      interpolate(x0,x3,third,x1);
      interpolate(x3,x0,third,x2);
    }
    else {
      space->Interpolate(x0,x3,third,x1);
      space->Interpolate(x3,x0,third,x2);
    }
  }
  else {
    Config dxp,dxn;
    if(space==NULL) {
      if(!dx1.empty()) {
	x2=x3-dx1*third;
      }
      if(!dx0.empty()) {
	x1 = x0+dx0*third;
      }
      if(dx1.empty()) x2 = x1 + third*(x3-x0);
      if(dx0.empty()) x1 = x2 - third*(x3-x0);
    }
    else {
      Vector temp;
      if(!dx1.empty()) 
	space->Integrate(x3,dx1*(-third),x2);
      if(!dx0.empty()) 
	space->Integrate(x0,dx0*third,x1);
      if(dx1.empty()) {
	Assert(!x1.empty());
	space->InterpolateDeriv(x1,x0,0,dxp);
	space->InterpolateDeriv(x1,x3,0,dxn);
	space->Integrate(x1,third*(dxn-dxp),x2);
      }
      if(dx0.empty()) {
	Assert(!x2.empty());
	space->InterpolateDeriv(x2,x0,0,dxp);
	space->InterpolateDeriv(x2,x3,0,dxn);
	space->Integrate(x2,third*(dxp-dxn),x1);
      }
    }
  }
}

void GeneralizedCubicBezierCurve::SetSmoothTangents(const Config* prev,const Config* next)
{
  Config dxp,dxn;
  if(space==NULL) {
    if(next) 
      dxn = (*next-x0)*0.5;
    if(prev) 
      dxp = (x3-*prev)*0.5;
  }
  else {
    Vector temp;
    if(next) { 
      space->InterpolateDeriv(x3,*next,0,dxn);
      space->InterpolateDeriv(x3,x0,0,temp);
      dxn -= temp;
      dxn *= 0.5;
    }
    if(prev) {
      space->InterpolateDeriv(x0,x3,0,dxp);
      space->InterpolateDeriv(x0,*prev,0,temp);
      dxp -= temp;
      dxp *= 0.5;
    }
  }
  SetNaturalTangents(dxp,dxn);
}

/* x0 = y(0)
 * x3 = y(dt)
 * xp = y(-dtp)
 * xn = y(dt+dtn)
 * fit a quadratic to each segment 
 *   y(u) = a u^2 + b u + c
 * and find y'(0)=b for prev segment, y'(dt) for next
 * 
 * c = x0
 * a dt^2 + b dt = x3-x0
 * a dtp^2 - b dtp = xp-x0
 * [dt^2  dt  ][a] = [x3-x0]
 * [dtp^2 -dtp][b]   [xp-x0]
 * 1/(dt^2 dtp + dt dtp^2) [ dtp   dt    ][x3-x0] = [a]
 *                         [ dtp^2 -dt^2][xp-x0]   [b]
 * b = (dtp^2 (x3-x0) - dt^2(xp-x0))/(dtp*dt*(dt+dtp))
 *   = [(x3-x0)dtp/dt - (xp-x0)dt/dtp]/(dt+dtp)
 *   = [(x3-x0)dtp/dt - (xp-x0)dt/dtp]/(dt+dtp)
 * assuming dt = 1
 *   = [(x3-x0)dtp - (xp-x0)/dtp]/(1+dtp)
 */
void GeneralizedCubicBezierCurve::SetSmoothTangents(const Config* prev,const Config* next,Real dtprev,Real dtnext)
{
  Config dxp,dxn,temp;
  if(next) {
    if(space==NULL) {
      temp = x0 - x3;
      dxn = *next - x3;
    }
    else {
      space->InterpolateDeriv(x3,*next,0,dxn);
      space->InterpolateDeriv(x3,x0,0,temp);
    }
    dxn *= 1.0/dtnext;
    temp *= dtnext;
    dxn -= temp;
    dxn *= 1.0/(1.0+dtnext);
  }
  if(prev) {
    if(space == NULL) {
      dxp = x3 - x0;
      temp = *prev - x0;
    }
    else {
      space->InterpolateDeriv(x0,x3,0,dxp);
      space->InterpolateDeriv(x0,*prev,0,temp);
    }
    temp *= 1.0/dtprev;
    dxp *= dtprev;
    dxp -= temp;
    dxp *= 1.0/(1.0+dtprev);
  }
  SetNaturalTangents(dxp,dxn);
}


//x01 = (1-u)x0 + u x1
//x12 = (1-u)x1 + u x2
//x23 = (1-u)x2 + u x3
//x012 = (1-u)x01 + u x12
//x123 = (1-u)x12 + u x23
//x(u) = (1-u)x012 + u x123
void GeneralizedCubicBezierCurve::Eval(Real u,Config& x) const
{
  Config temp1,temp2,temp3,temp4;
  if(space == NULL) {
    interpolate(x0,x1,u,temp1);
    interpolate(x1,x2,u,temp2);
    interpolate(x2,x3,u,temp3);
    interpolate(temp1,temp2,u,temp4);
    temp1.swap(temp4);
    interpolate(temp2,temp3,u,temp4);
    temp2.swap(temp4);
    interpolate(temp1,temp2,u,x);
  }
  else {
    space->Interpolate(x0,x1,u,temp1);
    space->Interpolate(x1,x2,u,temp2);
    space->Interpolate(x2,x3,u,temp3);
    space->Interpolate(temp1,temp2,u,temp4);
    temp1.swap(temp4);
    space->Interpolate(temp2,temp3,u,temp4);
    temp2.swap(temp4);
    space->Interpolate(temp1,temp2,u,x);
  }
}

//x'(u) = -x012 + x123 + (1-u)x012' + u x123'
//x012' = -x01 + x12 + (1-u)x01' + u x12'
//x123' = -x12 + x23 + (1-u)x12' + u x23'
//x01' = x1-x0
//x12' = x2-x1
//x23' = x3-x2
//x012' = x12-x01 + (1-u)(x1-x0) + u (x2-x1) = x12-x01 + (1-u)x1 + u x2 - [(1-u)x0 + u x1] = 2(x12-x01)
//x123' = x23-x12 + (1-u)(x2-x1) + u (x3-x2) = 2 (x23-x12)
//x'(u) = -x012 + x123 + 2(1-u)(x12-x01) + 2 u(x23-x12) = x123-x012 + 2[(1-u)x12 + u x23 -[(1-u) x01 + u x12]]
// = 3 (x123-x012)

//f(a,b,u) = (1-u)a+ub
//df/da = (1-u)I, df/db = uI, df/du = b-a
//df/da da/du
//x(u) = f(x012,x123,u)
//x'(u) = df/da(x012,x123,u) dx012/du + df/db(x012,x123,u) dx123/du + df/du(x0123,x123)
//x012'(u) = df/da(x01,x12,u) dx01/du + df/db(x01,x12,u) dx12/du + df/du(x01,x12,u)
//x01'(u) = df/du(x0,x1,u)
//x12'(u) = df/du(x1,x2,u)
//x23'(u) = df/du(x2,x3,u)
//x012'(u) = df/da(x01,x12,u) dx01/du + df/db(x01,x12,u) dx12/du + df/du(x01,x12,u)
void GeneralizedCubicBezierCurve::Deriv(Real u,Config& dx) const
{
  if(!space) {
    Config temp1,temp2,temp3,temp4;
    if(!space) {
      interpolate(x0,x1,u,temp1);
      interpolate(x1,x2,u,temp2);
      interpolate(x2,x3,u,temp3);
      interpolate(temp1,temp2,u,temp4);
      temp1.swap(temp4);
      interpolate(temp2,temp3,u,temp4);
    }
    else {
      space->Interpolate(x0,x1,u,temp1);
      space->Interpolate(x1,x2,u,temp2);
      space->Interpolate(x2,x3,u,temp3);
      space->Interpolate(temp1,temp2,u,temp4);
      temp1.swap(temp4);
      space->Interpolate(temp2,temp3,u,temp4);
    }
    dx = temp4 - temp1;
    dx *= 3.0;
  }
  else {
    Config dx01,dx12,dx23,x01,x12,x23,x012,x123,dx012,dx123,temp;
    space->Interpolate(x0,x1,u,x01);
    space->Interpolate(x1,x2,u,x12);
    space->Interpolate(x2,x3,u,x23);
    space->InterpolateDeriv(x0,x1,u,dx01);
    space->InterpolateDeriv(x1,x2,u,dx12);
    space->InterpolateDeriv(x2,x3,u,dx23);
    space->InterpolateDeriv(x01,x12,u,dx012);
    space->InterpolateDerivA(x01,x12,u,dx01,temp);
    dx012 += temp;
    space->InterpolateDerivB(x01,x12,u,dx12,temp);
    dx012 += temp;
    space->InterpolateDeriv(x12,x23,u,dx123);
    space->InterpolateDerivA(x12,x23,u,dx12,temp);
    dx123 += temp;
    space->InterpolateDerivB(x12,x23,u,dx23,temp);
    dx123 += temp;
    space->Interpolate(x01,x12,u,x012);
    space->Interpolate(x12,x23,u,x123);
    space->InterpolateDeriv(x012,x123,u,dx);
    space->InterpolateDerivA(x012,x123,u,dx012,temp);
    dx += temp;
    space->InterpolateDerivB(x012,x123,u,dx123,temp);
    dx += temp;
  }
  /*
  if(u == 1.0) {
    cout<<"End deriv: "<<endl;
    cout<<" x0="<<x0<<endl;
    cout<<" x1="<<x1<<endl;
    cout<<" x2="<<x2<<endl;
    cout<<" x3="<<x3<<endl;
    cout<<"Result="<<dx<<endl;
    cout<<"Ideal="<<(x3-x2)*3.0<<endl;
  }
  */
}

//x'(u) = 3 (x123-x012)
//x''(u) = 3 (x123'-x012') = 6 (x23-2x12+x01) 

//f(a,b,u) = (1-u)a+ub
//x01''(u) = d/du(df/du(x0,x1,u)) = ddf/du^2(x0,x1,u)
//x12''(u) = d/du(df/du(x1,x2,u)) = ddf/du^2(x1,x2,u)
//x23''(u) = d/du(df/du(x2,x3,u)) = ddf/du^2(x2,x3,u)
//x012''(u) = d/du[df/da(x01,x12,u) dx01/du + df/db(x01,x12,u) dx12/du + df/du(x01,x12,u)] =
//d/du[df/da(x01,x12,u) dx01/du] = ddf/duda(x01,x12,u) dx01/du + df/da(x01,x12,u) x01''(u)
//crazy...
void GeneralizedCubicBezierCurve::Accel(Real u,Config& ddx) const
{
  //Assert(space == NULL);
  Config temp1,temp2,temp3;
  if(space == NULL) {
    interpolate(x0,x1,u,temp1);
    interpolate(x1,x2,u,temp2);
    interpolate(x2,x3,u,temp3);
  }
  else {
    space->Interpolate(x0,x1,u,temp1);
    space->Interpolate(x1,x2,u,temp2);
    space->Interpolate(x2,x3,u,temp3);
  }
  if(space == NULL) {
    ddx.add(temp1,temp3);
    ddx.madd(temp2,-2.0);
    ddx *= 6;
  }
  else {
    static bool warned = false;
    if(!warned) {
      fprintf(stderr,"GeneralizedCubicBezierCurve: Warning, using linear acceleration evaluation\n");
      warned=true;
    }
    Vector temp;
    space->InterpolateDeriv(temp2,temp1,0,temp);
    space->InterpolateDeriv(temp2,temp3,0,ddx);
    ddx += temp;
    ddx *= 6;
  }
}

void GeneralizedCubicBezierCurve::GetBounds(Vector& xmin,Vector& xmax) const
{
  xmin = xmax = x0;
  for(int i=0;i<x0.n;i++) {
    if(x1(i) < xmin(i)) xmin(i) = x1(i);
    else if(x1(i) > xmax(i)) xmax(i) = x1(i);
    if(x2(i) < xmin(i)) xmin(i) = x2(i);
    else if(x2(i) > xmax(i)) xmax(i) = x2(i);
    if(x3(i) < xmin(i)) xmin(i) = x3(i);
    else if(x3(i) > xmax(i)) xmax(i) = x3(i);
  }
}

//only works with cartesian spaces
//x01 = (1-u)x0 + u x1
//x12 = (1-u)x1 + u x2
//x23 = (1-u)x2 + u x3
//x012 = (1-u)x01 + u x12
//x123 = (1-u)x12 + u x23
//x(u) = (1-u)x012 + u x123
//x'(u) = 3 (x123-x012)
//      = 3 ((1-u)x12 + u x23 - (1-u)x01- u x12)
//      = 3 ((1-u)((1-u)x1 + u x2) + u ((1-u)x2 + u x3) - (1-u)((1-u)x0 + u x1) - u((1-u)x1 + u x2))
//      = 3 ((1-u)^2(x1-x0) + (1-u)u(2 x2 - 2 x1) + u^2(x3 - x2)
//critical points: x''(u)=0 
//x''(u) = 6 (x23-2 x12+x01) 
//       = (1-u)*(6 x2 - 12 x1 + 6 x0) + u*(6 x3 - 12 x2 + 6 x1)
void GeneralizedCubicBezierCurve::GetDerivBounds(Vector& vmin,Vector& vmax,Vector& amin,Vector& amax) const
{
  if(!space) {
    amin = 6.0*(x2 - 2.0*x1 + x0);
    amax = 6.0*(x3 - 2.0*x2 + x1);

    vmin = 3.0*(x1-x0);
    vmax = 3.0*(x3-x2);
    for(int i=0;i<vmin.n;i++) {
      if(vmin[i] > vmax[i]) Swap(vmin[i],vmax[i]);
      if(Sign(amin[i]) != Sign(amax[i])) {
	Real criticalPoint = amin[i]/(amin[i]-amax[i]);
	Assert(criticalPoint >= 0 && criticalPoint <= 1.0);
	Real u2=Sqr(criticalPoint);
	Real t2=Sqr(1.0-criticalPoint);
	Real ut=criticalPoint*(1.0-criticalPoint);
	Real v = 3.0*(t2*(x1[i]-x0[i])+u2*(x3[i]-x2[i])+2.0*ut*(x2[i]-x1[i]));
	if(v < vmin[i]) vmin[i]=v;
	if(v > vmax[i]) vmax[i]=v;
      }
      if(amin[i] > amax[i]) Swap(amin[i],amax[i]);
    }
  }
  else {
    Accel(0,amin);
    Accel(1,amax);
    Deriv(0,vmin);
    Deriv(1,vmax);
    for(int i=0;i<vmin.n;i++) {
      if(vmin[i] > vmax[i]) Swap(vmin[i],vmax[i]);
      if(Sign(amin[i]) != Sign(amax[i])) {
	Real criticalPoint = amin[i]/(amin[i]-amax[i]);
	Assert(criticalPoint >= 0 && criticalPoint <= 1.0);
	Vector v;
	Deriv(criticalPoint,v);
	if(v[i] < vmin[i]) vmin[i]=v[i];
	if(v[i] > vmax[i]) vmax[i]=v[i];
      }
      if(amin[i] > amax[i]) Swap(amin[i],amax[i]);
    }
  }
}


void GeneralizedCubicBezierCurve::GetDerivBounds(Real u1,Real u2,Vector& vmin,Vector& vmax,Vector& amin,Vector& amax) const
{
  Accel(u1,amin);
  Accel(u2,amax);

  Deriv(u1,vmin);
  Deriv(u2,vmax);

  for(int i=0;i<vmin.n;i++) {
    if(vmin[i] > vmax[i]) Swap(vmin[i],vmax[i]);
    if(Sign(amin[i]) != Sign(amax[i])) {
      Real criticalPoint = amin[i]/(amin[i]-amax[i]);
      Assert(criticalPoint >= 0 && criticalPoint <= 1.0);
      Real u=u1+criticalPoint*(u2-u1);
      if(!space) {
	Real u2=Sqr(u);
	Real t2=Sqr(1.0-u);
	Real ut=u*(1.0-u);
	Real v = 3.0*(t2*(x1[i]-x0[i])+u2*(x3[i]-x2[i])+2.0*ut*(x2[i]-x1[i]));
	if(v < vmin[i]) vmin[i]=v;
	if(v > vmax[i]) vmax[i]=v;
      }
      else {
	Vector v;
	Deriv(u,v);
	if(v[i] < vmin[i]) vmin[i]=v[i];
	if(v[i] > vmax[i]) vmax[i]=v[i];
      }
    }
    if(amin[i] > amax[i]) Swap(amin[i],amax[i]);
  }
}




double GeneralizedCubicBezierCurve::OuterLength() const
{
  if(space)
    return space->Distance(x0,x1)+space->Distance(x1,x2)+space->Distance(x2,x3);
  else
    return x0.distance(x1)+x1.distance(x2)+x2.distance(x3);
}

void GeneralizedCubicBezierCurve::Midpoint(Vector& x) const
{
  if(space) {
    Eval(0.5,x);
  }
  else {
    const static Real a1 = 1.0/8.0;
    const static Real a2 = 3.0/8.0;
    x.mul(x0,a1);
    x.madd(x1,a2);
    x.madd(x2,a2);
    x.madd(x3,a1);
  }
}

void GeneralizedCubicBezierCurve::MidpointDeriv(Vector& v) const
{
  if(space) Deriv(0.5,v);
  else {
    v.add(x2,x3);
    v -= x1;
    v -= x0;
    v *= 3.0/4.0;
  }
}


Real GeneralizedCubicBezierSpline::TotalTime() const
{
  Real t=0;
  for(size_t i=0;i<durations.size();i++)
    t += durations[i];
  return t;
}

void GeneralizedCubicBezierSpline::TimeScale(Real scale) 
{
  for(size_t i=0;i<durations.size();i++)
    durations[i] *= scale;
}

void GeneralizedCubicBezierSpline::Append(const GeneralizedCubicBezierCurve& seg,Real duration)
{
  segments.push_back(seg);
  durations.push_back(duration);
}

void GeneralizedCubicBezierSpline::Concat(const GeneralizedCubicBezierSpline& s)
{
  segments.insert(segments.end(),s.segments.begin(),s.segments.end());
  durations.insert(durations.end(),s.durations.begin(),s.durations.end());
}

int GeneralizedCubicBezierSpline::ParamToSegment(Real u,Real* segparam) const
{
  if(segments.empty()) return -1;
  if(durations.empty()) {
    Real k=Floor(u*segments.size());
    if(segparam) *segparam = u*segments.size()-k;
    return (int)k;
  }
  Assert(durations.size()==segments.size());
  int k=0;
  while(u > durations[k]) {
    u -= durations[k];
    k++;
  }
  if(k >= (int)durations.size()) {
    if(segparam) *segparam = 1.0;
    return segments.size()-1;
  }
  else {
    if(segparam) *segparam = Max(u/durations[k],0.0);
    return k;
  }
}

int GeneralizedCubicBezierSpline::Eval(Real u,Vector& x) const
{
  Real param;
  int seg = ParamToSegment(u,&param);
  segments[seg].Eval(param,x);
  return seg;
}

int GeneralizedCubicBezierSpline::Deriv(Real u,Vector& dx) const
{
  Real param;
  int seg = ParamToSegment(u,&param);
  segments[seg].Deriv(param,dx);
  dx /= durations[seg];
  return seg;
}

int GeneralizedCubicBezierSpline::Accel(Real u,Vector& ddx) const
{
  Real param;
  int seg = ParamToSegment(u,&param);
  segments[seg].Accel(param,ddx);
  ddx /= Sqr(durations[seg]);
  return seg;
}

void GeneralizedCubicBezierSpline::GetPiecewiseLinear(vector<Real>& times,vector<Config>& milestones) const
{
  Assert(!durations.empty());
  Assert(durations.size() == segments.size());
  times.resize(durations.size()+1);
  milestones.resize(durations.size()+1);
  times[0] = 0;
  milestones[0] = segments[0].x0;
  for(size_t i=0;i<durations.size();i++) {
    times[i+1] = times[i]+durations[i];
    milestones[i+1] = segments[i].x3; 
  }
}

bool GeneralizedCubicBezierSpline::Save(ostream& out) const
{
  Assert(durations.size() == segments.size());
  out<<durations.size()<<endl;
  for(size_t i=0;i<segments.size();i++)
    out<<durations[i]<<"\t"<<segments[i].x0<<"\t"<<segments[i].x1<<"\t"<<segments[i].x2<<"\t"<<segments[i].x3<<endl;
  return true;
}

bool GeneralizedCubicBezierSpline::Load(istream& in,Mintos::GeodesicManifold* space)
{
  int n;
  in>>n;
  if(!in || n < 0) return false;
  durations.resize(n);
  segments.resize(n);
  for(size_t i=0;i<segments.size();i++) {
    in >> durations[i];
    segments[i].space = space;
    in >> segments[i].x0  >> segments[i].x1 >> segments[i].x2 >> segments[i].x3;
    if(!in) return false;
  }
  return true;
}
