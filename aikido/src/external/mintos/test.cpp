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
#include <mintos/TimeScaling.h>
#include "Timer.h"
#include "src/primitives.h"
using namespace Mintos;
using namespace Mintos::Spline;
using namespace Math3D;

///if you want to print the time scaling to a CSV format, set this to 1
#define PRINT_TIME_SCALING 0

Real Rand()
{
  return Real(rand())/RAND_MAX;
}

void RunConstrainedInterpolateTest(const Config& a,const Config& b,GeodesicManifold* space,VectorFieldFunction* constraint)
{
  ConstrainedInterpolator interp(space,constraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  SmoothConstrainedInterpolator interp2(space,constraint);
  interp2.ftol = interp.ftol;
  interp2.xtol = interp.xtol;
  interp2.maxNewtonIters = interp.maxNewtonIters;

  //run tests
  cout<<"Interpolating "<<a<<" -> "<<b<<endl;
  double xtols[10] = {1e-3,2.5e-3,5e-3,1e-2,2.5e-2,5e-2,1e-1,2.5e-1,5e-1,1};
  const static int N=10;

  cout<<"Descent interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error,smoothed error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp.xtol = xtols[iter];

    cout<<xtols[iter]<<",";
    int numTrials = 10;
    bool res;
    vector<Config> path;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      path.resize(0);
      res=true;
      path.push_back(a);
      Vector temp,dir;
      while(true) {
	dir=b-path.back();
	if(dir.norm() < interp.xtol) {
	  path.push_back(b);
	  break;
	}
	interp2.ProjectVelocity(path.back(),dir);
	Real n = dir.norm();
	if(n < 1e-7) {
	  res = false;
	  break;
	}
	dir *= interp.xtol / n;
	temp = path.back() + dir;
	interp2.Project(temp);
	path.push_back(temp);
      }
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<path.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      Real maxerrsmooth = 0.0;
      Vector val;
      Vector x;
      for(size_t i=0;i<path.size();i++) {
	(*interp.constraint)(path[i],val);
	maxerr = Max(maxerr,val.norm());
	if(i+1 < path.size()) {
	  int numdivs = Max(10000/(int)path.size(),2);
	  for(int j=1;j<numdivs;j++) {
	    space->Interpolate(path[i],path[i+1],Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrmid = Max(maxerrmid,val.norm());
	  }

	  //do smoothed
	  GeneralizedCubicBezierCurve curve(space);
	  curve.x0 = path[i];
	  curve.x3 = path[i+1];
	  if(i == 0) curve.x1 = curve.x0 + (path[i+1]-path[i])/3.0;
	  else curve.x1 = curve.x0 + 0.5*(path[i+1]-path[i-1])/3.0;
	  if(i+2 >= path.size()) curve.x2 = curve.x3 - (path[i+1]-path[i])/3.0;
	  else curve.x2 = curve.x3 - 0.5*(path[i+2]-path[i])/3.0;
	  for(int j=1;j<numdivs;j++) {
	    curve.Eval(Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrsmooth = Max(maxerrsmooth,val.norm());
	  }
	}
      }
      cout<<maxerr<<","<<maxerrmid<<","<<maxerrsmooth<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }
  

  cout<<"Linear interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error,smoothed error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp.xtol = xtols[iter];

    cout<<xtols[iter]<<",";
    int numTrials = 10;
    bool res;
    vector<Config> path;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      path.resize(0);
      res=interp.Make(a,b,path);
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<path.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      Real maxerrsmooth = 0.0;
      Vector val;
      Vector x;
      for(size_t i=0;i<path.size();i++) {
	(*interp.constraint)(path[i],val);
	maxerr = Max(maxerr,val.norm());
	if(i+1 < path.size()) {
	  int numdivs = Max(10000/(int)path.size(),2);
	  for(int j=1;j<numdivs;j++) {
	    space->Interpolate(path[i],path[i+1],Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrmid = Max(maxerrmid,val.norm());
	  }

	  //do smoothed
	  GeneralizedCubicBezierCurve curve(space);
	  curve.x0 = path[i];
	  curve.x3 = path[i+1];
	  if(i == 0) curve.x1 = curve.x0 + (path[i+1]-path[i])/3.0;
	  else curve.x1 = curve.x0 + 0.5*(path[i+1]-path[i-1])/3.0;
	  if(i+2 >= path.size()) curve.x2 = curve.x3 - (path[i+1]-path[i])/3.0;
	  else curve.x2 = curve.x3 - 0.5*(path[i+2]-path[i])/3.0;
	  for(int j=1;j<numdivs;j++) {
	    curve.Eval(Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrsmooth = Max(maxerrsmooth,val.norm());
	  }
	}
      }
      cout<<maxerr<<","<<maxerrmid<<","<<maxerrsmooth<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }

  cout<<"Smooth interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp2.xtol = xtols[iter];
    cout<<xtols[iter]<<",";

    GeneralizedCubicBezierSpline cpath;
    int numTrials = 10;
    bool res;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      cpath.segments.clear();
      cpath.durations.clear();
      res=interp2.Make(a,b,cpath);
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<cpath.segments.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      for(size_t i=0;i<cpath.segments.size();i++) {
	if(i > 0) Assert(cpath.segments[i].x0 == cpath.segments[i-1].x3);
	
	Vector val;
	(*interp.constraint)(cpath.segments[i].x0,val);
	maxerr = Max(maxerr,val.norm());
	(*interp.constraint)(cpath.segments[i].x3,val);
	maxerr = Max(maxerr,val.norm());
	
	Vector x;
	int numdivs = Max(10000/(int)cpath.segments.size(),2);
	for(int j=1;j<numdivs;j++) {
	  cpath.segments[i].Eval(Real(j)/Real(numdivs),x);
	  (*interp.constraint)(x,val);
	  maxerrmid = Max(maxerrmid,val.norm());
	}
      }
      cout<<maxerr<<","<<maxerrmid<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }
}


class CartesianCSpace : public GeodesicManifold
{
public:
  CartesianCSpace(int _n) : n(_n) {}
  int n;
};

class SphereConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    fx[0] = x.normSquared()-1;
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji = x;
    Ji *= 2;
    J.copyRow(0,Ji);
  }
};

class MultiSphereConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    Real xmod = Mod(x[0]+1.5,3.0)-1.5;
    fx[0] = x.normSquared()-Sqr(x[0])+Sqr(xmod)-1;
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji = x;
    Ji(0) = Mod(x[0]+1.5,3.0)-1.5;
    Ji *= 2;
    J.copyRow(0,Ji);
  }
};

class SineWaveConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    fx[0] = x[1] - Sin(x[0]);
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    J(0,0) = -Cos(x[0]);
    J(0,1) = 1.0;
  }
};


class TorusConstraint : public VectorFieldFunction
{
public:
  Real primaryRadius,secondaryRadius;
  TorusConstraint(Real _primaryRadius=1.0,Real _secondaryRadius=0.25)
    :primaryRadius(_primaryRadius),secondaryRadius(_secondaryRadius)
  {}
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    Assert(x.n >= 3);
    Real len = Sqrt(Sqr(x[1])+Sqr(x[0]));
    Real xc = x[0]*primaryRadius/len;
    Real yc = x[1]*primaryRadius/len;
    if(len == 0) { xc = primaryRadius; yc=0; }
    Real d2 = Sqr(x[0]-xc) + Sqr(x[1]-yc) + Sqr(x[2]);
    fx.resize(1);
    fx[0] = d2 - Sqr(secondaryRadius);
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji(3);
    Real len = Sqrt(Sqr(x[1])+Sqr(x[0]));
    if(FuzzyZero(len)) Ji.setZero();
    else {
      Real xc = x[0]*primaryRadius/len;
      Real yc = x[1]*primaryRadius/len;
      Real dlend0 = x[0]/len;
      Real dlend1 = x[1]/len;
      //derivative of xc w.r.t. x[0], x[1]
      Real dxcd0 = (len*primaryRadius - x[0]*primaryRadius*dlend0)/Sqr(len);
      Real dxcd1 = -x[0]*primaryRadius*dlend1/Sqr(len);
      //derivative of yc w.r.t. x[0], x[1]
      Real dycd0 = -x[1]*primaryRadius*dlend0/Sqr(len);
      Real dycd1 = (len*primaryRadius - x[1]*primaryRadius*dlend1)/Sqr(len);
      Ji[0] = 2.0*(x[0]-xc)*(1-dxcd0) - 2.0*(x[1]-yc)*dycd0;
      Ji[1] = -2.0*(x[0]-xc)*dxcd1 + 2.0*(x[1]-yc)*(1-dycd1);
      Ji[2] = 2*x[2];
    }
    J.copyRow(0,Ji);

    /*
    //TEMP: debugging
    Matrix Jdiff(1,3);
    Vector temp=x;
    JacobianCenteredDifference(*this,temp,0.001,Jdiff);
    if(!Jdiff.isEqual(J,1e-2)) {
      cout<<"Jacobian error at "<<x<<endl;
      cout<<Ji<<endl;
      cout<<Jdiff<<endl;
    }
    Assert(Jdiff.isEqual(J,1e-2));
    */
  }

  void Enumerate(Real resolution,vector<Vector>& grid)
  {
    int udivs = (int)Ceil(secondaryRadius*TwoPi/resolution);
    int thetadivs = (int)Ceil((primaryRadius+secondaryRadius)*TwoPi/resolution);
    Vector res(3);
    for(int i=0;i<thetadivs;i++) {
      Real theta=Real(i)/thetadivs*TwoPi;
      Matrix2 R; R.setRotate(theta);
      for(int j=0;j<udivs;j++) {
	Real u=Real(j)/udivs*TwoPi;
	Real sx=Cos(u)*secondaryRadius+primaryRadius,z=Sin(u)*secondaryRadius;
	res[0] = R(0,0)*sx;
	res[1] = R(1,0)*sx;
	res[2] = z;

	//TEMP: test
	Vector v;
	Eval(res,v);
	Assert(FuzzyZero(v[0]));
	grid.push_back(res);
      }
    }
  }

  void Sample(Vector& res)
  {
    res.resize(3);
    Real u=Rand()*TwoPi;
    Real sx=Cos(u)*secondaryRadius+primaryRadius,z=Sin(u)*secondaryRadius;
    Real theta=Rand()*TwoPi;
    Matrix2 R; R.setRotate(theta);
    res[0] = R(0,0)*sx;
    res[1] = R(1,0)*sx;
    res[2] = z;

    //TEMP: test
    Vector v;
    Eval(res,v);
    Assert(FuzzyZero(v[0]));
  }
};

void ConstrainedInterpolateTest()
{
  {
    CartesianCSpace space2(2);
    SineWaveConstraint sineWaveConstraint;
    Vector a(2),b(2);
    a(0) = 0; a(1) = 0; 
    b(0) = TwoPi*2; b(1) = 0;

    cout<<endl;
    cout<<"***Sine wave test***"<<endl;
    RunConstrainedInterpolateTest(a,b,&space2,&sineWaveConstraint);


    ConstrainedInterpolator interp(&space2,&sineWaveConstraint);
    interp.ftol = 1e-8;
    interp.xtol = 4;
    interp.maxNewtonIters = 100;

    SmoothConstrainedInterpolator interp2(&space2,&sineWaveConstraint);
    interp2.ftol = interp.ftol;
    interp2.xtol = interp.xtol;
    interp2.maxNewtonIters = interp.maxNewtonIters;

    vector<Config> path;
    bool res=interp.Make(a,b,path);
    cout<<path.size()<<" milestones"<<endl;
    cout<<"x,y"<<endl;
    for(size_t i=0;i<path.size();i++)
      cout<<path[i][0]<<","<<path[i][1]<<endl;
    cout<<endl;
    GeneralizedCubicBezierSpline cpath;
    res=interp2.Make(a,b,cpath);
    cout<<cpath.segments.size()<<" curves"<<endl;
    cout<<"x,y"<<endl;
    double dt=1e-3;
    Vector temp;
    for(size_t i=0;i<cpath.segments.size();i++) {
      double t=0;
      while(t < cpath.durations[i]) {
	cpath.segments[i].Eval(t/cpath.durations[i],temp);
	cout<<temp[0]<<","<<temp[1]<<endl;
	t += dt;
      }
      cpath.segments[i].Eval(1,temp);
      cout<<temp[0]<<","<<temp[1]<<endl;
    }
    cout<<endl;
    return;
  }

  CartesianCSpace space(3);
  SphereConstraint sphereConstraint;
  TorusConstraint torusConstraint;
  Vector a(3),b(3);
  //interpolate on sphere
  cout<<endl;
  cout<<"***Sphere test***"<<endl;
  a(0) = 1; a(1) = 0; a(2) = 0;
  b(0) = 0; b(1) = 1; b(2) = 0;
  //b(0) = Cos(Pi*9.0/10.0); b(1) = Sin(Pi*9.0/10.0); b(2) = 0;
  RunConstrainedInterpolateTest(a,b,&space,&sphereConstraint);

  //interpolate on torus
  cout<<endl;
  cout<<"***Torus test***"<<endl;
  a(0) = 1.25; a(1) = 0; a(2) = 0;
  b(0) = 0; b(1) = 1; b(2) = 0.25;
  RunConstrainedInterpolateTest(a,b,&space,&torusConstraint);


  //this should fail
  MultiSphereConstraint multiSphereConstraint;
  a(0) = 1; a(1) = 0; a(2) = 0;
  b(0) = 3; b(1) = 1; b(2) = 0;
  //printf("Running fail constrained interpolate test...\n");
  //RunConstrainedInterpolateTest(a,b,&space,&multiSphereConstraint);

  cout<<endl;
  cout<<"***All-torus test***"<<endl;
  //test all points on torus
  vector<Vector> pts;
  //torusConstraint.Enumerate(0.05,pts);
  torusConstraint.Enumerate(0.01,pts);
  //a(0) = 1.25; a(1) = 0; a(2) = 0;
  a(0) = 1.0; a(1) = 0.0; a(2) = 0.25;
  cout<<"Source: "<<a(0)<<","<<a(1)<<","<<a(2)<<endl;
  SmoothConstrainedInterpolator interp(&space,&torusConstraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  for(size_t i=0;i<pts.size();i++) {
    GeneralizedCubicBezierSpline cpath;
    bool res=interp.Make(a,pts[i],cpath);
    cout<<pts[i][0]<<","<<pts[i][1]<<","<<pts[i][2]<<","<<res<<endl;
  }
}


void TimeOptimizeTest1DLine()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //straight line path from 0 to 3
    for(int j=0;j<numSegments;j++) {
      vs[j] = Vector(1,3.0);
      vmins[j] = Vector(1,3.0);
      vmaxs[j] = Vector(1,3.0);
      amins[j] = Vector(1,0.0);
      amaxs[j] = Vector(1,0.0);
    }
    vs.back() = Vector(1,3.0);
    Timer timer;
    TimeScaling timeScaling;
    //timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
      for(size_t j=0;j<timeScaling.ds.size();j++) 
	Assert(vmin[0] <= timeScaling.ds[j] && timeScaling.ds[j] <= vmax[0]);
    }
  }
}

void TimeOptimizeTest1DSine()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024,2028};
  for(int i=0;i<11;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //sine wave
    //p(s) = sin(s*2pi/period)
    //p'(s) = 2pi/period*cos(s*2pi/period)
    //p''(s) = -(2pi/period)^2*sin(s*2pi/period)
    Real period = 1.0;
    for(int j=0;j<numSegments;j++) {
      Real umin = Real(j)/numSegments;
      Real umax = Real(j+1)/numSegments;
      Real xmin = umin*TwoPi/period;
      Real xmax = umax*TwoPi/period;
      Real vmin=Cos(xmin),vmax=Cos(xmax);
      Real amin=-Sin(xmin),amax=-Sin(xmax);
      if(vmin > vmax) Swap(vmin,vmax);
      if(amin > amax) Swap(amin,amax);
      if(xmax > TwoPi && xmin < TwoPi)  vmax=1;
      if(xmax > 2*TwoPi && xmin < 2*TwoPi)  vmax=1;
      if(xmax > Pi && xmin < Pi)  vmin=-1;
      if(xmax > 3*Pi && xmin < 3*Pi)  vmin=-1;
      if(xmax > Pi/2 && xmin < Pi/2)  amin=-1;
      if(xmax > Pi/2+TwoPi && xmin < Pi/2+TwoPi)  amin=-1;
      if(xmax > 3*Pi/2 && xmin < 3*Pi/2)  amax=1;
      if(xmax > 3*Pi/2+TwoPi && xmin < 3*Pi/2+TwoPi)  amax=1;
      Assert(vmax - vmin <= 1.0/numSegments*TwoPi);
      Assert(amax - amin <= 1.0/numSegments*TwoPi/period);
      Assert(vmin >= -1.0 && vmax <= 1.0);
      Assert(amin >= -1.0 && amax <= 1.0);
      Assert(vmin <= vmax);
      Assert(amin <= amax);
      vmax *= TwoPi/period;
      vmin *= TwoPi/period;
      amax *= Sqr(TwoPi/period);
      amin *= Sqr(TwoPi/period);
      vs[j] = Vector(1,TwoPi/period*Cos(xmin));
      if(j+1==numSegments)
	vs[j+1] = Vector(1,TwoPi/period*Cos(xmax));
      vmins[j] = Vector(1,vmin);
      vmaxs[j] = Vector(1,vmax);
      amins[j] = Vector(1,amin);
      amaxs[j] = Vector(1,amax);
    }
    Timer timer;
    TimeScaling timeScaling;
    //timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    //bool res=timeScaling.SolveMinTimeArcLength(vmin,vmax,amin,amax,divs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
    }

#if PRINT_TIME_SCALING
    if(i+1==11) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<Sin(timeScaling.params[i]*TwoPi/period);
	if(i<vmins.size()) {
	  Real dp=Cos(timeScaling.params[i]*TwoPi/period)*TwoPi/period;
	  Real ddp=-Sin(timeScaling.params[i]*TwoPi/period)*Sqr(TwoPi/period);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
#endif // PRINT_TIME_SCALING
  }
}


void TimeOptimizeTest1DParabolic()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from -1 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++) {
      divs[j] = Real(j)/Real(numSegments);
      vs[j] = Vector(1,8.0*Abs(divs[j]-0.5));
      if(j > 0) {
	vmins[j-1] = vs[j-1];
	vmaxs[j-1] = vs[j];
	if(divs[j] < 0.5)
	  Swap(vmins[j-1],vmaxs[j-1]);
	amins[j-1] = Vector(1,8.0*Sign(divs[j-1]-0.5));
	amaxs[j-1] = Vector(1,8.0*Sign(divs[j-1]-0.5));
      }
    }
    Timer timer;
    TimeScaling timeScaling;
    //timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTimeArcLength(vmin,vmax,amin,amax,divs,vs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
      /*
      for(size_t j=0;j<timeScaling.ds.size();j++) 
	Assert(vmin[0] <= timeScaling.ds[j]*vs[j][0] && timeScaling.ds[j]*vs[j][0] <= vmax[0]);
      */
    }
#if PRINT_TIME_SCALING
    if(i+1==10) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<4.0*Pow(timeScaling.params[i]-0.5,2.0)*Sign(timeScaling.params[i]-0.5);
	if(i<vmins.size()) {
	  Real dp=8.0*Abs(timeScaling.params[i]-0.5);
	  Real ddp=8.0*Sign(timeScaling.params[i]-0.5);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
#endif // PRINT_TIME_SCALING
  }
}

void TimeOptimizeTest1DCubic()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from -1 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++) {
      divs[j] = Real(j)/Real(numSegments);
      vs[j] = Vector(1,3.0*8.0*(divs[j]-0.5)*(divs[j]-0.5));
      if(j > 0) {
	vmins[j-1] = vs[j-1];
	vmaxs[j-1] = vs[j];
	amins[j-1] = Vector(1,6.0*8.0*(divs[j-1]-0.5));
	amaxs[j-1] = Vector(1,6.0*8.0*(divs[j]-0.5));
      }
    }
    Timer timer;
    TimeScaling timeScaling;
    //timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
      for(size_t j=0;j<timeScaling.ds.size();j++) 
	Assert(vmin[0] <= timeScaling.ds[j]*vs[j][0] && timeScaling.ds[j]*vs[j][0] <= vmax[0]);
    }
#if PRINT_TIME_SCALING
    if(i+1==10) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<8.0*Pow(timeScaling.params[i]-0.5,3.0);
	if(i<vmins.size()) {
	  Real dp=24.0*Sqr(timeScaling.params[i]-0.5);
	  Real ddp=48.0*(timeScaling.params[i]-0.5);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
#endif //PRINT_TIME_SCALING
  }
}


void TimeOptimizeTest2DCircle()
{
  Vector vmin(2,-1.0),vmax(2,1.0);
  Vector amin(2,-1.0),amax(2,1.0);
  //path param goes from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //circles
    //p(s) = (sin(s*2pi/period),cos(s*2pi/period)
    //p'(s) = 2pi/period*(cos(s*2pi/period),-sin(s*2pi/period))
    //p''(s) = (2pi/period)^2*(-sin(s*2pi/period),-cos(s*2pi/period))
    Real period = 1.0;
    for(int j=0;j<numSegments;j++) {
      Real umin = Real(j)/numSegments;
      Real umax = Real(j+1)/numSegments;
      Real xmin = umin*TwoPi/period;
      Real xmax = umax*TwoPi/period;
      Vector2 vmin(Cos(xmin),-Sin(xmin)),vmax(Cos(xmax),-Sin(xmax));
      Vector2 amin(-Sin(xmin),-Cos(xmin)),amax(-Sin(xmax),-Cos(xmax));
      if(vmin.x > vmax.x) Swap(vmin.x,vmax.x);
      if(vmin.y > vmax.y) Swap(vmin.y,vmax.y);
      if(amin.x > amax.x) Swap(amin.x,amax.x);
      if(amin.y > amax.y) Swap(amin.y,amax.y);
      if(xmax > TwoPi && xmin < TwoPi) { vmax.x=1; amin.y=-1; }
      if(xmax > 2*TwoPi && xmin < 2*TwoPi) { vmax.x=1; amin.y=-1; }
      if(xmax > Pi && xmin < Pi) { vmin.x=-1; amax.y=1; }
      if(xmax > 3*Pi && xmin < 3*Pi) { vmin.x=-1; amax.y=1; }
      if(xmax > Pi/2 && xmin < Pi/2)  { vmin.y=-1; amin.x=-1; }
      if(xmax > Pi/2+TwoPi && xmin < Pi/2+TwoPi) { vmin.y=-1; amin.x=-1; }
      if(xmax > 3*Pi/2 && xmin < 3*Pi/2) { vmax.y=1; amax.x=1; }
      if(xmax > 3*Pi/2+TwoPi && xmin < 3*Pi/2+TwoPi) { vmax.y=1; amax.x=1; }
      Assert(vmin.x >= -1.0 && vmax.x <= 1.0);
      Assert(vmin.y >= -1.0 && vmax.y <= 1.0);
      Assert(amin.x >= -1.0 && amax.x <= 1.0);
      Assert(amin.y >= -1.0 && amax.y <= 1.0);
      Assert(vmin.x <= vmax.x);
      Assert(vmin.y <= vmax.y);
      Assert(amin.x <= amax.x);
      Assert(amin.y <= amax.y);
      vmax *= TwoPi/period;
      vmin *= TwoPi/period;
      amax *= Sqr(TwoPi/period);
      amin *= Sqr(TwoPi/period);
      vmins[j] = Vector(2,vmin);
      vmaxs[j] = Vector(2,vmax);
      amins[j] = Vector(2,amin);
      amaxs[j] = Vector(2,amax);
      vs[j] = Vector(2,Vector2(TwoPi/period*Cos(xmin),-TwoPi/period*Sin(xmin)));
      if(j+1==numSegments)
	vs[j+1] = Vector(2,Vector2(TwoPi/period*Cos(xmax),-TwoPi/period*Sin(xmax)));
    }
    Timer timer;
    TimeScaling timeScaling;
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    printf("End time: %g\n",timeScaling.times.back());
    printf("Solution time: %g s\n",time);

#if PRINT_TIME_SCALING
    if(i+1==10) {
      cout<<"t,s,ds,x,y:"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++)
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<Sin(timeScaling.params[i]*TwoPi/period)<<","<<Cos(timeScaling.params[i]*TwoPi/period)<<endl;
    }
#endif
  }
}

void TimeOptimizeTest2DCircleBezier()
{
  CartesianCSpace space(2);
  SphereConstraint sphereConstraint;
  SmoothConstrainedInterpolator interp(&space,&sphereConstraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  vector<Vector> pts;
  pts.resize(5);
  pts[0].resize(2);
  pts[1].resize(2);
  pts[2].resize(2);
  pts[3].resize(2);
  pts[4].resize(2);
  pts[0](0)=1;    pts[0](1)=0;
  pts[1](0)=0;    pts[1](1)=1;
  pts[2](0)=-1;    pts[2](1)=0;
  pts[3](0)=0;    pts[3](1)=-1;
  pts[4](0)=1;    pts[4](1)=0;
  Timer timer;
  TimeScaledBezierCurve curve;
  MultiSmoothInterpolate(interp,pts,curve.path);
  for(size_t i=0;i<curve.path.segments.size();i++)
    Assert(curve.path.segments[i].space == &space);

  printf("Num segments: %d\n",curve.path.segments.size());
  Vector vmin(2,-1.0),vmax(2,1.0);
  Vector amin(2,-1.0),amax(2,1.0);
  bool res=curve.OptimizeTimeScaling(vmin,vmax,amin,amax);
  if(!res) {
    printf("Error optimizing path\n");
    return;
  }
  printf("End time: %g\n",curve.EndTime());
  printf("Solution time: %g s\n",timer.ElapsedTime());
  Assert(curve.timeScaling.times.size()==curve.timeScaling.params.size());
  
#if PRINT_TIME_SCALING
  cout<<"t,s,ds,x,y,dx,dy:"<<endl;
  for(size_t i=0;i<curve.timeScaling.times.size();i++) {
    Vector x,v;
    curve.Eval(curve.timeScaling.times[i],x);
    curve.Deriv(curve.timeScaling.times[i],v);
    cout<<curve.timeScaling.times[i]<<","<<curve.timeScaling.params[i]<<","<<curve.timeScaling.ds[i]<<","<<x[0]<<","<<x[1]<<","<<v[0]<<","<<v[1]<<endl;
  }
#endif
}


void TimeOptimizeTest()
{
  printf("***** Optimizing 1D line *****\n");
  TimeOptimizeTest1DLine();
  printf("***** Optimizing 1D sine *****\n");
  TimeOptimizeTest1DSine();
  printf("***** Optimizing 2D circle *****\n");
  TimeOptimizeTest2DCircle();
  printf("***** Optimizing 1D cubic *****\n");
  TimeOptimizeTest1DCubic();
  printf("***** Optimizing 1D parabolic *****\n");
  TimeOptimizeTest1DParabolic();
  printf("***** Optimizing 1D circle bezier *****\n");
  TimeOptimizeTest2DCircleBezier();
}


int main(int argc, char** argv)
{
  ConstrainedInterpolateTest();
  TimeOptimizeTest();
  return 0;
}
