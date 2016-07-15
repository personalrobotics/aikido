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

#include <mintos/TimeScaling.h>
#include "Timer.h"
#include <fstream>
using namespace Mintos;
using namespace Mintos::Spline;

int main(int argc,const char** argv)
{
  if(argc < 5) {
    printf("USAGE: timeopt spline limits gridRes dt\n");
    printf("  Optimizes the time scaling of the cubic spline 'spline' under\n");
    printf("  velocity/acceleration limits in the file 'limits'.\n");
    printf("  The time-scaling domain is split into gridRes points.\n");
    printf("  The output trajectory  is a list of time/milestone pairs\n");
    printf("  discretized at timestep dt.\n");
    return 0;
  }
  TimeScaledBezierCurve output;
  {
    ifstream in(argv[1],ios::in);
    if(!output.path.Load(in)) {
      printf("Unable to load spline file %s\n",argv[1]);
      return 1;
    }
  }
  Vector vmin,vmax,amin,amax;
  {
    ifstream in(argv[2],ios::in);
    if(!in) {
      printf("Unable to load limits file %s\n",argv[2]);
      return 1;
    }
    in >> vmin >> vmax >> amin >> amax;
    if(!in) {
      printf("Error loading limits file %s\n",argv[2]);
      return 1;
    }
  }

  for(size_t i=0;i<output.path.segments.size();i++) {
    if(output.path.segments[i].x0.n != output.path.segments[0].x0.n) {
      printf("Invalid milestone size on segment %d: %d vs %d\n",i,output.path.segments[i].x0.n,output.path.segments[0].x0.n);
      return 1;
    }
    if(output.path.segments[i].x1.n != output.path.segments[0].x0.n) {
      printf("Invalid milestone size on segment %d: %d vs %d\n",i,output.path.segments[i].x1.n,output.path.segments[0].x0.n);
      return 1;
    }
    if(output.path.segments[i].x2.n != output.path.segments[0].x0.n) {
      printf("Invalid milestone size on segment %d: %d vs %d\n",i,output.path.segments[i].x2.n,output.path.segments[0].x0.n);
      return 1;
    }
    if(output.path.segments[i].x3.n != output.path.segments[0].x0.n) {
      printf("Invalid milestone size on segment %d: %d vs %d\n",i,output.path.segments[i].x3.n,output.path.segments[0].x0.n);
      return 1;
    }
  }
  if(vmin.n != output.path.segments[0].x0.n) {
    printf("Invalid length of limits: %d vs %d\n",vmin.n,output.path.segments[0].x0.n);
    return 1;
  }
  int gridRes;
  Real dt;
  gridRes = atoi(argv[3]);
  dt = atof(argv[4]);

  if((int)output.path.segments.size() < gridRes) {
    GeneralizedCubicBezierSpline path;
    path.segments.reserve(gridRes+output.path.segments.size());
    path.durations.reserve(gridRes+output.path.segments.size());
    Config xp,vp,xn,vn;
    Real T = output.path.TotalTime();
    for(size_t i=0;i<output.path.segments.size();i++) {
      int n = (int)Ceil(gridRes*(output.path.durations[i]/T));
      if(n <= 0) {
	printf("Error: curve segment %d has nonpositive duration?\n",i);
	return 1;
      }

      //split segment[i] into n pieces
      Real divScale = 1.0/Real(n);
      xp = output.path.segments[i].x0;
      output.path.segments[i].Deriv(0,vp);
      for(int j=0;j<n;j++) {
	Real u2=Real(j+1)/Real(n);
	output.path.segments[i].Eval(u2,xn);
	output.path.segments[i].Deriv(u2,vn);
	path.segments.resize(path.segments.size()+1);
	path.durations.push_back(divScale*output.path.durations[i]);
	path.segments.back().x0 = xp;
	path.segments.back().x3 = xn;
	path.segments.back().SetNaturalTangents(vp*divScale,vn*divScale);
	swap(xp,xn);
	swap(vp,vn);
      }
    }
    output.path = path;
  }

  Timer timer;
  if(!output.OptimizeTimeScaling(vmin,vmax,amin,amax)) {
    printf("Error: OptimizeTimeScaling failed\n");
    return 1;
  }
  Real elapsedTime = timer.ElapsedTime();

  printf("Optimized duration %g, time %g\n",output.EndTime(),elapsedTime);
  vector<Vector> milestones;
  output.GetDiscretizedPath(dt,milestones);
  for(size_t i=0;i<milestones.size();i++)
    cout<<dt*i<<"\t"<<milestones[i]<<endl;;

  printf("Saving time scaling to timeopt_plot.csv\n");
  output.Plot("timeopt_plot.csv",vmin,vmax,amin,amax,1.0/Real(gridRes));
  return 0;
}
