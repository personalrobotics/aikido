#include <mintos/Mintos.h>
#include <mintos/spline/SplineInterpolate.h>

namespace Mintos { 

bool InterpolateAndTimeOptimize(const Config& a,const Config& b,
				GeodesicManifold* space,
				VectorFieldFunction* constraint,
				Real constraintTolerance,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output)
{
  vector<Vector> configs(2);
  configs[0] = a;
  configs[1] = b;
  return InterpolateAndTimeOptimize(configs,space,constraint,constraintTolerance,vmin,vmax,amin,amax,output);
}

bool InterpolateAndTimeOptimize(const vector<Config>& configs,
				GeodesicManifold* space,
				VectorFieldFunction* constraint,
				Real constraintTolerance,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output)
{
  if(constraint) {
    SmoothConstrainedInterpolator interp(space,constraint);
    interp.xtol = constraintTolerance;
    if(!MultiSmoothInterpolate(interp,configs,output.path)) return false;
  }
  else {
    GeneralizedCubicBezierSpline path;
    SplineInterpolate(configs,path.segments,space);
    path.durations.resize(path.segments.size());
    for(size_t i=0;i+1<path.segments.size();i++)
      path.durations[i] = 1.0/path.durations.size();
    //treat constraintTolerance as a resolution, discretize the curves into
    //n pieces
    int n=(int)Ceil(1.0/constraintTolerance);
    output.path.segments.resize(n);
    output.path.durations.resize(n);
    Config xp,vp,xn,vn;
    path.Eval(0,xp);
    path.Deriv(0,vp);
    for(int i=0;i<n;i++) {
      Real u2=Real(i+1)/Real(n);
      path.Eval(u2,xn);
      path.Deriv(u2,vn);
      output.path.segments[i].x0 = xp;
      output.path.segments[i].x3 = xn;
      output.path.segments[i].SetNaturalTangents(vp,vn);
      output.path.durations[i] = 1.0/Real(n);
    }
    swap(xp,xn);
    swap(vp,vn);
  }
  if(!output.OptimizeTimeScaling(vmin,vmax,amin,amax)) return false;
  return true;
}

bool InterpolateAndTimeOptimize(const vector<Config>& configs,
				Real timingResolution,
				const Vector& vmin,const Vector& vmax,
				const Vector& amin,const Vector& amax,
				TimeScaledBezierCurve& output)
{
  return InterpolateAndTimeOptimize(configs,NULL,NULL,
				    timingResolution,
				    vmin,vmax,amin,amax,output);
}


} //namespace Mintos
