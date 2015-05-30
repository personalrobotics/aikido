#ifndef PARABOLIC_RAMP_MATH_H
#define PARABOLIC_RAMP_MATH_H

#include <math.h>
#include <stdlib.h>
#include <vector>

namespace ParabolicRamp {

typedef double Real;
typedef std::vector<double> Vector;

//can replace this with your favorite representation/tests of infinity
const static Real Inf = 1e300;
inline bool IsInf(Real x) { return x==Inf; }
inline bool IsFinite(Real x) { return fabs(x)<Inf; }

inline Real Sqr(Real x) { return x*x; }
inline Real Sqrt(Real x) { return sqrt(x); }
inline Real Abs(Real x) { return fabs(x); }
inline Real Sign(Real x) { return (x>0 ? 1 : (x<0 ? -1 : 0)); }
inline Real Min(Real x,Real y) { return (x<y?x:y); }
inline Real Max(Real x,Real y) { return (x>y?x:y); }
inline bool FuzzyZero(Real x,Real tol) { return Abs(x)<=tol; }
inline bool FuzzyEquals(Real x,Real y,Real tol) { return Abs(x-y)<=tol; }
inline void Swap(Real& x,Real& y) { Real temp=x; x=y; y=temp; }

// Returns a random number in [0,1)
inline Real Rand() { return Real(rand())/Real(RAND_MAX); }

} //namespace ParabolicRamp

#endif
