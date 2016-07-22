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

#ifndef MATH_MATH_H
#define MATH_MATH_H

#if defined(__APPLE__) || defined(MACOSX)
#include "/usr/include/math.h"
#else
#include <math.h>
#endif

#include <float.h>
#include <limits>
#include <mintos/misc/utils.h>  //for Max/Min definition
#include <mintos/Config.h>
#include "infnan.h"

/**
 * NEEDED ON SOLARIS
 */

#if defined (__SVR4) && defined (__sun)
#include <ieeefp.h>

static inline int isinf(double x) { return !finite(x) && x==x; }
static inline int isinff(double x) { return isinf(x); }
static inline int finitef(double x) { return finite(x); }

static inline float acosf (float x) { return acos(x); }
static inline float asinf (float x) { return asin(x); }
static inline float atanf (float x) { return atan(x); }
static inline float atan2f (float x, float y) { return atan2(x, y); }
static inline float ceilf (float x) { return ceil(x); }
static inline float cosf (float x) { return cos(x); }
static inline float coshf (float x) { return cosh(x); }
static inline float expf (float x) { return exp(x); }
static inline float fabsf (float x) { return fabs(x); }
static inline float fmodf (float x, float y) { return fmod(x, y); }
static inline float floorf (float x) { return floor (x); }
static inline float logf (float x) { return log (x); }
static inline float sinf (float x) { return sin (x); }
static inline float sinhf (float x) { return sinh (x); }
static inline float sqrtf (float x) { return sqrt (x); }
static inline float powf (float x, float y) { return pow (x,y); }
static inline float tanf (float x) { return tan (x); }
static inline float tanhf (float x) { return tanh(x); }
#endif

/**
 * END SOLARIS SECTION
 */

/** @defgroup Math 
 * @brief Definitions of mathematical concepts and numerical algorithms.
 */

/** @file math/math.h
 * @ingroup Math
 * @brief Common math typedefs, constants, functions
 *
 * Defines the typedef Real, and the capitalized std math functions
 * overloaded to handle both floats and doubles.
 *
 * Real is defined to be a double if MATH_DOUBLE is defined, otherwise
 * a float.
 */

/** @ingroup Math
 * @brief Contains all definitions in the Math package.
 */
namespace Math {

/** @addtogroup Math */
/*@{*/

const static double dZero = 0.0;
const static double dOne = 1.0;
const static double dTwo = 2.0;
const static double dHalf = 0.5;
const static double dE = 2.71828182845904523536028747135266;
const static double dLog2e = 1.4426950408889634073599246810018921; //log_2 e
const static double dLog10e = 0.4342944819032518276511289189166051;  //log_10 e
const static double dLn2 = 0.6931471805599453094172321214581766;  //log_e 2
const static double dLn10 = 2.3025850929940456840179914546843642;  //log_e 10 
const static double dPi = 3.1415926535897932384626433832795;
const static double dTwoPi = 6.2831853071795864769252867665590;
const static double dPi_2 = 1.5707963267948966192313216916397;
const static double dPi_4 = 0.7853981633974483096156608458198757;
const static double dSqrt2 = 1.4142135623730950488016887242096981;
const static double dSqrt1_2 = 0.7071067811865475244008443621048490;
const static double dEpsilon = 1e-8;
const static double dDtoRConst = 0.017453292519943295769236907684886;
const static double dRtoDConst = 57.295779513082320876798154814105;

const static float fZero = 0.0f;
const static float fOne = 1.0f;
const static float fTwo = 2.0f;
const static float fHalf = 0.5f;
const static float fE = 2.718281828f;
const static float fLog2e = 1.4426950408f; //log_2 e
const static float fLog10e = 0.4342944819f;  //log_10 e
const static float fLn2 = 0.69314718056f;  //log_e 2
const static float fLn10 = 2.302585092994f;  //log_e 10 
const static float fPi = 3.141592654f;
const static float fTwoPi = 6.283185307f;
const static float fPi_2 = 1.5707963267f;
const static float fPi_4 = 0.7853981634f;
const static float fSqrt2 = 1.4142135624f;
const static float fSqrt1_2 = 0.70710678119f;
const static float fEpsilon = 1e-5f;
const static float fDtoRConst = 0.0174532925f;
const static float fRtoDConst = 57.29577951f;

#ifdef MATH_DOUBLE

typedef double Real;

const static Real Zero = dZero;
const static Real One = dOne;
const static Real Two = dTwo;
const static Real Half = dHalf;
const static Real E = dE;
const static Real Log2e = dLog2e; //log_2 e
const static Real Log10e = dLog10e;  //log_10 e
const static Real Ln2 = dLn2;  //log_e 2
const static Real Ln10 = dLn10;  //log_e 10 
const static Real Pi = dPi;
const static Real TwoPi = dTwoPi;
const static Real Pi_2 = dPi_2;
const static Real Pi_4 = dPi_4;
const static Real Sqrt2 = dSqrt2;
const static Real Sqrt1_2 = dSqrt1_2;
const static Real Epsilon = dEpsilon;
const static Real DtoRConst = dDtoRConst;
const static Real RtoDConst = dRtoDConst;
const static Real Inf = dInf;

#else

typedef float Real;

const static Real Zero = fZero;
const static Real One = fOne;
const static Real Two = fTwo;
const static Real Half = fHalf;
const static Real E = fE;
const static Real Log2e = fLog2e; //log_2 e
const static Real Log10e = fLog10e;  //log_10 e
const static Real Ln2 = fLn2;  //log_e 2
const static Real Ln10 = fLn10;  //log_e 10 
const static Real Pi = fPi;
const static Real TwoPi = fTwoPi;
const static Real Pi_2 = fPi_2;
const static Real Pi_4 = fPi_4;
const static Real Sqrt2 = fSqrt2;
const static Real Sqrt1_2 = fSqrt1_2;
const static Real Epsilon = fEpsilon;
const static Real DtoRConst = fDtoRConst;
const static Real RtoDConst = fRtoDConst;
const static Real Inf = fInf;

#endif //MATH_DOUBLE

//math.h aliases
inline double Abs(double x) { return fabs(x); }
inline double Sqr(double x) { return x*x; }
inline double Sqrt(double x) { return sqrt(x); }
inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }
inline double Pow(double x, double y) { return pow(x,y); }
inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }
inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }
inline double Asin(double x) { return asin(x); }
inline double Acos(double x) { return acos(x); }
inline double Atan(double x) { return atan(x); }
inline double Atan2(double y, double x) { return atan2(y,x); }
inline double Floor(double x) { return floor(x); }
inline double Ceil(double x) { return ceil(x); }
inline double Mod(double x, double y) { return fmod(x,y); }

inline float Abs(float x) { return fabsf(x); }
inline float Sqr(float x) { return x*x; }
inline float Sqrt(float x) { return sqrtf(x); }
inline float Exp(float x) { return expf(x); }
inline float Log(float x) { return logf(x); }
inline float Pow(float x, float y) { return powf(x,y); }
inline float Sin(float x) { return sinf(x); }
inline float Cos(float x) { return cosf(x); }
inline float Tan(float x) { return tanf(x); }
inline float Sinh(float x) { return sinhf(x); }
inline float Cosh(float x) { return coshf(x); }
inline float Tanh(float x) { return tanhf(x); }
inline float Asin(float x) { return asinf(x); }
inline float Acos(float x) { return acosf(x); }
inline float Atan(float x) { return atanf(x); }
inline float Atan2(float y, float x) { return atan2f(y,x); }
inline float Floor(float x) { return floorf(x); }
inline float Ceil(float x) { return ceilf(x); }
inline float Mod(float x, float y) { return fmodf(x,y); }

/// 1/x
inline double Inv(double x) { return dOne/x; }
/// Returns {-1,0,1} if x is {<0,0,>0}
inline double Sign(double x) { return (x>0 ? dOne : (x<0 ? -dOne : 0)); }
/// Returns a if x < a, b if x > b, otherwise x
inline double Clamp(double x, double a, double b) { return (x<a? a : (x>b? b : x)); }
inline double Trunc(double x) { return (x>=0 ? Floor(x) : -Floor(-x)); }
inline double Frac(double x) { return x - Trunc(x); }

inline float Inv(float x) { return fOne/x; }
inline float Sign(float x) { return (x>0 ? fOne : (x<0 ? -fOne : 0)); }
inline float Clamp(float x, float a, float b) { return (x<a? a : (x>b? b : x)); }
inline float Trunc(float x) { return (x>=0 ? Floor(x) : -Floor(-x)); }
inline float Frac(float x) { return x - Trunc(x); }

/// Dot product of a scalar as a 1-d vector
inline double dot(double a, double b) { return a*b; }
/// Returns true if a and b are within +/- eps
inline bool FuzzyEquals(double a, double b, double eps=dEpsilon) { return Abs(a-b) <= eps; }
/// Returns true if a is zero within +/- eps
inline bool FuzzyZero(double a, double eps=dEpsilon) { return Abs(a) <= eps; }
/// Returns 1/x if x is nonzero, otherwise 0 
inline double PseudoInv(double x, double eps=dZero) { return (FuzzyZero(x,eps)?dZero:dOne/x); }
inline float dot(float a, float b) { return a*b; }
inline bool FuzzyEquals(float a, float b, float eps=fEpsilon) { return Abs(a-b) <= eps; }
inline bool FuzzyZero(float a, float eps=fEpsilon) { return Abs(a) <= eps; }
inline float PseudoInv(float x, float eps=fZero) { return (FuzzyZero(x,eps)?fZero:fOne/x); }

/// Kronecker delta
template <class T> inline Real Delta(T i, T j) { return (i==j? One : Zero); }
template <class T> inline Real Delta(T x) { return (x==0? One : Zero); }

/// Degree to radian conversion
inline double DtoR(double f) { return f*dDtoRConst; }
/// Radian to degree conversion
inline double RtoD(double f) { return f*dRtoDConst; }
inline float DtoR(float f) { return f*fDtoRConst; }
inline float RtoD(float f) { return f*fRtoDConst; }

/*@}*/
} //namespace Math

#endif
