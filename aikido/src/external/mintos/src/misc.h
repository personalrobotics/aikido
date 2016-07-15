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

#ifndef MATH_MISC_H
#define MATH_MISC_H

#include <mintos/math/math.h>

/** @file misc.h
 * @ingroup Math
 * @brief Miscellaneous math functions.
 */

namespace Math {

  /** @addtogroup Math */
  /*@{*/

/// Computes up to 2 soln's to quadratic equation a^2 x + b x + c = 0.
/// Returns the number of solutions (0, 1, or 2)
int quadratic(double a, double b, double c, double& x1, double& x2);
/// Computes up to 3 soln's to quadratic equation a^3 x + b^2 x + c x + d = 0.
/// Returns the number of solutions (0 to 3)
int cubic(double a, double b, double c, double d, double x[3]);
/// Returns c where a^2+b^2=c^2
double pythag(double a, double b);
/// Returns b where a^2+b^2=c^2
double pythag_leg(double a,double c);
/// Computes the sinc function sin(x)/x (stable for small x)
double Sinc(double x);
/// Computes the derivative of the sinc function
double Sinc_Dx(double x);
/// Computes the factorial of n, with double precision
double dFactorial(unsigned int n);
/// Computes the log of the factorial of n
double dLogFactorial(unsigned int n);
/// Computes n choose k in double precision
double dChoose(unsigned int n,unsigned int k);
/// Computes the log of n choose k
double dLogChoose(unsigned int n,unsigned int k);
/// Computes the taylor series coefficient x^n/n!
double TaylorCoeff(double x,unsigned int n);
/// Computes the gamma function
double Gamma(double x);
/// Computes log gamma(x)
double LogGamma(double x);
/// Computes 1 / gamma(x)
double GammaInv(double x);
/// Computes the beta function B(a,b)
double Beta(double a,double b);
/// Computes log B(a,b)
double LogBeta(double a,double b);
/// Computes the normalized incomplete beta function B(a,b,x)
double NormalizedIncompleteBeta(double a,double b,double x);
/// Computes the error function 
double Erf(double x);
/// Computes the complimentary error function?
double ErfComplimentary(double x);

/// Returns -1 if x<eps, 0 if |x|<=eps, 1 if x>eps
inline int FuzzySign(double x,double eps=dEpsilon) { return (x>eps?1:(x<-eps?-1:0)); }
/// Returns true of a and be have different signs, treating 0 as 
/// a positive number
inline bool OpposingSigns_pos(double a,double b) { return (a<dZero) != (b<dZero); }
/// Same as above, but treats 0 as a negative number
inline bool OpposingSigns_neg(double a,double b) { return (a<=dZero) != (b<=dZero); }


int quadratic(float a, float b, float c, float& x1, float& x2);
int cubic(float a, float b, float c, float d, float x[3]);
float pythag(float a, float b);
float pythag_leg(float a,float c);
float Sinc(float x);
float Sinc_Dx(float x);
float fFactorial(unsigned int n);
float fLogFactorial(unsigned int n);
float fChoose(unsigned int n,unsigned int k);
float fLogChoose(unsigned int n,unsigned int k);
float TaylorCoeff(float x,unsigned int n);
float Gamma(float x);
float LogGamma(float x);
float GammaInv(float x);
float Beta(float a,float b);
float LogBeta(float a,float b);
float NormalizedIncompleteBeta(float a,float b,float x);
float Erf(float x);
float ErfComplimentary(float x);

inline int FuzzySign(float x,float eps=fEpsilon) { return (x>eps?1:(x<-eps?-1:0)); }
inline bool OpposingSigns_pos(float a,float b) { return (a<fZero) != (b<fZero); }
inline bool OpposingSigns_neg(float a,float b) { return (a<=fZero) != (b<=fZero); }



///Calculates x^i where i is an integer (in O(log i) time)
template <class T>
T IntegerPower(const T x, int i)
{
  if(i<0) return 1.0/IntegerPower(x,-i);
  if(i==0) return 1;
  if(i&1)	//odd
    { T y=IntegerPower(x,(i-1)>>1); return x*y*y; }
  else //even
    { T y=IntegerPower(x,i>>1); return y*y; }
}

///Returns n! in O(n) time
inline unsigned int Factorial(unsigned int n)
{
  //assert(n <= 12); //otherwise, it'll cause overflow with 32-bit arith
  int x=1;
  for(unsigned int i=1;i<=n;i++) x*=i;
  return x;
}
 
///Returns n!/(n-k)! in O(k) time
inline unsigned int FactorialTruncated(unsigned int n,unsigned int k)
{
  unsigned int fact=1;
  for(unsigned int j=0;j<k;j++) fact*=(n-j);
  return fact;
}

///Returns `n choose k' = n!/k!(n-k)! in O(k) time
inline unsigned int Choose(unsigned int n,unsigned int k)
{
  if(2*k > n) return FactorialTruncated(n,n-k)/Factorial(n-k);
  else return FactorialTruncated(n,k)/Factorial(k);
}

inline Real rFactorial(unsigned int n) { return dFactorial(n); }
inline Real LogFactorial(unsigned int n) { return dLogFactorial(n); }
inline Real rChoose(unsigned int n,unsigned int k) { return dChoose(n,k); }
inline Real LogChoose(unsigned int n,unsigned int k) { return dLogChoose(n,k); }

  /*@}*/
} //namespace Math

#endif

