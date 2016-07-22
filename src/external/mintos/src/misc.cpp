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
#include "misc.h"
#include "stdio.h"
#include <mintos/misc/errors.h>
#if HAVE_GSL
#include <gsl/gsl_sf_gamma.h>
#include <gsl/gsl_sf_erf.h>
#endif //HAVE_GSL

namespace Math {

static const double dFour = 4.0;
static const double dTwoPi_3 = dTwoPi/3.0;
static const double dThird = 1.0/3.0;
static const float fFour = 4.0f;
static const float fTwoPi_3 = fTwoPi/3.0f;
static const float fThird = 1.0f/3.0f;

//returns the # of real roots found (-1 if infinite)
int quadratic(double a, double b, double c, double& x1, double& x2) {
  //printf("quadratic %f %f %f\n", a, b, c);

  if(a == 0)
  {
	  if(b == 0)
	  {
		if(c == 0)
			return -1;
		return 0;
	  }
	  x1=-c/b;
	  return 1;
  }
  if(c == 0) { //det = b^2
    x1 = 0;
    x2 = -b/a;
    return 2;
  }

  double det = b*b-dFour*a*c;
  if(det < Zero)
    return 0;
  if(det == Zero) {
    x1 = -b/(2.0*a);
    return 1;
  }
  det = Sqrt(det);
  //(-b + d) / (2 a)  = (-b + d)(-b - d)/(2a)(-b - d)
  //= 4ac / (2a) (-b - d) = 2c / (-b - d)
  //(-b - d) / (2 a)  =  (-b - d) (-b + d) / (2a)(-b + d)
  //=  2c / (-b + d)
  //can choose
  //x1 = (-b + d)/2a    or    2c / (-b - d)
  //x2 = (-b - d)/2a    or    2c / (-b + d)
  //what if a and (-b-d) are both close to zero?  (happens when b < 0)
  //what if a and (-b+d) are both close to zero?  (happens when b > 0)
  //in first case: x1 is screwed, x2 is fine
  //in second case: x1 is fine, x2 is screwed
  if(Abs(-b - det) < Abs(a))
    x1 = 0.5 * (-b + det)/a;
  else
    x1 = 2.0 * c / (-b-det);
  if(Abs(-b + det) < Abs(a)) 
    x2 = 0.5 * (-b-det) / a;
  else 
    x2 = 2.0 * c / (-b+det);
  return 2;
}

int cubic(double a, double b, double c, double d, double x[3])
{
  if(a==0) 
    return quadratic(b,c,d,x[0],x[1]);
  if(a != 1) {
    b /= a;
    c /= a;
    d /= a;
  }
  double Q = (Sqr(b)-3*c)/9;
  double R = (2*b*b*b - 9*b*c+ 27*d)/54;
  double Q3 = Q*Q*Q; 
  double b_3 = b*dThird;
  if(R*R < Q3) {
    double sqrtQ = Sqrt(Q);
    double theta_3 = Acos(R/(sqrtQ*Q))*dThird;
    x[0] = -dTwo*sqrtQ*Cos(theta_3) - b_3;
    x[1] = -dTwo*sqrtQ*Cos(theta_3+dTwoPi_3) - b_3;
    x[2] = -dTwo*sqrtQ*Cos(theta_3-dTwoPi_3) - b_3;
    return 3;
  }
  else {
    double A = -Sign(R)*Pow(Abs(R)+Sqrt(R*R-Q3),dThird);
    double B = (A==0?0:Q/A);
    x[0] = A+B-b_3;
    return 1;
  }
}

double pythag(double a, double b)		//reduce roundoff of large numbers
{
  double absa = Abs(a);
  double absb = Abs(b);
  if(absa > absb)
    return absa*Sqrt(One + Sqr(absb/absa));
  else if(absb == 0)
    return Zero;
  else
    return absb*Sqrt(One + Sqr(absa/absb));
}

double pythag_leg(double a,double c)
{
  Assert(c >= 0);
  Assert(c >= Abs(a));
  if(c == 0) return 0;
  return c*Sqrt(One-Sqr(a/c));
}

double Sinc(double x)
{
	const double small=1e-7;
	if(Abs(x) < small) {	//taylor expand around 0
		const static double c[5]={1.0,-1.0/6.0,1.0/120.0,-1.0/5040.0,1.0/362880.0};
		double x2=x*x;
		return c[0]+x2*(c[1]+x2*(c[2]+x2*(c[3]+x2*c[4])));
	}
	else return Sin(x)/x;
}

double Sinc_Dx(double x)
{
	const double small=1e-4;
	if(Abs(x) < small) {	//taylor expand around 0
		const static double c[4]={-2.0/6.0,4.0/120.0,-6.0/5040.0,8.0/362880.0};
		double x2=x*x;
		return x*(c[0]+x2*(c[1]+x2*(c[2]+x2*c[3])));
	}
	else return Cos(x)/x-Sin(x)/(x*x);
}

#if HAVE_GSL
double dFactorial(unsigned int n) { return gsl_sf_fact(n); }
double dLogFactorial(unsigned int n) { return gsl_sf_lnfact(n); }
double dChoose(unsigned int n,unsigned int k) { return gsl_sf_choose(n,k); }
double dLogChoose(unsigned int n,unsigned int k) { return gsl_sf_lnchoose(n,k); }
double TaylorCoeff(double x,unsigned int n) 
{
  if(x < 0) 
    return (n&1?-1.0:1.0)*gsl_sf_taylorcoeff(n,-x); 
  else
    return gsl_sf_taylorcoeff(n,x); 
}
double Gamma(double x) { return gsl_sf_gamma(x); }
double LogGamma(double x) { return gsl_sf_lngamma(x); }
double GammaInv(double x) { return gsl_sf_gammainv(x); }
double Beta(double a,double b) { return gsl_sf_beta(a,b); }
double LogBeta(double a,double b) { return gsl_sf_lnbeta(a,b); }
double NormalizedIncompleteBeta(double a,double b,double x) { return gsl_sf_beta_inc(a,b,x); }
double Erf(double x) { return gsl_sf_erf(x); }
double ErfComplimentary(double x) { return gsl_sf_erfc(x); }

#else

double dFactorial(unsigned int n) { fprintf(stderr,"GSL not defined!\n"); return Factorial(n); }
double dLogFactorial(unsigned int n) { fprintf(stderr,"GSL not defined!\n"); return Log(dFactorial(n)); }
double dChoose(unsigned int n,unsigned int k) { fprintf(stderr,"GSL not defined!\n"); return Choose(n,k); }
double dLogChoose(unsigned int n,unsigned int k) { fprintf(stderr,"GSL not defined!\n"); return Log(dChoose(n,k)); }
double TaylorCoeff(double x,unsigned int n) { fprintf(stderr,"GSL not defined!\n"); return IntegerPower(x,n); }
double Gamma(double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double LogGamma(double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double GammaInv(double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double Beta(double a,double b) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double LogBeta(double a,double b) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double NormalizedIncompleteBeta(double a,double b,double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double Erf(double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }
double ErfComplimentary(double x) { fprintf(stderr,"GSL not defined!\n"); return 0; }

#endif



//returns the # of real roots found (-1 if infinite)
int quadratic(float a, float b, float c, float& x1, float& x2) {
  //printf("quadratic %f %f %f\n", a, b, c);

  if(a == 0)
  {
	  if(b == 0)
	  {
		if(c == 0)
			return -1;
		return 0;
	  }
	  x1=-c/b;
	  return 1;
  }
  if(c == 0) { //det = b^2
    x1 = 0;
    x2 = -b/a;
    return 2;
  }

  float det = b*b-fFour*a*c;
  if(det < Zero)
    return 0;
  if(det == Zero) {
    x1 = -b/(2.0*a);
    return 1;
  }
  det = Sqrt(det);
  if(Abs(-b - det) < Abs(a))
    x1 = 0.5 * (-b + det)/a;
  else
    x1 = 2.0 * c / (-b-det);
  if(Abs(-b + det) < Abs(a)) 
    x2 = 0.5 * (-b-det) / a;
  else 
    x2 = 2.0 * c / (-b+det);
  return 2;
}

int cubic(float a, float b, float c, float d, float x[3])
{
  if(a==0) 
    return quadratic(b,c,d,x[0],x[1]);
  if(a != 1) {
    b /= a;
    c /= a;
    d /= a;
  }
  float Q = (Sqr(b)-3*c)/9;
  float R = (2*b*b*b - 9*b*c+ 27*d)/54;
  //q and r are such that x^3-3Q*x+2r=0
  float Q3 = Q*Q*Q; 
  float b_3 = b*fThird;
  if(R*R < Q3) {
    float sqrtQ = Sqrt(Q);
    float theta_3 = Acos(R/(sqrtQ*Q))*fThird;
    x[0] = -fTwo*sqrtQ*Cos(theta_3) - b_3;
    x[1] = -fTwo*sqrtQ*Cos(theta_3+fTwoPi_3) - b_3;
    x[2] = -fTwo*sqrtQ*Cos(theta_3-fTwoPi_3) - b_3;
    /*
    printf("x^3+%f*x^2+%f*x+%f=0\n",b,c,d);
    printf("Three solutions! %f %f %f\n",x[0],x[1],x[2]);
    for(int i=0;i<3;i++)
      printf("0=%f\n",x[i]*x[i]*x[i]+b*x[i]*x[i]+c*x[i]+d);
    */
    return 3;
  }
  else {
    float A = -Sign(R)*Pow(Abs(R)+Sqrt(R*R-Q3),fThird);
    float B = (A==0?0:Q/A);
    x[0] = A+B-b_3;
    return 1;
  }
}

float pythag(float a, float b)		//reduce roundoff of large numbers
{
  float absa = Abs(a);
  float absb = Abs(b);
  if(absa > absb)
    return absa*Sqrt(One + Sqr(absb/absa));
  else if(absb == 0)
    return Zero;
  else
    return absb*Sqrt(One + Sqr(absa/absb));
}

float pythag_leg(float a,float c)
{
  Assert(c >= 0);
  Assert(c >= Abs(a));
  if(c == 0) return 0;
  return c*Sqrt(One-Sqr(a/c));
}

float Sinc(float x)
{
	const float small=1e-5f;
	if(Abs(x) < small) {	//taylor expand around 0
		const static float c[5]={1.0,-1.0/6.0,1.0/120.0,-1.0/5040.0,1.0/362880.0};
		float x2=x*x;
		return c[0]+x2*(c[1]+x2*(c[2]+x2*(c[3]+x2*c[4])));
	}
	else return Sin(x)/x;
}

float Sinc_Dx(float x)
{
	const float small=1e-2f;
	if(Abs(x) < small) {	//taylor expand around 0
		const static float c[4]={-2.0/6.0,4.0/120.0,-6.0/5040.0,8.0/362880.0};
		float x2=x*x;
		return x*(c[0]+x2*(c[1]+x2*(c[2]+x2*c[3])));
	}
	else return Cos(x)/x-Sin(x)/(x*x);
}

float fFactorial(unsigned int n) { return dFactorial(n); }
float fLogFactorial(unsigned int n) { return dLogFactorial(n); }
float fChoose(unsigned int n,unsigned int k) { return dChoose(n,k); }
float fLogChoose(unsigned int n,unsigned int k) { return dLogChoose(n,k); }
float TaylorCoeff(float x,unsigned int n) { return TaylorCoeff(double(x),n); }
float Gamma(float x) { return Gamma(double(x)); }
float LogGamma(float x) { return LogGamma(double(x)); }
float GammaInv(float x) { return GammaInv(double(x)); }
float Beta(float a,float b) { return Beta(double(a),double(b)); }
float LogBeta(float a,float b) { return LogBeta(double(a),double(b)); }
float NormalizedIncompleteBeta(float a,float b,float x) { return NormalizedIncompleteBeta(double(a),double(b),double(x)); }
float Erf(float x) { return Erf(double(x)); }
float ErfComplimentary(float x) { return ErfComplimentary(double(x)); }

} //namespace Math
