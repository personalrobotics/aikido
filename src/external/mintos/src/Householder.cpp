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
#include "Householder.h"
#include "misc.h"
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {

/* replace v[0:n-1] with a householder vector (v[0:n-1]) and
   coefficient tau that annihilate v[1:n-1].  Tau is returned */
template <class T>
T HouseholderTransform(VectorTemplate<T>& v)
{
  Assert(v.n != 0);
  if (v.n == 1) return 0;
  T alpha, beta, tau ;   
  VectorTemplate<T> x; x.setRef(v,1); 
  T xnorm = x.norm();
  if (xnorm == 0)  {
    return 0;
  }
      
  alpha = v(0);
  beta = - (alpha >= 0.0 ? 1 : -1) * pythag(alpha, xnorm);
  tau = (beta - alpha) / beta ;
  
  x.inplaceDiv(alpha-beta);
  v(0)=beta;
  return tau;
}

  /* applies a householder transformation v,tau to matrix A */
template <class T>
void HouseholderPreMultiply (T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A)
{
  if (tau == 0) return;
  int i, j;
  for (j=0;j<A.n;j++) {
    /* Compute wj = Akj vk */
    
    T wj = A(0,j);  
    for (i=1;i<A.m;i++)  /* note, computed for v(0) = 1 above */
      wj += A(i,j) * v(i);
    
    /* Aij = Aij - tau vi wj */
    
    /* i = 0 */
    A(0,j) -= tau*wj;
    
    /* i = 1 .. M-1 */   
    for (i=1;i<A.m;i++)
      A(i,j) -= tau * v(i) * wj;
  }
}

/* applies a householder transformation v,tau to matrix m from the
   right hand side in order to zero out rows */
template <class T>
void HouseholderPostMultiply (T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A)
{
  if (tau == 0) return;

  /* A = A - tau w v' */
  int i, j;
  for (i=0;i<A.m;i++) {
    T wi = A(i,0);

    for (j=1;j<A.n;j++)  /* note, computed for v(0) = 1 above */
      wi += A(i,j)*v(j);
      
      /* j = 0 */
    A(i,0) -= tau*wi;

    /* j = 1 .. N-1 */    
    for (j=1;j<A.n;j++) 
      A(i,j) -= tau * wi * v(j);
  }
}

/* applies a householder transformation tau,v to vector w */
template <class T>
void HouseholderApply (T tau, const VectorTemplate<T>& v, VectorTemplate<T>& w)
{
  if (tau == 0) return;

  /* compute d = v'w */
  T d = w(0);
  VectorTemplate<T> v1,w1;
  v1.setRef(v,1);
  w1.setRef(w,1);
  d += v1.dot(w1);

  /* compute w = w - tau (v) (v'w) */
  w(0) -= tau*d;
  w1.madd(v1,-tau*d);
}


/* applies a householder transformation v,tau to a matrix being
   built up from the identity matrix, using the first column of A as
   a householder vector */
template <class T>
void HouseholderHM1(T tau, MatrixTemplate<T>& A)
{
  int i,j;
  if (tau == 0)  {
    A(0,0) = 1;
    for(j=1;j<A.n;j++) A(0,j)=0;
    for(i=1;i<A.m;i++) A(i,0)=0;
    return;
  }

  /* w = A' v */
  for(j=1;j<A.n;j++) {
    T wj = 0;   /* A0j * v0 */
    for (i=1;i<A.m;i++)
      wj += A(i,j) * A(i,0);
    
    /* A = A - tau v w' */
    A(0,j) -= tau*wj;
    for(i=1;i<A.m;i++)
      A(i,j) -= tau*A(i,0)*wj;
  }

  for(i=1;i<A.m;i++)
      A(i,0) *= -tau;

  A(0,0) = (T)1 - tau;
}


#define DEFINEHH(T) \
  template T HouseholderTransform(VectorTemplate<T>& v); \
  template void HouseholderPreMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A); \
  template void HouseholderPostMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A); \
  template void HouseholderApply(T tau, const VectorTemplate<T>& v, VectorTemplate<T>& w); \
  template void HouseholderHM1(T tau, MatrixTemplate<T>& A);

DEFINEHH(float);
DEFINEHH(double);

}
