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
#include "AABB.h"

namespace Math {

int AABBLineSearch(const Vector& x0,const Vector& dx,const Vector& bmin,const Vector& bmax,Real& t)
{
  Assert(x0.n == dx.n);
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  int res=-1;
  for(int i=0;i<bmax.n;i++) {
    Assert(x0(i) <= bmax(i));
    Assert(x0(i) >= bmin(i));
    if(x0(i) + t*dx(i) > bmax(i)) {
      //if x0 + t*dx = bmax-e
      //then x0+t*dx = bmin+e
      //what if dx is small?
      //t = (bmax-x0-e)/dx+e2;
      //=> x0+t dx = x0 + (bmax-x0-e) + e2 dx
      //           = (bmax-e) + e2 dx
      t = (bmax(i)-x0(i))/dx(i)*(1 - Epsilon);
      res = i;
    }
    if(x0(i) + t*dx(i) < bmin(i)) {
      //if x0 + t*dx = bmin+e
      //then x0+t*dx = bmin+e
      t = (bmin(i)-x0(i))/dx(i)*(1 - Epsilon);
      res = i;
    }
    if(!(x0(i)+t*dx(i) <= bmax(i))) {
      printf("Error: %d: %g+%g*%g=%g <= %g?\n",i,x0(i),t,dx(i),x0(i)+t*dx(i),bmax(i));
    }
    Assert(x0(i)+t*dx(i) <= bmax(i));
    if(!(x0(i)+t*dx(i) >= bmin(i))) {
      printf("Error: %d: %g+%g*%g=%g >= %g?\n",i,x0(i),t,dx(i),x0(i)+t*dx(i),bmin(i));
    }
    Assert(x0(i)+t*dx(i) >= bmin(i));
  }
  return res;

}

inline bool ClipLine1D(Real q, Real p, Real& umin, Real& umax)
{
   Real r;
   if(p<0) {			//entering
     r=-q/p;
     if(r > umax) return false;
     if(r > umin) umin = r;
   }
   else if(p>0) {
     r=-q/p;
     if(r < umin) return false;
     if(r < umax) umax = r;
   }
   else {
     if(q>0) return false;
   }
   return true;
}

bool AABBClipLine(const Vector& x0,const Vector& dx,
		  const Vector& bmin,const Vector& bmax,
		  Real& u0,Real& u1)
{
  Assert(x0.n == dx.n);
  Assert(x0.n == bmin.n);
  Assert(x0.n == bmax.n);
  for(int i=0;i<x0.n;i++) {
    //for each face, p is dot(dx, normal), q is signed dist to plane (dot(v,normal)-offset)
    if(!ClipLine1D(bmin(i) - x0(i), -dx(i), u0,u1)) return false;
    if(!ClipLine1D(x0(i) - bmax(i), dx(i), u0,u1)) return false;
  }
  return true;
}

} //namespace Math
