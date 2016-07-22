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
#ifndef MATH_AABB_H
#define MATH_AABB_H

#include <mintos/math/vector.h>
#include <mintos/misc/errors.h>

/** @file math/AABB.h
 * @ingroup Math
 * @brief Functions defining axis-aligned bounding boxes (AABBs).
 * 
 * An AABB consists of two Vectors (bmin,bmax), such that a Vector x
 * is inside the AABB if \f$ bmin_i \leq x_i \leq bmax_i \f$ for all i.
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

/// Clamps x to be contained in the AABB (bmin,bmax)
inline void AABBClamp(Vector& x,const Vector& bmin,const Vector& bmax,Real d=Zero)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++)
    x(i) = Clamp(x(i),bmin(i)+d,bmax(i)-d);
}

/// Returns the minimum distance from x to the boundaries of the
/// AABB (bmin,bmax).  Also returns the element index that contains the
/// minimum margin.
inline Real AABBMargin(const Vector& x,const Vector& bmin,const Vector& bmax,int& index)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  Real margin=Inf;
  index=-1;
  for(int i=0;i<x.n;i++) {
    if(x(i)-bmin(i) < margin) {
      margin = x(i)-bmin(i);
      index=i;
    }
    if(bmax(i)-x(i) < margin) {
      margin = bmax(i)-x(i);
      index=i;
    } 
  }
  return margin;
}

/// Returns the minimum distance from x to the boundaries of the
/// AABB (bmin,bmax)
inline Real AABBMargin(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  Real margin=Inf;
  for(int i=0;i<x.n;i++) {
    margin = Min(margin,x(i)-bmin(i));
    margin = Min(margin,bmax(i)-x(i));
  }
  return margin;
}

/// Returns true if the AABB (bmin,bmax) contains x
inline bool AABBContains(const Vector& x,const Vector& bmin,const Vector& bmax,Real d=Zero)
{
  Assert(x.n==bmin.n);
  Assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++)
    if(x(i) < bmin(i)+d || x(i) > bmax(i)+d) return false;
  return true;
}

/// For a point inside the AABB (bmin,bmax), limits the desired step t
/// x = x0+t*dx such that x is inside the AABB.  Returns the index responsible
/// for this limit
int AABBLineSearch(const Vector& x0,const Vector& dx,const Vector& bmin,const Vector& bmax,Real& t);

/// Clips the line segment x=x0+u*dx, with t in [u0,u1]
/// to the AABB (bmin,bmax).  Returns false if there is no intersection.
bool AABBClipLine(const Vector& x0,const Vector& dx,
		  const Vector& bmin,const Vector& bmax,
		  Real& u0,Real& u1);

/*@}*/
} //namespace Math

#endif

