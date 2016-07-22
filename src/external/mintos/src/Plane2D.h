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
#ifndef MATH3D_PLANE2D_H
#define MATH3D_PLANE2D_H

#include "primitives.h"

namespace Math3D {

typedef Vector2 Point2D;
struct Plane2D;
struct Segment2D;

/** @brief A 2D plane class
 * @ingroup Math3D
 *
 * Represents plane with a normal and offset such that x on the plane 
 * satisfy dot(normal,x) = offset.
 */
struct Plane2D
{
  void setPointNormal(const Point2D& a, const Vector2& n);
  void setPoints(const Point2D& a, const Point2D& b);
  void setTransformed(const Plane2D& pin, const Matrix3& xform);
  
  Real distance(const Point2D& v) const;
  void project(const Point2D& in, Point2D& out) const;		///<projects onto the plane
  void getBasis(Vector2& xb) const;            ///<returns a plane basis vector
  
  bool intersectsSegment(const Segment2D&, Real* t);
  
  ///returns the dimension of the intersection of the 2 planes. <br>
  ///if 0, they don't intersect, pt is intersection "point at infinty". <br>
  ///1, it's a point returned in pt <br>
  ///2, the planes are identical
  int allIntersections(const Plane2D& p,Vector2& pt) const;

  Vector2 normal;
  Real offset;
};

} //namespace Math3D

#endif
