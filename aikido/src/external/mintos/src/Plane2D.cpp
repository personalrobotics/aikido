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
#include "Plane2D.h"
#include "Segment2D.h"
using namespace Math3D;

void Plane2D::setPointNormal(const Point2D& a, const Vector2& n)
{
	normal.setNormalized(n);
	offset = dot(a,normal);
}


void Plane2D::setPoints(const Point2D& a, const Point2D& b)
{
  Vector2 v;
  v.setPerpendicular(b-a);
  setPointNormal(a,v);
}

void Plane2D::setTransformed(const Plane2D& pin, const Matrix3& xform)
{
	xform.mulVector(pin.normal, normal);

	Vector2 v, v_out;
	v = pin.normal*pin.offset;
	xform.mulPoint(v, v_out);
	setPointNormal(v_out, normal);
}

//returns the orthogonal distance from the plane to the point
Real Plane2D::distance(const Point2D& v) const
{
	return dot(v,normal) - offset;
}

//projects a Vector2 onto this plane
void Plane2D::project(const Point2D& vin, Point2D& vout) const
{
	vout = vin - normal*distance(vin);
}

void Plane2D::getBasis(Vector2& xb) const
{
  xb.setPerpendicular(normal);
}


bool Plane2D::intersectsSegment(const Segment2D& s, Real* t)
{
  Real da = distance(s.a);
  Real db = distance(s.b);
  if(da < Zero) {
    if(db < Zero) return false;
  }
  else {
    if(db > Zero) return false;
  }
  if(t) {
    //d = da + t*(db-da)
    if(da == db) //segment on plane
      *t = Zero;
    else
      *t = da/(da-db);
  }
  return true;
}

int Plane2D::allIntersections(const Plane2D& p,Vector2& pt) const
{
  Real x = p.normal.y*offset - normal.y*p.offset;
  Real y = normal.x*p.offset - p.normal.x*offset;
  Real w = normal.x*p.normal.y - normal.y*p.normal.x;
  if(Abs(w) < Epsilon) {
    //linearly dependent
    if(Abs(x) < Epsilon && Abs(y) < Epsilon)  //overlap
      return 2;
    //pt at infinity
    pt.x = x;
    pt.y = y;
    return 0;
  }
  else {
    pt.x = x/w;
    pt.y = y/w;
    return 1;
  }
}
