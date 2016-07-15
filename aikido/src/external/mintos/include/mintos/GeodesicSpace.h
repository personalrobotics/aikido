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

#ifndef MINTOS_GEODESIC_SPACE_H
#define MINTOS_GEODESIC_SPACE_H

#include <mintos/math/vector.h>
#include <mintos/math/metric.h>
#include <mintos/math/interpolate.h>

namespace Mintos { 

  using namespace Math;

  typedef Vector Config;

/** @brief Information about interpolation and derivatives on a manifold. 
 *
 * The Interpolate function is a function x=f(a,b,u).  This
 * class provides information about df/du (InterpolateDeriv),
 * df/da*a' (InterpolateDerivA), df/db*b' (InterpolateDerivB), and ddf/du^2
 * (InterpolateDeriv2). 
 *
 * It also allows you to integrate a tangent vector to the manifold da
 * starting from configuration a (Integrate).
 *
 * An example of the use of this would be geodesic interpolator on a sphere
 * that is parameterized via theta/phi angles.
 *
 * The default implementation assumes a linear interpolation.
 */
class GeodesicManifold
{
 public:
  virtual ~GeodesicManifold() {}
  ///optional constraint checking
  virtual bool IsFeasible(const Config& x) { return true; }

  ///optionally overrideable (default uses euclidean space)
  virtual Real Distance(const Config& x, const Config& y) { return Distance_L2(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { interpolate(x,y,u,out); }

  virtual void InterpolateDeriv(const Config& a,const Config& b,Real u,Vector& dx) { dx.sub(b,a); }
  virtual void InterpolateDerivA(const Config& a,const Config& b,Real u,const Vector& da,Vector& dx) { dx.mul(da,1-u); }
  virtual void InterpolateDerivB(const Config& a,const Config& b,Real u,const Vector& db,Vector& dx) { dx.mul(db,u); }
  virtual void InterpolateDeriv2(const Config& a,const Config& b,Real u,Vector& ddx) { ddx.resize(a.n,Zero); }
  virtual void Integrate(const Config& a,const Vector& da,Config& b) { b.add(a,da); }
};

} //namespace Mintos

#endif
