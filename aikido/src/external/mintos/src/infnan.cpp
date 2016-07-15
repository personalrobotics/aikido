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
#include <mintos/math/infnan.h>
#include <mintos/math/math.h>
#include <stdio.h>
#include <math.h>

namespace Math {

int IsNaN(double x)
{
#ifdef _MSC_VER
  return _isnan(x);
#elif HAVE_DECL_ISNAN
  return isnan(x);
#elif HAVE_IEEE_COMPARISONS
  return (x!=x?1:0);
#else
  #error "IsNaN: Neither Microsoft's _isnan, isnan, or IEEE comparisons defined"
  return 0;
#endif
}

int IsFinite(double x)
{
#ifdef _MSC_VER
  return _finite(x);
#elif HAVE_DECL_ISFINITE
  return isfinite(x);
#elif HAVE_DECL_FINITE
  return finite(x);
#elif HAVE_IEEE_COMPARISONS
  double y=x-x;
  return (y==y?1:0);
#else
  #error "IsFinite: Neither Microsoft's _isfinite, isfinite, or IEEE comparisons defined"
  return 0;
#endif
}

int IsInf(double x)
{
#ifdef _MSC_VER  //doesn't have isinf
  int cls = _fpclass(x);
  if(cls == _FPCLASS_PINF) return 1;
  else if(cls == _FPCLASS_NINF) return -1;
  else return 0;
#elif HAVE_DECL_ISINF
  if(isinf(x)) {
    if(x > 0) return 1;
    else return -1;
  }
  else return 0;
#elif HAVE_IEEE_COMPARISONS
  double y=x-x;
  if(IsNaN(y)) 
    return (x>0?1:-1);
  else return 0;
#else
  #error "IsInf: Neither Microsoft's _fpclass, isinf, or IEEE comparisons defined"
  return 0;
#endif
}



int IsNaN(float x)
{
#ifdef _MSC_VER
  return _isnan(x);
#elif HAVE_DECL_ISNAN
  //return isnanf(x);
  return isnan(x);
#elif HAVE_IEEE_COMPARISONS
  return (x!=x?1:0);
#else
  #error "IsNaN: Neither Microsoft's _isnan, isnan, or IEEE comparisons defined"
  return 0;
#endif
}

int IsFinite(float x)
{
#ifdef _MSC_VER
  return _finite(x);
#elif HAVE_DECL_FINITE
  return finitef(x);
#elif HAVE_DECL_ISFINITE
  return isfinitef(x);
#elif HAVE_IEEE_COMPARISONS
  float y=x-x;
  return (y==y?1:0);
#else
  #error "IsFinite: Neither Microsoft's _isfinite, isfinite, or IEEE comparisons defined"
  return 0;
#endif
}

int IsInf(float x)
{
#ifdef _MSC_VER  //doesn't have isinf
  int cls = _fpclass(x);
  if(cls == _FPCLASS_PINF) return 1;
  else if(cls == _FPCLASS_NINF) return -1;
  else return 0;
#elif HAVE_DECL_ISINF
  //if(isinff(x)) {
  if(isinf(x)) {
    if(x > 0) return 1;
    else return -1;
  }
  else return 0;
#elif HAVE_IEEE_COMPARISONS
  float y=x-x;
  if(IsNaN(y)) 
    return (x>0?1:-1);
  else return 0;
#else
  #error "IsInf: Neither Microsoft's _fpclass, isinf, or IEEE comparisons defined"
  return 0;
#endif
}

} //namespace Math
