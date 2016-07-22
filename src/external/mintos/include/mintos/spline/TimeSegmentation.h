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

#ifndef SPLINE_TIME_SEGMENTATION_H
#define SPLINE_TIME_SEGMENTATION_H

#include <mintos/math/math.h>
#include <vector>
#include <algorithm>

namespace Mintos {
namespace Spline {

  using namespace Math;

///@brief Divides a real-valued range t[0],t[1],...,t[n-1] into segments
struct TimeSegmentation : public std::vector<Real>
{
  static int Map(const std::vector<Real>& timing,Real t) {
    if(timing.empty() || t < timing.front())    return -1;
    std::vector<Real>::const_iterator i=--std::upper_bound(timing.begin(),timing.end(),t);
    if(i == timing.end()) { return (int)timing.size()-1; }
    else { return i-timing.begin(); }
  }

  ///Same as above, but calculates a parameter u such that
  ///t=(1-u)*t[i]+u*t[i+1].
  static int Map(const std::vector<Real>& timing,Real t,Real& param) { 
    if(timing.empty() || t < timing.front())  { param=0.0; return -1; }
    std::vector<Real>::const_iterator i=--std::upper_bound(timing.begin(),timing.end(),t),n;
    if(i == timing.end() || i==--timing.end()) { param=1.0; return (int)timing.size()-1; }
    else { n=i; ++n; }
    param = (t-*i)/(*n-*i);
    return i-timing.begin();
  }

  ///Returns an index i such that t is in [t[i],t[i+1]).  If it is beyond
  ///the end of the times, returns n-1.  If it is before the first
  ///time, returns -1.
  int Map(Real t) const { 
    return TimeSegmentation::Map(*this,t);
  }

  ///Same as above, but calculates a parameter u such that
  ///t=(1-u)*t[i]+u*t[i+1].
  int Map(Real t,Real& param) const { 
    return TimeSegmentation::Map(*this,t,param);
  }

};

} //namespace Spline
} //namespace Mintos
#endif
