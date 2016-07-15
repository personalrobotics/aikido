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

#ifndef MATH_INFNAN_H
#define MATH_INFNAN_H

#if defined (__APPLE__) || defined (MACOSX)
#include "/usr/include/math.h"
#else
#include <math.h>
#endif

#ifndef INFINITY
  #include <limits>
#endif //INFINITY

/** @file math/infnan.h
 * @ingroup Math
 * @brief Cross-platform infinity and not-a-number routines.
 *
 * Not necessarily throroughly tested.  Developed partially
 * because cygwin's isnan's go into infinite loops.
 */

namespace Math { 

/** @addtogroup Math */
/*@{*/


#ifdef INFINITY
  const static double dInf = INFINITY;
  const static float fInf = INFINITY;
#else
  const static double dInf = std::numeric_limits<double>::infinity();
  const static float fInf = std::numeric_limits<float>::infinity();
#endif // INFINITY


///Returns nonzero if x is not-a-number (NaN)
int IsNaN(double x);
int IsNaN(float x);
///Returns nonzero unless x is infinite or a NaN
int IsFinite(double x);
int IsFinite(float x);
///Returns +1 if x is +inf, -1 if x is -inf, and 0 otherwise
int IsInf(double x);
int IsInf(float x);

/*@}*/
} //namespace Math

#endif
