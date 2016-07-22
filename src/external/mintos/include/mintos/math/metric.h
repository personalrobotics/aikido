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


#ifndef MATH_METRIC_H
#define MATH_METRIC_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"

/** @file metric.h 
 * @ingroup Math
 * @brief Standard vector/matrix metrics
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

///A class that can generate norms for arbitrary amounts of streaming data without
///storing the data in memory.
template <class T>
class NormAccumulator
{
 public:
  NormAccumulator(Real exponent=2.0);
  void collect(T val);
  void collect(T val,Real weight);
  T norm() const;
  T normSquared() const;
  inline void operator << (T val) { collect(val); }
  inline operator T () const { return norm(); }

  Real exponent;
  T data;
};

template<class T>
T Norm(const VectorTemplate<T>& x,Real norm);
template<class T>
T Norm_L1(const VectorTemplate<T>& x);
template<class T>
T Norm_L2(const VectorTemplate<T>& x);
///Same as above, but robust to over/underflow
template<class T>
T Norm_L2_Safe(const VectorTemplate<T>& x);
template<class T>
T Norm_LInf(const VectorTemplate<T>& x);
template<class T>
T Norm_Mahalanobis(const VectorTemplate<T>& x,const MatrixTemplate<T>& A);
template<class T>
T Norm_Weighted(const VectorTemplate<T>& x,Real norm,const VectorTemplate<T>& w);
template<class T>
T Norm_WeightedL1(const VectorTemplate<T>& x,const VectorTemplate<T>& w);
template<class T>
T Norm_WeightedL2(const VectorTemplate<T>& x,const VectorTemplate<T>& w);
template<class T>
T Norm_WeightedLInf(const VectorTemplate<T>& x,const VectorTemplate<T>& w);

template<class T>
T Distance(const VectorTemplate<T>& x,const VectorTemplate<T>& y,Real norm);
template<class T> 
T Distance_L1(const VectorTemplate<T>& x,const VectorTemplate<T>& y);
template<class T>
T Distance_L2(const VectorTemplate<T>& x,const VectorTemplate<T>& y);
///Same as above, but robust to over/underflow
template<class T>
T Distance_L2_Safe(const VectorTemplate<T>& x,const VectorTemplate<T>& y);
template<class T>
T Distance_LInf(const VectorTemplate<T>& x,const VectorTemplate<T>& y);
template<class T>
T Distance_Mahalanobis(const VectorTemplate<T>& x,const VectorTemplate<T>& y,const MatrixTemplate<T>& A);
template<class T>
T Distance_Weighted(const VectorTemplate<T>& x,const VectorTemplate<T>& y,Real norm,const VectorTemplate<T>& w);
template<class T>
T Distance_WeightedL1(const VectorTemplate<T>& x,const VectorTemplate<T>& y,const VectorTemplate<T>& w);
template<class T>
T Distance_WeightedL2(const VectorTemplate<T>& x,const VectorTemplate<T>& y,const VectorTemplate<T>& w);
template<class T>
T Distance_WeightedLInf(const VectorTemplate<T>& x,const VectorTemplate<T>& y,const VectorTemplate<T>& w);

template<class T>
T Norm_L1(const MatrixTemplate<T>& A);
template<class T>
T Norm_LInf(const MatrixTemplate<T>& A);
template<class T>
T Norm_Frobenius(const MatrixTemplate<T>& A);
///Same as above, but robust to over/underflow
template<class T>
T Norm_Frobenius_Safe(const MatrixTemplate<T>& A);

template<class T>
T Distance_L1(const MatrixTemplate<T>& A,const MatrixTemplate<T>& B);
template<class T>
T Distance_LInf(const MatrixTemplate<T>& A,const MatrixTemplate<T>& B);
template<class T>
T Distance_Frobenius(const MatrixTemplate<T>& A,const MatrixTemplate<T>& B);
///Same as above, but robust to over/underflow
template<class T>
T Distance_Frobenius_Safe(const MatrixTemplate<T>& A,const MatrixTemplate<T>& B);

/*@}*/
} //namespace Math

#endif
