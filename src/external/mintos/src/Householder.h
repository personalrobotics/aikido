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
#ifndef MATH_HOUSEHOLDER_H
#define MATH_HOUSEHOLDER_H

#include <mintos/math/VectorTemplate.h>
#include <mintos/math/MatrixTemplate.h>

/** @file Householder.h
 * @ingroup Math
 * @brief Functions for householder transformations.
 */

namespace Math {

  /** @addtogroup Math */
  /*@{*/

/** Replace v[0:n-1] with a householder vector (v[0:n-1]) and
   coefficient tau that annihilate v[1:n-1].  Tau is returned */
template <class T>
T HouseholderTransform(VectorTemplate<T>& v);

/** Applies a householder transformation v,tau to matrix A */
template <class T>
void HouseholderPreMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A);

/** Applies a householder transformation v,tau to matrix m from the
   right hand side in order to zero out rows */
template <class T>
void HouseholderPostMultiply(T tau, const VectorTemplate<T>& v, MatrixTemplate<T>& A);

/** Applies a householder transformation tau,v to vector w */
template <class T>
void HouseholderApply(T tau, const VectorTemplate<T>& v, VectorTemplate<T>& w);

/** Applies a householder transformation v,tau to a matrix being
   built up from the identity matrix, using the first column of A as
   a householder vector */
template <class T>
void HouseholderHM1(T tau, MatrixTemplate<T>& A);

  /*@}*/
} //namespace Math

#endif
