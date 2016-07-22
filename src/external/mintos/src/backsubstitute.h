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
#ifndef MATH_BACK_SUBSTITUTE_H
#define MATH_BACK_SUBSTITUTE_H

#include <mintos/math/MatrixTemplate.h>

namespace Math {

// If A is upper triangular nxn, solves Ax=b
template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// If A is lower triangular nxn, solves Ax=b
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// If A is lower triangular nxn, solves A^t*x=b
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);

// The following are identical to the above, but assume the diagonal of a is all ones
template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);
template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);
template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x);


// The following perform the same operations as above but with x and b matrices
template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);
template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);


} // namespace Math

#endif
