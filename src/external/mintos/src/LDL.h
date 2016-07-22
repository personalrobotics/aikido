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
#ifndef MATH_LDL_H
#define MATH_LDL_H

#include <mintos/math/matrix.h>

namespace Math {

/** @ingroup Math
 * @brief Performs the LDL^t decompositoin of a symmetric matrix A.
 *
 * L is stored in the lower diagonal of LDL, D is stored in the diagonal.
 * If any of the elements of D's diagonal have abs value less than 
 * zeroTolerance (i.e. A is singular), then they are ignored.
 */
template <class T>
struct LDLDecomposition
{
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;

  LDLDecomposition();
  LDLDecomposition(const MatrixT& A);

  void set(const MatrixT& A);
  bool backSub(const VectorT& b, VectorT& x) const;
  void LBackSub(const VectorT& b, VectorT& x) const;
  void LTBackSub(const VectorT& b, VectorT& x) const;
  bool DBackSub(const VectorT& b, VectorT& x) const;
  bool backSub(const MatrixT& B, MatrixT& X) const;
  bool getInverse(MatrixT& Ainv) const;
  void getPseudoInverse(MatrixT& Ainv) const;
  void getL(MatrixT& L) const;
  void getD(VectorT& d) const;
  void getA(MatrixT& A) const;
  void mulL(const Vector& x,Vector& y) const;
  void mulLT(const Vector& x,Vector& y) const;
  void mulD(const Vector& x,Vector& y) const;

  /// Update the LDL decomposition for A + xx^t
  void update(const VectorT& x);
  /// "Downdate" the LDL decomposition for A - xx^t (on failure, LDL is undefined)
  bool downdate(const VectorT& x);

  MatrixT LDL;
  T zeroTolerance;
  int verbose;
};

} // namespace Math

#endif
