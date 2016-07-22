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

#ifndef MATH_SV_DECOMPOSITION_H
#define MATH_SV_DECOMPOSITION_H

#include "MatrixTemplate.h"
#include "DiagonalMatrix.h"

namespace Math { 

/** @ingroup Math
 * @brief Performs the singular value decomposition.
 *
 * Decomposes the matrix A to UWV^t.  The values in the diagonal matrix W
 * are the singular values.  The backsubstitution functions can either
 * ignore zero singular values (within tolerance epsilon) or used a
 * "damped" backsubstitution where W is replaced with W+lambda*I.
 *
 * The nullspace matrix is the matrix N such that for each column n,
 * A*n = 0.
 */
template <class T>
class SVDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  typedef DiagonalMatrixTemplate<T> DiagonalMatrixT;

  SVDecomposition();
  SVDecomposition(const MatrixT& A);
  
  bool set(const MatrixT& A);
  void setIdentity(int m,int n);
  void setZero(int m,int n);
  void resize(int m,int n);
  void clear();
  void backSub(const VectorT& b, VectorT& x) const;
  void dampedBackSub(const VectorT& b,T lambda, VectorT& x) const;
  void nullspaceComponent(const VectorT& x,VectorT& xNull) const;
  int getRank() const;
  int getNull() const;
  void getInverse(MatrixT& Ainv) const;
  void getDampedPseudoInverse(MatrixT& Aplus,T lambda) const;
  void getNullspace(MatrixT& N) const;

  ///sorts the singular values from highest to lowest
  void sortSVs();
  
  MatrixT U;
  DiagonalMatrixT W;
  MatrixT V;

  //settings
  int maxIters;
  T epsilon;
};

/** @ingroup Math
 * @brief Performs a pre/postconditioned singular value decomposition.
 *
 * Performs the SVD on the matrix A' = Pre^-1*A*Post^-1 such that
 * A = Pre*U*W*Vt*Post.
 *
 * Also zero's out small elements of A' with tolerance zeroElementEpsilon.
 *
 * @see SVDecomposition
 */
template <class T>
class RobustSVD
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  typedef DiagonalMatrixTemplate<T> DiagonalMatrixT;

  RobustSVD();
  RobustSVD(const MatrixT& A);
  
  bool set(const MatrixT& A);
  bool setConditioned(const MatrixT& A);
  void setIdentity(int m,int n);
  void setZero(int m,int n);
  void resize(int m,int n);
  void clear();
  void backSub(const VectorT& b, VectorT& x) const;
  void dampedBackSub(const VectorT& b,T lambda, VectorT& x) const;
  void nullspaceComponent(const VectorT& x,VectorT& xNull) const;
  int getRank() const;
  int getNull() const;
  void getInverse(MatrixT& Ainv) const;
  void getDampedPseudoInverse(MatrixT& Aplus,T lambda) const;
  void getNullspace(MatrixT& N) const;
  void calcConditioning(const MatrixT& A);
  
  DiagonalMatrixT Pre;
  SVDecomposition<T> svd;
  DiagonalMatrixT Post;

  //settings
  T zeroElementEpsilon;
  bool preMultiply,postMultiply;
};

} //namespace Math

#endif
