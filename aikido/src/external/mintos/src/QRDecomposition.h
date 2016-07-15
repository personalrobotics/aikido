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
#ifndef MATH_QR_DECOMPOSITION_H
#define MATH_QR_DECOMPOSITION_H

#include <mintos/math/MatrixTemplate.h>

namespace Math {

/** @ingroup Math
 * @brief Calculates the QR decomposition.
 *
 * Factorise an M x N matrix A into A = Q R, where Q is orthogonal
 * (M x M) and R is upper triangular (M x N).
 *
 * Q is stored as a packed set of Householder transformations in the
 * strict lower triangular part of the input matrix.  R is stored in
 * the diagonal and upper triangle of the input matrix.
 *
 * The full matrix for Q can be obtained as the product
 *
 *       Q = Q_k .. Q_2 Q_1
 *
 * where k = MIN(M,N) and
 *
 *       Q_i = (I - tau_i * v_i * v_i')
 *
 * and where v_i is a Householder vector
 *
 *       v_i = [1, m(i+1,i), m(i+2,i), ... , m(M,i)]
 *
 * This storage scheme is the same as in LAPACK.  
 */
template <class T>
class QRDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  bool set(const MatrixT& A);
  void backSub(const VectorT& x,VectorT& b) const;
  void leastSquares(const VectorT& x,VectorT& b,VectorT& residual) const;
  void QMul(const VectorT& b,VectorT& x) const;
  void QtMul(const VectorT& b,VectorT& x) const;
  void RBackSub(const VectorT& b,VectorT& x) const;
  void getQ(MatrixT& Q) const;
  void getR(MatrixT& R) const;
  
  MatrixT QR;
  VectorT tau;
};

/** @ingroup Math
 * @brief The QR decomposition as computed by the algorithm
 * in Numerical Recipes in C.
 */
template <class T>
class NRQRDecomposition
{
public:
  typedef MatrixTemplate<T> MatrixT;
  typedef VectorTemplate<T> VectorT;
  bool set(const MatrixT& A);
  void backSub(const VectorT& x,VectorT& b) const;
  void QBackSub(const VectorT& x,VectorT& b) const;
  void getQ(MatrixT& Q) const;
  
  MatrixT QR;
  VectorT c;
  bool singular;
};

} //namespace Math

#endif
