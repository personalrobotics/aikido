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
#ifndef MATH_SPARSE_FUNCTION_H
#define MATH_SPARSE_FUNCTION_H

#include <mintos/math/function.h>
#include <mintos/math/sparsematrix.h>
#include <mintos/math/sparsevector.h>

/** @ingroup Math
 * @file math/sparsefunction.h
 * @brief Vector field classes with sparse jacobians
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

/** @brief A vector field function with a sparse jacobian.  The Jacobian_Sparse
 * and Jacobian_i_Sparse methods must be overridden.
 */
class SparseVectorFunction : public VectorFieldFunction
{
public:
  virtual void Jacobian(const Vector& x,Matrix& J) {
    SparseMatrix sJ(J.m,J.n);
    Jacobian_Sparse(x,sJ);
    sJ.get(J);
  }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji) {
    SparseVector sJi(Ji.n);
    Jacobian_i_Sparse(x,i,sJi);
    sJi.get(Ji);
  }
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) {
    SparseMatrix sHi(Hi.m,Hi.n);
    Hessian_i_Sparse(x,i,sHi);
    sHi.get(Hi);
  }
  virtual void Jacobian_Sparse(const Vector& x,SparseMatrix& J)=0;
  virtual void Jacobian_i_Sparse(const Vector& x,int i,SparseVector& Ji)=0;
  virtual void Hessian_i_Sparse(const Vector& x,int i,SparseMatrix& Hi) {
    FatalError("Hessian_i_Sparse() not defined in subclass of SparseVectorFunction");
  }
};

} //namespace Math

#endif
