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
#ifndef OPTIMIZATION_LSQR_INTERFACE_H
#define OPTIMIZATION_LSQR_INTERFACE_H

#include <mintos/math/sparsematrix.h>
#include <mintos/math/vector.h>

namespace Optimization
{
using namespace Math;

/** @ingroup Optimization
 * @brief An interface to a sparse least-squares solver (lsqr).
 */
struct LSQRInterface
{
  LSQRInterface();
  bool Solve(const SparseMatrix& A,const Vector& b);

  //input quantities
  Vector x0;       ///<initial guess for x -- default set to 0's
  Real dampValue;  ///<damping term, min ||Ax-b||^2 + dampValue||x||^2
  Real relError;   ///<relative error in defining A,b
  Real condLimit;  ///<stop if the estimated condition number of A exceeds condLim
  int maxIters;    ///<maximum number of iterations, set to 0 to use default value
  int verbose;     ///<0 - no output printed, 1 - output to stdout, 2 - output to stderr

  //output quantities
  Vector x;        ///<the solution vector
  Vector stdErr;  ///<the standard error estimates
  int numIters;    ///<the number of iterations used
  Real condEstA;   ///<condition number estimate for A
  Real residualNorm; //<the estimate of the final residual's norm
};

} //namespace Optimization

#endif
