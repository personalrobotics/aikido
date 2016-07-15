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
#include "LSQRInterface.h"
#include "lsqr.h"
using namespace Optimization;
using namespace std;

struct SparseMatrixMultiplier : public lsqr_func
{
  SparseMatrixMultiplier(const dSparseMatrix_RM& _A) :A(_A) { }

  /* compute  y = y + A*x*/
  virtual void MatrixVectorProduct (const dVector& x, dVector& y)
  {
    A.madd(x,y);
  }
  /* compute  x = x + At*y*/
  virtual void MatrixTransposeVectorProduct (dVector& x, const dVector& y)
  {
    A.maddTranspose(y,x);
  }

  const dSparseMatrix_RM& A;
};

LSQRInterface::LSQRInterface()
  :dampValue(0),relError(0),condLimit(0),maxIters(0),verbose(1)
{}

bool LSQRInterface::Solve(const SparseMatrix& A,const Vector& b)
{
  dSparseMatrix_RM dA; dA.copy(A);
  SparseMatrixMultiplier func(dA);
  lsqr_input input;
  lsqr_work work;
  lsqr_output output;
  input.num_rows = A.m;
  input.num_cols = A.n;
  input.damp_val = dampValue;
  input.rel_mat_err = input.rel_rhs_err = relError;
  input.cond_lim = condLimit;
  if(maxIters)  input.max_iter = maxIters;
  else          input.max_iter = A.n*4;
  switch(verbose) {
  case 1: input.lsqr_fp_out = stdout;  break;
  case 2: input.lsqr_fp_out = stderr;  break;
  default: input.lsqr_fp_out = NULL;   break;
  }
  input.rhs.copy(b);
  if(x0.n == 0) input.sol.resize(A.n,Zero);
  else if(x0.n == A.n) input.sol.copy(x0);
  else {
    cerr<<"Initial guess doesn't have correct dimensions"<<endl;
    cerr<<"Using zeros for initial guess"<<endl;
    input.sol.resize(A.n,Zero);
  }

  lsqr(input,output,work,func);

  numIters = output.num_iters;
  condEstA = output.mat_cond_num;
  residualNorm = output.resid_norm;
  x.copy(output.sol);
  stdErr.copy(output.std_err);
  
  switch(output.term_flag) {
  case lsqr_output::X0Exact:
    if(verbose) cout<<"LSQR: X0 is the exact solution!"<<endl;
    break;
  case lsqr_output::ExactSolutionRelMat:
    if(verbose) cout<<"LSQR: Solved approximately the exact solution"<<endl;
    break;
  case lsqr_output::LSSolutionRelMat:
    if(verbose) cout<<"LSQR: Solved approximately a least-squares solution"<<endl;
    break;
  case lsqr_output::IllConditioned:
    if(verbose) cout<<"LSQR: The matrix is probably ill-conditioned"<<endl;
    return false;
  case lsqr_output::ExactSolution:
    if(verbose) cout<<"LSQR: Solved the exact solution"<<endl;
    break;
  case lsqr_output::LSSolution:
    if(verbose) cout<<"LSQR: Solved the least-squares solution"<<endl;
    break;
  case lsqr_output::ConditionError:
    if(verbose) cout<<"LSQR: The condition number became very large"<<endl;
    return false;
  case lsqr_output::MaxItersReached:
    if(verbose) cout<<"LSQR: The max # of iterations has been reached, residual "<<residualNorm<<endl;
    return false;
  default:
    cerr<<"LSQR: Unknown return value "<<output.term_flag<<endl;
    return false;
  }
  return true;
}
