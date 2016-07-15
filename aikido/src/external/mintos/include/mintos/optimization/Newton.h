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

#ifndef OPTIMIZATION_NEWTON_H
#define OPTIMIZATION_NEWTON_H

#include <mintos/math/function.h>
#include <mintos/math/root.h>
#include <mintos/math/SVDecomposition.h>
#include <vector>

namespace Math {
  template <class T> class SparseMatrixTemplate_RM;
  typedef SparseMatrixTemplate_RM<Real> SparseMatrix;
}

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief A globally convergent Newton's method for multidimensional root
 * solving.  Solves for func(x)=0.
 */
class NewtonRoot
{
public:
  NewtonRoot(VectorFieldFunction* func);
  virtual ~NewtonRoot();
  bool GlobalSolve(int& iters,ConvergenceResult* res=NULL);
  ConvergenceResult Solve(int& iters);
  ConvergenceResult Solve_Sparse(int& iters);
  bool LineMinimization(const Vector& g, const Vector& p, Real *f);
  Real MaxDistance(const Vector& x);
  virtual bool SolveUnderconstrainedLS(const Matrix& A,const Vector& b,Vector& x);
  virtual bool SolveUnderconstrainedLS(const SparseMatrix& A,const Vector& b,Vector& x);
  virtual Real Merit();  //evaluates the merit function at x

  Vector x;
  VectorFieldFunction* func;
  Real tolf,tolmin,tolx;
  Real stepMax;  ///< maximum distance to step
  Real lambda;   ///< damped-least-squares constant 
  Vector bmin,bmax; ///< optional bound constraints
  bool sparse;   ///< set to true if should use a sparse least-squares solver
  int verbose;
  int debug;

  //temporary
  RobustSVD<Real> svd;
  Vector fx, g, p, xold;
  Matrix fJx;
};

/** @ingroup Optimization
 * @brief A globally convergent Newton's method with inequality constraints
 * c(x) >= 0.
 */
class ConstrainedNewtonRoot : public NewtonRoot
{
public:
  ConstrainedNewtonRoot(VectorFieldFunction* func,VectorFieldFunction* c);
  bool GlobalSolve(int& iters,ConvergenceResult* res=NULL);
  ConvergenceResult SolveConstrained(int& iters);
  ConvergenceResult SolveConstrained_Sparse(int& iters);
  virtual Real Merit();

  VectorFieldFunction* c;
  Real tolc;

  //temporary
  Vector cx;
  std::vector<int> activeSetC,activeSetBound;
  Matrix A;  //solve for A*dx = rhs
  Vector rhs;
};

///Returns C(x).maxElement()
Real SurfaceDistance(VectorFieldFunction*C,const Vector& x);
///Returns true if C(x)<=tol, pointwise
bool SatisfiesEquality(VectorFieldFunction*C,const Vector& x,Real tol=Epsilon);
///Returns true if C(x) >= margin
bool SatisfiesInequality(VectorFieldFunction*C,const Vector& x,Real margin=Zero);
///Returns C(x).minElement() (and the index if non=NULL)
Real InequalityMargin(VectorFieldFunction* c,const Vector& x,int* index=NULL);

} //namespace Optimization

#endif
