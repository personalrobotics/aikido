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
#ifndef OPTIMIZATION_LINEAR_PROGRAM_H
#define OPTIMIZATION_LINEAR_PROGRAM_H

#include <mintos/math/matrix.h>
#include <mintos/math/sparsematrix.h>
#include <mintos/math/sparsevector.h>
#include <vector>

namespace Optimization {
using namespace Math;

/** @brief Linear constraints for LP's and QP's
 * @ingroup Optimization
 *
 * Represents the constraints
 * @verbatim
 *   qi <= ai.x <= pi <br>
 *   lj <= xj <= uj
 * @endverbatim
 *
 * A free constraint (default) is specified by qi=-Inf and pi=Inf
 * An equality is specified by setting qi=pi.
 * An upper or lower-bounded inequality is set by setting qi=-Inf or pi=Inf.
 * A double-bounded inequality has qi < pi.
 * Similar rules apply to bound constraints.
 *
 * EqualityError() measures the maximum of |ai.x-pi| over all equality
 * constraints i.
 *
 * InequalityMargin() measures the minimum of ai.x-qi and pi-ai.x for
 * all non-equality constraints i.  (Negative when infeasible, as usual.)
 *
 * BoundMargin() measures the minimum of xj-lj, and uj-xj over all j.
 *
 * InfeasibilityMeasure() measures the maximum of qi-ai.x, ai.x-pi, lj-xj,
 * and xj-uj for all i and j.  Positive for strictly feasible, negative
 * for infeasible.
 *
 * Get/SetSimpleForm() converts the problem to/from the form
 * Aeq*x=beq, Aineq*x<=bineq.
 *
 * AddVariable/AddConstraint are relatively inefficient at building the
 * matrix.  Resize is much faster.
 */
struct LinearConstraints
{
  enum BoundType { Free, LowerBound, UpperBound, Bounded, Fixed };
  void Resize(int m,int n);
  void AddConstraint(Real qi,const Vector& Ai,Real pi);
  void AddConstraints(int num);
  void AddVariable(Real li=-Inf,Real ui=Inf);
  void AddVariables(int num);
  bool IsValid() const;
  BoundType ConstraintType(int i) const;
  BoundType VariableType(int j) const;
  inline static bool HasLowerBound(BoundType b) { return b==LowerBound || b==Bounded; }
  inline static bool HasUpperBound(BoundType b) { return b==UpperBound || b==Bounded; }
  bool HasEqualities() const;
  bool HasInequalities() const;
  bool HasBounds() const;
  Real EqualityError(const Vector& x) const;
  Real InequalityMargin(const Vector& x) const;
  Real BoundMargin(const Vector& x) const;
  Real InfeasibilityMeasure(const Vector& x) const;
  bool SatisfiesEqualities(const Vector& x,Real tol=Epsilon) const;
  bool SatisfiesInequalities(const Vector& x) const;
  bool SatisfiesBounds(const Vector& x) const;
  bool IsFeasible(const Vector& x,Real equalityTol=Epsilon) const;
  void ProjectDirection(Vector& v) const;
  void Print(std::ostream& out=std::cout) const;
  void GetSimpleForm(Matrix& Aeq,Vector& beq,Matrix& Aineq,Vector& bineq) const;
  void SetSimpleForm(const Matrix& Aeq,const Vector& beq,const Matrix& Aineq,const Vector& bineq);
  void SetRef(const LinearConstraints&);
  void Copy(const LinearConstraints&);
  void Swap(LinearConstraints&);
  const LinearConstraints& operator = (const LinearConstraints& lp) { Copy(lp); return *this; }

  Matrix A;
  Vector q,p;
  Vector l,u;
};

/** @brief Sparse linear constraints for LP's and QP's
 * @ingroup Optimization
 *
 * Unlike dense constraints, AddVariable/AddConstraint are relatively
 * efficient.
 *
 * @see LinearConstraints
 */
struct LinearConstraints_Sparse
{
  typedef LinearConstraints::BoundType BoundType;
  void Resize(int m,int n);
  void AddConstraint(Real qi,const SparseVector& Ai,Real pi);
  void AddConstraints(int num);
  void AddVariable(Real li=-Inf,Real ui=Inf);
  void AddVariables(int num);
  bool IsValid() const;
  BoundType ConstraintType(int i) const;
  BoundType VariableType(int j) const;
  bool HasEqualities() const;
  bool HasInequalities() const;
  bool HasBounds() const;
  inline static bool HasLowerBound(BoundType b) { return LinearConstraints::HasLowerBound(b); }
  inline static bool HasUpperBound(BoundType b) { return LinearConstraints::
HasUpperBound(b); }
  Real EqualityError(const Vector& x) const;
  Real InequalityMargin(const Vector& x) const;
  Real BoundMargin(const Vector& x) const;
  Real InfeasibilityMeasure(const Vector& x) const;
  bool SatisfiesEqualities(const Vector& x,Real tol=Epsilon) const;
  bool SatisfiesInequalities(const Vector& x) const;
  bool SatisfiesBounds(const Vector& x) const;
  bool IsFeasible(const Vector& x,Real equalityTol=Epsilon) const;
  void ProjectDirection(Vector& v) const;
  void Print(std::ostream& out=std::cout) const;
  void GetSimpleForm(SparseMatrix& Aeq,Vector& beq,SparseMatrix& Aineq,Vector& bineq) const;
  void SetSimpleForm(const SparseMatrix& Aeq,const Vector& beq,const SparseMatrix& Aineq,const Vector& bineq);
  void Copy(const LinearConstraints_Sparse&);
  void Swap(LinearConstraints_Sparse&);
  const LinearConstraints_Sparse& operator = (const LinearConstraints_Sparse& lp) { Copy(lp); return *this; }

  SparseMatrix A;
  Vector q,p;
  Vector l,u;
};


/** @brief Linear program definition.
 * @ingroup Optimization
 *
 * Represents the LP
 * min/max c.x subject to <br>
 *   qi <= ai.x <= pi <br>
 *   lj <= xj <= uj
 */
struct LinearProgram : public LinearConstraints
{
  enum Result { Feasible, Infeasible, Unbounded, Error };
  LinearProgram();
  void Resize(int m,int n);
  void AddVariable(Real l=-Inf,Real u=Inf);
  void AddVariables(int num);
  bool IsValid() const;
  Real Objective(const Vector& x) const;
  void Print(std::ostream& out=std::cout) const;
  const LinearProgram& operator = (const LinearProgram&);

  bool minimize;
  Vector c;
};

/** @brief Linear program definition with sparse matrix A.
 * @ingroup Optimization
 */  
 struct LinearProgram_Sparse : public LinearConstraints_Sparse
{
  typedef LinearProgram::Result Result;
  typedef LinearProgram::BoundType BoundType;

  LinearProgram_Sparse();
  void Resize(int m,int n);
  void AddVariable(Real l=-Inf,Real u=Inf);
  void AddVariables(int num);
  bool IsValid() const;
  Real Objective(const Vector& x) const;
  void Print(std::ostream& out=std::cout) const;
  const LinearProgram_Sparse& operator = (const LinearProgram_Sparse&);

  bool minimize;
  Vector c;
};

} //namespace Optimization

#endif
