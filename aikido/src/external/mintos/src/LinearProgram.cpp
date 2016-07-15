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
#include "LinearProgram.h"
#include <mintos/math/SVDecomposition.h>
#include <mintos/misc/errors.h>

using namespace Optimization;
using namespace std;

LinearProgram::LinearProgram()
  :minimize(true)
{}

void LinearProgram::Resize(int m,int n)
{
  LinearConstraints::Resize(m,n);
  c.resize(n,Zero);
}

void LinearProgram::AddVariables(int num)
{
  LinearConstraints::AddVariables(num);
  c.resizePersist(A.n,0.0);
}

void LinearProgram::AddVariable(Real lj,Real uj)
{
  LinearConstraints::AddVariable(lj,uj);
  c.resizePersist(A.n,0.0);
}

bool LinearProgram::IsValid() const
{
  return (A.n == c.n) && LinearConstraints::IsValid();
}

Real LinearProgram::Objective(const Vector& x) const
{
  return dot(x,c);
}

void LinearProgram::Print(std::ostream& out) const
{
  out<<(minimize ? "min" : "max")<<" x.[";
  for(int i=0;i<A.n;i++)
    out<<c(i)<<" ";
  out<<"] subject to:"<<endl;
  LinearConstraints::Print(out);
}

const LinearProgram& LinearProgram::operator = (const LinearProgram& lp)
{
  minimize = lp.minimize;
  c = lp.c;
  LinearConstraints::operator = (lp);
  return *this;
}


LinearProgram_Sparse::LinearProgram_Sparse()
  :minimize(true)
{}

void LinearProgram_Sparse::Resize(int m,int n)
{
  LinearConstraints_Sparse::Resize(m,n);
  c.resize(n,Zero);
}

void LinearProgram_Sparse::AddVariables(int num)
{
  LinearConstraints_Sparse::AddVariables(num);
  c.resizePersist(A.n,0.0);
}

void LinearProgram_Sparse::AddVariable(Real lj,Real uj)
{
  LinearConstraints_Sparse::AddVariable(lj,uj);
  c.resizePersist(A.n,0.0);
}


bool LinearProgram_Sparse::IsValid() const
{
  return (A.n == c.n) && LinearConstraints_Sparse::IsValid();
}

Real LinearProgram_Sparse::Objective(const Vector& x) const
{
  return dot(x,c);
}

void LinearProgram_Sparse::Print(std::ostream& out) const
{
  out<<(minimize ? "min" : "max")<<" x.[";
  for(int i=0;i<A.n;i++)
    out<<c(i)<<" ";
  out<<"] subject to:"<<endl;
  LinearConstraints_Sparse::Print(out);
}

const LinearProgram_Sparse& LinearProgram_Sparse::operator = (const LinearProgram_Sparse& lp)
{
  minimize = lp.minimize;
  c = lp.c;
  LinearConstraints_Sparse::operator = (lp);
  return *this;
}






void LinearConstraints::Resize(int m,int n)
{
  A.resize(m,n,Zero);
  q.resize(m,-Inf);
  p.resize(m,Inf);
  l.resize(n,-Inf);
  u.resize(n,Inf);
}

bool LinearConstraints::IsValid() const
{
  if(A.isEmpty()) return l.n==u.n;
  return (q.n==A.m) && (p.n==A.m) &&
    (l.n == A.n) && (u.n == A.n);
}


LinearConstraints::BoundType LinearConstraints::ConstraintType(int i) const
{
  if(IsInf(q(i))==-1) {
    if(IsInf(p(i)) == 1) return Free;
    return UpperBound;
  }
  else if(IsInf(p(i)) == 1) {
    return LowerBound;
  }
  else {
    if(q(i)==p(i)) return Fixed;
    else return Bounded;
  }
}

LinearConstraints::BoundType LinearConstraints::VariableType(int j) const
{
  if(IsInf(l(j))==-1) {
    if(IsInf(u(j)) == 1) return Free;
    return UpperBound;
  }
  else if(IsInf(u(j)) == 1) {
    return LowerBound;
  }
  else {
    if(l(j)==u(j)) return Fixed;
    else return Bounded;
  }
}

bool LinearConstraints::HasEqualities() const
{
  for(int i=0;i<A.m;i++)
    if(ConstraintType(i) == Fixed) return true;
  return false;
}

bool LinearConstraints::HasInequalities() const
{
  for(int i=0;i<A.m;i++)
    if(ConstraintType(i) != Fixed && ConstraintType(i) != Free) return true;
  return false;
}

bool LinearConstraints::HasBounds() const
{
  for(int i=0;i<l.n;i++)
    if(VariableType(i) != Free) return true;
  return false;
}

Real LinearConstraints::InequalityMargin(const Vector& x) const
{
  Real minMargin=Inf;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) != Fixed) {
      Real d = A.dotRow(i,x);
      if(d-q(i) < minMargin) minMargin = d-q(i);
      if(p(i)-d < minMargin) minMargin = p(i)-d;
    }
  }
  return minMargin;
}

Real LinearConstraints::BoundMargin(const Vector& x) const
{
  Real minMargin=Inf;
  for(int i=0;i<x.n;i++) {
    if(x(i)-l(i) < minMargin) minMargin = x(i)-l(i);
    if(u(i)-x(i) < minMargin) minMargin = u(i)-x(i);
  }
  return minMargin;
}

Real LinearConstraints::EqualityError(const Vector& x) const 
{
  Real maxMargin=0;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == Fixed) {
      Real m=Abs(A.dotRow(i,x)-q(i));
      if(m > maxMargin) maxMargin=m;
    }
  }
  return maxMargin;
}

Real LinearConstraints::InfeasibilityMeasure(const Vector& x) const 
{
  Real minMargin=Inf;
  for(int i=0;i<A.m;i++) {
    Real d = A.dotRow(i,x);
    if(d-q(i) < minMargin) minMargin = d-q(i);
    if(p(i)-d < minMargin) minMargin = p(i)-d;
  }
  for(int i=0;i<x.n;i++) {
    if(x(i)-l(i) < minMargin) minMargin = x(i)-l(i);
    if(u(i)-x(i) < minMargin) minMargin = u(i)-x(i);
  }
  return minMargin;
}

bool LinearConstraints::SatisfiesInequalities(const Vector& x) const
{
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) != Fixed) {
      Real d = A.dotRow(i,x);
      if(d > p(i) || d < q(i)) return false;
    }
  }
  return true;
}

bool LinearConstraints::SatisfiesBounds(const Vector& x) const
{
  for(int i=0;i<A.n;i++) {
    if(x(i) > u(i) || x(i) < l(i)) return false;
  }
  return true;
}

bool LinearConstraints::SatisfiesEqualities(const Vector& x,Real tol) const 
{
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == Fixed) {
      if(!FuzzyEquals(A.dotRow(i,x),q(i),tol)) return false;
    }
  }
  return true;
}

bool LinearConstraints::IsFeasible(const Vector& x,Real equalityTol) const 
{
  if(!SatisfiesBounds(x)) return false;
  if(!SatisfiesInequalities(x)) return false;
  if(!SatisfiesEqualities(x,equalityTol)) return false;
  return true;
}

void LinearConstraints::ProjectDirection(Vector& v) const
{
  cerr<<"LinearConstraints::ProjectDirection(): is deprecated!"<<endl;
  getchar();
  //if constraint is a.x <= b, if direction increases a.x, crop it to 0
  //if it's a.x = b, crop direction to 0
  Real initialMaxErr=Inf;
  Real maxErr=0;
  vector<int> bounds;
  Matrix C;
  RobustSVD<Real> svd;
  Vector vNull;
  int maxIters=20;
  for(int iters=0;iters<maxIters;iters++) {
    bounds.resize(0);
    maxErr=Zero;
    for(int i=0;i<A.m;i++) {
      Real p = A.dotRow(i,v);
      bool include = false;
      switch(ConstraintType(i)) {
      case Free: break;
      case Fixed:
      case Bounded:
        include=true; break;
      case UpperBound:
        include = (p>0); break;
      case LowerBound:
        include = (p<0); break;
      }
      if(include) {
	/*
	cout<<"ProjectDirection: error at bound "<<i<<": "<<p<<endl;
	Vector ai;
	A.getRowRef(i,ai);
	cout<<"Bound direction: "<<VectorPrinter(ai)<<endl;
	*/
	bounds.push_back(i);
	maxErr = Max(maxErr,Abs(p));
      }
    }
    if(maxErr < 1e-5) return;
    /*
    if(maxErr >= initialMaxErr) {
      cout<<"ProjectDirection(): Error, the error bound increased!"<<endl;
      cout<<"From "<<initialMaxErr<<" to "<<maxErr<<endl;
      Abort();
    }
    */
    initialMaxErr=maxErr;
    //cout<<"ProjectVector: "<<bounds.size()<<" bounds, max error "<<maxErr<<endl;
    C.resize((int)bounds.size(),A.n);
    for(size_t i=0;i<bounds.size();i++) {
      Vector ai;
      A.getRowRef(bounds[i],ai);
      C.copyRow((int)i,ai);
    }

    //TODO: should this be put into SVDecomposition::set()?
    svd.svd.U.resize(C.m,C.n);
    if(!svd.set(C)) {
      cout<<"ProjectDirection(): Warning: unable to set SVD, returning prematurely"<<endl;
      return;
    }
    svd.nullspaceComponent(v,vNull);
    v -= vNull;
  }
  cout<<"Error: ProjectDirection didn't converge within "<<maxIters<<" iters, resulting error "<<maxErr<<endl;
  if(maxErr > 1e-3) {
    cout<<"Press enter to continue..."<<endl;
    getchar();
  }
}

void LinearConstraints::Print(std::ostream& out) const
{
  int rowinc = 1+(A.n/6);
  int numlines = rowinc;
  int lastline = 0;
  for(int i=0;i<A.m;i++,numlines+=rowinc) {
    if(ConstraintType(i)==LowerBound || ConstraintType(i)==Bounded)
      out<<q(i)<<" < ";
    out<<"[";
    for(int j=0;j<A.n;j++)
      out<<A(i,j)<<" ";
    out<<"].x";
    if(ConstraintType(i)==UpperBound || ConstraintType(i)==Bounded)
      out<<" < "<<p(i);
    else if(ConstraintType(i)==Fixed)
      out<<" = "<<p(i);
    out<<endl;
    
    if((out == cout || out == cerr) && (numlines-lastline) >= 40) {
      out<<"Press enter to continue..."<<endl;
      getchar();
      lastline = numlines;
    }
  }
  for(int i=0;i<A.n;i++,numlines++) {
    switch(VariableType(i)) {
    case Free: break;
    case Fixed: out<<"x["<<i<<"] = "<<u(i)<<endl; break;
    case LowerBound: out<<"x["<<i<<"] >= "<<l(i)<<endl; break;
    case UpperBound: out<<"x["<<i<<"] <= "<<u(i)<<endl; break;
    case Bounded:
      out<<l(i)<<" <= x["<<i<<"] <= "<<u(i)<<endl;
      break;
    }

    if((&out == &cout || &out == &cerr) && (numlines-lastline) >= 40) {
      out<<"Press enter to continue..."<<endl;
      getchar();
      lastline = numlines;
    }
  }
}

void LinearConstraints::AddConstraints(int num)
{
  A.resizePersist(A.m+num,A.n,0.0);
  p.resizePersist(A.m,Inf);
  q.resizePersist(A.m,-Inf);
}

void LinearConstraints::AddConstraint(Real qi,const Vector& Ai,Real pi)
{
  A.resizePersist(A.m+1,A.n);
  A.copyRow(A.m-1,Ai);
  q.resizePersist(q.n+1,qi);
  p.resizePersist(p.n+1,pi);
}

void LinearConstraints::AddVariables(int num)
{
  A.resizePersist(A.m,A.n+num,0.0);
  l.resizePersist(A.n,-Inf);
  u.resizePersist(A.n,Inf);
}

void LinearConstraints::AddVariable(Real lj,Real uj)
{
  A.resizePersist(A.m,A.n+1,0.0);
  l.resizePersist(A.n,lj);
  u.resizePersist(A.n,uj);
}


void LinearConstraints::GetSimpleForm(Matrix& Aeq,Vector& beq,Matrix& Aineq,Vector& bineq) const
{
  int neq=0,nineq=0;
  for(int i=0;i<A.m;i++) {
    switch(ConstraintType(i)) {
    case Fixed: neq++; break;
    case Bounded: nineq+=2; break;
    case UpperBound:
    case LowerBound:
      nineq++;
      break;
    default: break;
    }
  }
  for(int i=0;i<A.n;i++) {
    switch(VariableType(i)) {
    case Fixed: neq++; break;
    case Bounded: nineq+=2; break;
    case UpperBound:
    case LowerBound:
      nineq++;
      break;
    default: break;
    }
  }
  Aeq.resize(neq,A.n);
  Aineq.resize(nineq,A.n);
  beq.resize(neq);
  bineq.resize(nineq);

  //copy the rows
  neq = nineq = 0;
  Vector temp;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == Fixed) {
      A.getRowRef(i,temp);
      Aeq.copyRow(neq,temp);
      beq(neq) = p(i);
      neq++;
    }
    else if(HasUpperBound(ConstraintType(i))) {
      Aineq.getRowRef(nineq,temp);
      A.getRowCopy(i,temp);
      bineq(nineq) = p(i);
      nineq++;
    }
    else if(HasLowerBound(ConstraintType(i))) {
      Aineq.getRowRef(nineq,temp);
      A.getRowCopy(i,temp);
      temp.inplaceNegative();
      bineq(nineq) = -q(i);
      nineq++;
    }
  }
  for(int i=0;i<A.n;i++) {
    switch(VariableType(i)) {
    case Fixed:
      Aeq(neq,i) = 1;
      beq(neq) = l(i);
      neq++;
      break;
    case Bounded: 
      Aineq(nineq,i) = 1;
      Aineq(nineq+1,i) = -1;
      bineq(nineq) = u(i);
      bineq(nineq+1) = -l(i);
      nineq+=2;
      break;
    case UpperBound:
      Aineq(nineq,i) = 1;
      bineq(nineq) = u(i);
      nineq++;
      break;
    case LowerBound:
      Aineq(nineq,i) = -1;
      bineq(nineq) = -l(i);
      nineq++;
      break;
    default: break;
    }
  }
}

void LinearConstraints::SetSimpleForm(const Matrix& Aeq,const Vector& beq,const Matrix& Aineq,const Vector& bineq)
{
  Assert(Aeq.n == Aineq.n);
  Assert(beq.n == Aeq.m);
  Assert(bineq.n == Aineq.m);
  Resize(Aeq.m+Aineq.m,Aeq.n);
  A.copySubMatrix(0,0,Aeq);
  A.copySubMatrix(Aeq.m,0,Aineq);
  q.copySubVector(0,beq);
  p.copySubVector(0,beq);
  p.copySubVector(Aeq.m,bineq);
}

void LinearConstraints::SetRef(const LinearConstraints& lp)
{
  A.setRef(lp.A);
  q.setRef(lp.q);
  p.setRef(lp.p);
  u.setRef(lp.u);
  l.setRef(lp.l);
}

void LinearConstraints::Copy(const LinearConstraints& lp)
{
  A = lp.A;
  q = lp.q;
  p = lp.p;
  u = lp.u;
  l = lp.l;
}

void LinearConstraints::Swap(LinearConstraints& lp)
{
  A.swap(lp.A);
  q.swap(lp.q);
  p.swap(lp.p);
  u.swap(lp.u);
  l.swap(lp.l);
}

void LinearConstraints_Sparse::Resize(int m,int n)
{
  A.resize(m,n);
  q.resize(m,-Inf);
  p.resize(m,Inf);
  l.resize(n,-Inf);
  u.resize(n,Inf);
}


void LinearConstraints_Sparse::AddConstraints(int num)
{
  A.resize(A.m+num,A.n);
  p.resizePersist(A.m,Inf);
  q.resizePersist(A.m,-Inf);
}

void LinearConstraints_Sparse::AddConstraint(Real qi,const SparseVector& Ai,Real pi)
{
  A.resize(A.m+1,A.n);
  A.copyRow(A.m-1,Ai);
  q.resizePersist(q.n+1,qi);
  p.resizePersist(p.n+1,pi);
  Assert(q(q.n-1) == qi);
  Assert(p(p.n-1) == pi);
}

void LinearConstraints_Sparse::AddVariables(int num)
{
  A.resize(A.m,A.n+num);
  l.resizePersist(A.n,-Inf);
  u.resizePersist(A.n,Inf);
}

void LinearConstraints_Sparse::AddVariable(Real lj,Real uj)
{
  A.resize(A.m,A.n+1);
  l.resizePersist(A.n,lj);
  u.resizePersist(A.n,uj);
}

bool LinearConstraints_Sparse::IsValid() const
{
  return (q.n==A.m) && (p.n==A.m) &&
    (l.n == A.n) && (u.n == A.n);
}


LinearConstraints::BoundType LinearConstraints_Sparse::ConstraintType(int i) const
{
  if(IsInf(q(i))==-1) {
    if(IsInf(p(i)) == 1) return LinearConstraints::Free;
    return LinearConstraints::UpperBound;
  }
  else if(IsInf(p(i)) == 1) {
    return LinearConstraints::LowerBound;
  }
  else {
    if(q(i)==p(i)) return LinearConstraints::Fixed;
    else return LinearConstraints::Bounded;
  }
}

LinearConstraints::BoundType LinearConstraints_Sparse::VariableType(int j) const
{
  if(IsInf(l(j))==-1) {
    if(IsInf(u(j)) == 1) return LinearConstraints::Free;
    return LinearConstraints::UpperBound;
  }
  else if(IsInf(u(j)) == 1) {
    return LinearConstraints::LowerBound;
  }
  else {
    if(l(j)==u(j)) return LinearConstraints::Fixed;
    else return LinearConstraints::Bounded;
  }
}


bool LinearConstraints_Sparse::HasEqualities() const
{
  for(int i=0;i<A.m;i++)
    if(ConstraintType(i) == LinearConstraints::Fixed) return true;
  return false;
}

bool LinearConstraints_Sparse::HasInequalities() const
{
  for(int i=0;i<A.m;i++)
    if(ConstraintType(i) != LinearConstraints::Fixed && ConstraintType(i) != LinearConstraints::Free) return true;
  return false;
}

bool LinearConstraints_Sparse::HasBounds() const
{
  for(int i=0;i<A.m;i++)
    if(VariableType(i) != LinearConstraints::Free) return true;
  return true;
}

Real LinearConstraints_Sparse::InequalityMargin(const Vector& x) const
{
  Real minMargin=Inf;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) != LinearConstraints::Fixed) {
      Real d = A.dotRow(i,x);
      if(d-q(i) < minMargin) minMargin = d-q(i);
      if(p(i)-d < minMargin) minMargin = p(i)-d;
    }
  }
  return minMargin;
}

Real LinearConstraints_Sparse::BoundMargin(const Vector& x) const
{
  Real minMargin=Inf;
  for(int i=0;i<x.n;i++) {
    if(x(i)-l(i) < minMargin) minMargin = x(i)-l(i);
    if(u(i)-x(i) < minMargin) minMargin = u(i)-x(i);
  }
  return minMargin;
}

Real LinearConstraints_Sparse::EqualityError(const Vector& x) const 
{
  Real maxMargin=0;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == LinearConstraints::Fixed) {
      Real m=Abs(A.dotRow(i,x)-q(i));
      if(m > maxMargin) maxMargin=m;
    }
  }
  return maxMargin;
}

Real LinearConstraints_Sparse::InfeasibilityMeasure(const Vector& x) const 
{
  Real minMargin=Inf;
  for(int i=0;i<A.m;i++) {
    Real d = A.dotRow(i,x);
    if(d-q(i) < minMargin) minMargin = d-q(i);
    if(p(i)-d < minMargin) minMargin = p(i)-d;
  }
  for(int i=0;i<x.n;i++) {
    if(x(i)-l(i) < minMargin) minMargin = x(i)-l(i);
    if(u(i)-x(i) < minMargin) minMargin = u(i)-x(i);
  }
  return minMargin;
}

bool LinearConstraints_Sparse::SatisfiesInequalities(const Vector& x) const
{
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) != LinearConstraints::Fixed) {
      Real d = A.dotRow(i,x);
      if(d > p(i) || d < q(i)) return false;
    }
  }
  return true;
}

bool LinearConstraints_Sparse::SatisfiesBounds(const Vector& x) const
{
  for(int i=0;i<A.n;i++) {
    if(x(i) > u(i) || x(i) < l(i)) return false;
  }
  return true;
}

bool LinearConstraints_Sparse::SatisfiesEqualities(const Vector& x,Real tol) const 
{
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == LinearConstraints::Fixed) {
      if(!FuzzyEquals(A.dotRow(i,x),q(i),tol)) return false;
    }
  }
  return true;
}

bool LinearConstraints_Sparse::IsFeasible(const Vector& x,Real equalityTol) const 
{
  if(!SatisfiesBounds(x)) return false;
  if(!SatisfiesInequalities(x)) return false;
  if(!SatisfiesEqualities(x,equalityTol)) return false;
  return true;
}

void LinearConstraints_Sparse::Print(std::ostream& out) const
{
  int numlines = 1+(A.n/6);
  int lastline = 0;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i)==LinearConstraints::LowerBound || ConstraintType(i)==LinearConstraints::Bounded)
      out<<q(i)<<" < ";
    out<<"[";
    for(SparseMatrix::RowT::const_iterator j=A.rows[i].begin();j!=A.rows[i].end();j++)
      if(Abs(j->second) < 1e-3) 
	out<<j->first<<":"<<"e"<<" ";
      else
	out<<j->first<<":"<<j->second<<" ";
    out<<"].x";
    if(ConstraintType(i)==LinearConstraints::UpperBound || ConstraintType(i)==LinearConstraints::Bounded)
      out<<" < "<<p(i);
    else if(ConstraintType(i)==LinearConstraints::Fixed)
      out<<" = "<<p(i);
    out<<endl;

    if((&out == &cout || &out == &cerr) && (numlines-lastline) >= 40) {
      out<<"Press enter to continue..."<<endl;
      getchar();
      lastline = numlines;
    }
    numlines += 1+(A.rows[i].numEntries()/6);
  }
  for(int i=0;i<A.n;i++,numlines++) {
    switch(VariableType(i)) {
    case LinearConstraints::Free: break;
    case LinearConstraints::Fixed: out<<"x["<<i<<"] = "<<u(i)<<endl; break;
    case LinearConstraints::LowerBound: out<<"x["<<i<<"] >= "<<l(i)<<endl; break;
    case LinearConstraints::UpperBound: out<<"x["<<i<<"] <= "<<u(i)<<endl; break;
    case LinearConstraints::Bounded:
      out<<l(i)<<" <= x["<<i<<"] <= "<<u(i)<<endl;
      break;
    }

    if((out == cout || out == cerr) && (numlines-lastline) == 40) {
      out<<"Press enter to continue..."<<endl;
      getchar();
      lastline = numlines;
    }
  }
}


void LinearConstraints_Sparse::GetSimpleForm(SparseMatrix& Aeq,Vector& beq,SparseMatrix& Aineq,Vector& bineq) const
{
  int neq=0,nineq=0;
  for(int i=0;i<A.m;i++) {
    switch(ConstraintType(i)) {
    case LinearConstraints::Fixed: neq++; break;
    case LinearConstraints::Bounded: nineq+=2; break;
    case LinearConstraints::UpperBound:
    case LinearConstraints::LowerBound:
      nineq++;
      break;
    default: break;
    }
  }
  for(int i=0;i<A.n;i++) {
    switch(VariableType(i)) {
    case LinearConstraints::Fixed: neq++; break;
    case LinearConstraints::Bounded: nineq+=2; break;
    case LinearConstraints::UpperBound:
    case LinearConstraints::LowerBound:
      nineq++;
      break;
    default: break;
    }
  }
  Aeq.resize(neq,A.n);
  Aineq.resize(nineq,A.n);
  beq.resize(neq);
  bineq.resize(nineq);

  //copy the rows
  neq = nineq = 0;
  for(int i=0;i<A.m;i++) {
    if(ConstraintType(i) == LinearConstraints::Fixed) {
      Aeq.rows[neq] = A.rows[i];
      beq(neq) = p(i);
      neq++;
    }
    else if(HasUpperBound(ConstraintType(i))) {
      Aineq.rows[nineq] = A.rows[i];
      bineq(nineq) = p(i);
      nineq++;
    }
    else if(HasLowerBound(ConstraintType(i))) {
      Aineq.rows[nineq] = A.rows[i];
      for(SparseMatrix::RowIterator j=Aineq.rows[nineq].begin();j!=Aineq.rows[nineq].end();j++)
	j->second = -j->second;
      bineq(nineq) = -q(i);
      nineq++;
    }
  }
  for(int i=0;i<A.n;i++) {
    switch(VariableType(i)) {
    case LinearConstraints::Fixed:
      Aeq(neq,i) = 1;
      beq(neq) = l(i);
      neq++;
      break;
    case LinearConstraints::Bounded: 
      Aineq(nineq,i) = 1;
      Aineq(nineq+1,i) = -1;
      bineq(nineq) = u(i);
      bineq(nineq+1) = -l(i);
      nineq+=2;
      break;
    case LinearConstraints::UpperBound:
      Aineq(nineq,i) = 1;
      bineq(nineq) = u(i);
      nineq++;
      break;
    case LinearConstraints::LowerBound:
      Aineq(nineq,i) = -1;
      bineq(nineq) = -l(i);
      nineq++;
      break;
    default: break;
    }
  }
}

void LinearConstraints_Sparse::SetSimpleForm(const SparseMatrix& Aeq,const Vector& beq,const SparseMatrix& Aineq,const Vector& bineq)
{
  Assert(Aeq.n == Aineq.n);
  Assert(beq.n == Aeq.m);
  Assert(bineq.n == Aineq.m);
  Resize(Aeq.m+Aineq.m,Aeq.n);
  A.copySubMatrix(0,0,Aeq);
  A.copySubMatrix(Aeq.m,0,Aineq);
  q.copySubVector(0,beq);
  p.copySubVector(0,beq);
  p.copySubVector(Aeq.m,bineq);
}

void LinearConstraints_Sparse::Copy(const LinearConstraints_Sparse& lp)
{
  A = lp.A;
  q = lp.q;
  p = lp.p;
  u = lp.u;
  l = lp.l;
}


void LinearConstraints_Sparse::Swap(LinearConstraints_Sparse& lp)
{
  A.swap(lp.A);
  q.swap(lp.q);
  p.swap(lp.p);
  u.swap(lp.u);
  l.swap(lp.l);
}
