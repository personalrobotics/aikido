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
#include "LDL.h"
#include "backsubstitute.h"
#include <mintos/math/DiagonalMatrix.h>
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {

template<class T>
LDLDecomposition<T>::LDLDecomposition()
  :zeroTolerance((T)1e-8),verbose(1)
{}

template<class T>
LDLDecomposition<T>::LDLDecomposition(const MatrixT& A)
  :zeroTolerance((T)1e-8),verbose(1)
{
  set(A);
}

template<class T>
void LDLDecomposition<T>::set(const MatrixT& A)
{
  Assert(A.m == A.n);
  LDL.resize(A.n,A.n);
  int i,j,k;
  T sum;
  for(i=0;i<A.n;i++) {
    sum = A(i,i);
    for(k=0;k<i;k++) sum -= LDL(k,k)*Sqr(LDL(i,k));
    LDL(i,i) = sum;
    if(FuzzyZero(sum,zeroTolerance)) {
      /*
      if(verbose >= 1)
	cerr<<"Warning: LDLt decomposition has a zero element on diagonal "<<i<<endl;
      */
    }

    for(j=i+1;j<A.n;j++) {
      sum = A(i,j);
      for(int k=0;k<i;k++) sum -= LDL(k,k)*LDL(i,k)*LDL(j,k);
      if(LDL(i,i) == 0) {
	if(!FuzzyZero(sum,zeroTolerance)) {
	  if(verbose >= 1) cerr<<"LDLDecomposition: Zero diagonal, what to do with sum "<<sum<<"?"<<endl;
	  sum = 0;
	}
      }
      else 
	sum /= LDL(i,i);
      LDL(j,i) = LDL(i,j) = sum;
    }
  }


  /*
  MatrixT L,LD,LDLt;
  VectorT D;
  getL(L);
  getD(D);
  //cout<<"A: "; A.print();
  //cout<<"L: "; L.print();
  //cout<<"D: "; D.print();
  LD = L;
  for(int i=0;i<A.n;i++)
    LD.scaleCol(i,LDL(i,i));
  LDLt.mulTransposeB(LD,L);
  //cout<<"LDLt: "; LDLt.print();
  LDLt -= A;
  cout<<"Max error in LDL "<<LDLt.maxAbsElement()<<endl;
  */
}

template<class T>
bool LDLDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
  //LDLt*x=b
  //DLt*x=L^-1*b=y
  //Lt*x=D^-1*y=y'
  //x=(Lt^-1)y
  VectorT y;
  LBackSub(b,y);
  bool res=DBackSub(y,y);
  LTBackSub(y,x);
  return res;
}

template<class T>
bool LDLDecomposition<T>::backSub(const MatrixT& B, MatrixT& X) const
{
  X.resize(B.m,B.n);
  MatrixT temp(B.m,B.n);
  L1BackSubstitute(LDL,B,temp);
  VectorT tempi;
  bool res=true;
  for(int i=0;i<temp.n;i++) {
    temp.getColRef(i,tempi);
    if(!DBackSub(tempi,tempi)) res=false;
  }
  Lt1BackSubstitute(LDL,temp,X);
  return res;
}

template<class T>
void LDLDecomposition<T>::LBackSub(const VectorT& b, VectorT& x) const
{
  Assert(b.n == LDL.n);
  x.resize(LDL.n);
  L1BackSubstitute(LDL,b,x);
}

template<class T>
bool LDLDecomposition<T>::DBackSub(const VectorT& b, VectorT& x) const
{
  bool res=true;
  x.resize(b.n);
  Assert(b.n==x.n);
  for(int i=0;i<x.n;i++) {
    if(!FuzzyZero(LDL(i,i),zeroTolerance))
      x(i) = b(i)/LDL(i,i);
    else {
      if(!FuzzyZero(b(i),zeroTolerance)) {
	if(verbose >= 1) 
	  cerr<<"LDLDecomposition::DBackSub(): Warning, zero on the diagonal, b("<<i<<")="<<b(i)<<endl;
	res = false;
	x(i) = Sign(b(i))*Inf;
      }
      else
	x(i) = 0;
    }
  }
  return res;
}

template<class T>
void LDLDecomposition<T>::LTBackSub(const VectorT& b, VectorT& x) const
{
  Assert(b.n == LDL.n);
  x.resize(LDL.n);
  Lt1BackSubstitute(LDL,b,x);
}

template<class T>
bool LDLDecomposition<T>::getInverse(MatrixT& Ainv) const
{
  Ainv.resize(LDL.n,LDL.n);
  bool res=true;
  VectorT temp(LDL.n,Zero),y,x;
  for(int i=0;i<LDL.n;i++) {
    temp(i)=One;
    LBackSub(temp,y);
    if(!DBackSub(y,y)) res=false;
    LTBackSub(y,x);
    //fill in a column
    for(int j=0;j<LDL.n;j++)
      Ainv(j,i)=x(j);
    temp(i)=Zero;
  }
  return true;
}

template<class T>
void LDLDecomposition<T>::getPseudoInverse(MatrixT& Ainv) const
{
  Ainv.resize(LDL.n,LDL.n);
  VectorT temp(LDL.n,Zero),y,x;
  for(int i=0;i<LDL.n;i++) {
    temp(i)=One;
    LBackSub(temp,y);
    for(int j=0;j<y.n;j++) {
      if(!FuzzyZero(LDL(j,j),zeroTolerance))
	y(j) = y(j)/LDL(j,j);
      else
	y(j) = 0.0;
    }
    LTBackSub(y,x);
    //fill in a column
    for(int j=0;j<LDL.n;j++)
      Ainv(j,i)=x(j);
    temp(i)=Zero;
  }

  T tol = Ainv.maxAbsElement()*Epsilon;
  for(int i=0;i<LDL.n;i++)
    for(int j=0;j<i;j++) {
      if(!FuzzyEquals(Ainv(i,j),Ainv(j,i),tol))
	cout<<Ainv<<endl;
      Assert(FuzzyEquals(Ainv(i,j),Ainv(j,i),tol));
      Ainv(i,j)=Ainv(j,i) = 0.5*(Ainv(i,j)+Ainv(j,i));
    }
}

template<class T>
void LDLDecomposition<T>::getL(MatrixT& L) const
{
  Assert(LDL.m == LDL.n);
  L.resize(LDL.m,LDL.n);
  for(int i=0;i<LDL.n;i++) {
    L(i,i) = One;
    for(int j=0;j<i;j++)
      L(i,j) = LDL(i,j);
    for(int j=i+1;j<LDL.n;j++)
      L(i,j) = Zero;
  }
}

template<class T>
void LDLDecomposition<T>::getD(VectorT& d) const
{
  Assert(LDL.m == LDL.n);
  d.resize(LDL.n);
  LDL.getDiagCopy(0,d);
}

template <class T>
void LDLDecomposition<T>::mulL(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) {
    Real sum = x(i);  //Lii = 1
    for(int j=0;j<i;j++)
      sum += LDL(i,j)*x(j);
    y(i) = sum;
  }
}

template <class T>
void LDLDecomposition<T>::mulLT(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) {
    Real sum = x(i);  //Lii = 1
    for(int j=i+1;j<n;j++)
      sum += LDL(j,i)*x(j);
    y(i) = sum;
  }
}

template <class T>
void LDLDecomposition<T>::mulD(const Vector& x,Vector& y) const
{
  int n=LDL.n;
  Assert(x.n == n);
  y.resize(n);
  for(int i=0;i<n;i++) y(i) = x(i)*LDL(i,i);
}

template <class T>
void LDLDecomposition<T>::getA(MatrixT& A) const
{
  MatrixT L,temp;
  DiagonalMatrixTemplate<T> D;
  getL(L);
  getD(D);
  D.postMultiply(L,temp);
  A.mulTransposeB(temp,L);
}

template<class T>
void LDLDecomposition<T>::update(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=LDL.n;
  Assert(x.n == n);

  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = LDL(i,i);
    T temp = alpha + Sqr(x(i))/deltai;
    deltai = deltai*temp;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    LDL(i,i) = deltai;
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*LDL(k,i);
      LDL(k,i) += gamma*x(k);
    }
  }
}

template <class T>
bool LDLDecomposition<T>::downdate(const VectorT& _x)
{
  VectorT x = _x;  //make a copy, we'll change it
  int n=LDL.n;
  Assert(x.n == n);

  T alpha=1;
  for(int i=0;i<n;i++) {
    T deltai = LDL(i,i);
    T temp = alpha - Sqr(x(i))/deltai;
    deltai = deltai*temp;
    if(FuzzyZero(deltai,zeroTolerance)) return false;
    T gamma = x(i)/deltai;
    deltai = deltai / alpha;
    alpha = temp;
    LDL(i,i) = deltai;
    for(int k=i+1;k<n;k++) {
      x(k) -= x(i)*LDL(k,i);
      LDL(k,i) -= gamma*x(k);
    }
  }
  return true;
}

template class LDLDecomposition<float>;
template class LDLDecomposition<double>;
//template class LDLDecomposition<Complex>;

} //namespace Math
