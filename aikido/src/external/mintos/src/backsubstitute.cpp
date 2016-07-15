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
#include "backsubstitute.h"
#include "complex.h"
#include "MatrixPrinter.h"
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {

const static Real kBackSubZeroTolerance = (Real)1e-4;

template <class T>
bool UBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T aii,sum;
	for(int i=n-1; i>=0; i--) {
		aii=a(i,i);
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(i,j)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //cerr<<"UBackSubstitute: dividing by zero: "<<sum<<"/"<<aii<<endl;
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

// If A is lower triangular nxn, solves Ax=b
template <class T>
bool LBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T aii,sum;
	for(int i=0; i<n; i++) {
		aii=a(i,i);
		sum=b[i];
		for(int j=0; j<i; j++)
			sum-=a(i,j)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //cerr<<"LBackSubstitute: dividing by zero: "<<sum<<"/"<<aii<<endl;
		    //cerr<<MatrixPrinter(a)<<endl;
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

// If A is lower triangular nxn, solves A^t*x=b
template <class T>
bool LtBackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{ 
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
 	T aii,sum;
	for(int i=n-1; i>=0; i--) {
		aii=a(i,i);
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(j,i)*x[j];
		if(aii == 0) {
		  if(!FuzzyZero(sum,(T)kBackSubZeroTolerance)) {
		    //cerr<<"LtBackSubstitute: dividing by zero: "<<sum<<"/"<<aii<<endl;
		    return false;
		  }
		  x[i]=0;
		}
		else
		  x[i]=sum/aii;
	}
	return true;
}

template <class T>
void U1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{ 
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
  T sum;
	for(int i=n-1; i>=0; i--) {
		sum=b(i);
		for(int j=i+1; j<n; j++)
			sum-=a(i,j)*x[j];
    x[i]=sum;
	}
}

template <class T>
void L1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
	T sum;
	for(int i=0; i<n; i++) {
		sum=b[i];
		for(int j=0; j<i; j++)
			sum-=a(i,j)*x[j];
		x[i]=sum;
	}
}

template <class T>
void Lt1BackSubstitute(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x)
{
  Assert(a.isSquare());
  Assert(a.n == b.n);
  if(x.empty()) x.resize(a.n);
  else Assert(a.n == x.n);
  int n=a.n;
  T sum;
	for(int i=n-1; i>=0; i--) {
		sum=b[i];
		for(int j=i+1; j<n; j++)
			sum-=a(j,i)*x[j];
		x[i]=sum;
	}
}

template <class T>
inline bool UBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!UBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline bool LBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!LBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline bool LtBackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    if(!LtBackSubstitute(a,bi,xi)) return false;
  }
  return true;
}
template <class T>
inline void U1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    U1BackSubstitute<T>(a,bi,xi);
  }
}
template <class T>
inline void L1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    L1BackSubstitute(a,bi,xi);
  }
}
template <class T>
inline void Lt1BackSubstitute(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x)
{
  if(x.isEmpty()) 
    x.resize(a.n,b.n);
  else Assert(x.m == a.n && x.n == b.n);
  for(int i=0;i<x.n;i++) {
    VectorTemplate<T> xi,bi;
    x.getColRef(i,xi);
    b.getColRef(i,bi);
    Lt1BackSubstitute(a,bi,xi);
  }
}

#define DEFINEBACKSUBSTITUTE(T) \
template bool UBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool LBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool LtBackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void U1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void L1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template void Lt1BackSubstitute<T>(const MatrixTemplate<T>& a, const VectorTemplate<T>& b, VectorTemplate<T>& x); \
template bool UBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template bool LBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template bool LtBackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void U1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void L1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x); \
template void Lt1BackSubstitute<T>(const MatrixTemplate<T>& a, const MatrixTemplate<T>& b, MatrixTemplate<T>& x);

DEFINEBACKSUBSTITUTE(float);
DEFINEBACKSUBSTITUTE(double);
DEFINEBACKSUBSTITUTE(Complex);

} //namespace Math
