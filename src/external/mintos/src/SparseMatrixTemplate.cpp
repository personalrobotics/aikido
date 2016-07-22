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
#include <mintos/math/SparseMatrixTemplate.h>
#include "complex.h"
using namespace std;

namespace Math {

template <class T>
SparseMatrixTemplate_RM<T>::SparseMatrixTemplate_RM()
  :m(0),n(0)
{}

template <class T>
SparseMatrixTemplate_RM<T>::SparseMatrixTemplate_RM(int _m,int _n)
  :m(0),n(0)
{
  resize(_m,_n);
}

template <class T>
SparseMatrixTemplate_RM<T>::SparseMatrixTemplate_RM(const MyT& rhs)
  :m(0),n(0)
{
  copy(rhs);
}

template <class T>
void SparseMatrixTemplate_RM<T>::initialize(int _m, int _n)
{
  clear();
  resize(_m,_n);
}

template <class T>
void SparseMatrixTemplate_RM<T>::resize(int _m, int _n)
{
  if(m != _m || n != _n) {
    m = _m;
    n = _n;
    rows.resize(m);
    for(size_t i=0;i<rows.size();i++)
      rows[i].n = n;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::clear()
{
  m=n=0;
  rows.clear();
}

template <class T>
T& SparseMatrixTemplate_RM<T>::operator () (int i, int j)
{
  Assert(isValidRow(i));
  Assert(isValidCol(j));
  RowIterator entry=rows[i].find(j);
  if(entry != rows[i].end()) return entry->second;
  else {
    entry = rows[i].insert(j,(T)0);
    return entry->second;
  }
}

template <class T>
T* SparseMatrixTemplate_RM<T>::getEntry(int i, int j)
{
  Assert(isValidRow(i));
  Assert(isValidCol(j));
  RowIterator entry=rows[i].find(j);
  if(entry != rows[i].end()) {
    Assert(entry->first == j);
    return &(entry->second);
  }
  else return NULL;
}

template <class T>
const T* SparseMatrixTemplate_RM<T>::getEntry(int i,int j) const
{
  Assert(isValidRow(i));
  Assert(isValidCol(j));
  ConstRowIterator entry=rows[i].find(j);
  if(entry != rows[i].end()) {
    Assert(entry->first == j);
    return &(entry->second);
  }
  else return NULL;
}

template <class T>
void SparseMatrixTemplate_RM<T>::insertEntry(int i, int j, const T& val)
{
  Assert(isValidRow(i));
  Assert(isValidCol(j));
  rows[i].insert(j,val);
}

template <class T>
void SparseMatrixTemplate_RM<T>::eraseEntry(int i, int j)
{
  Assert(isValidRow(i));
  Assert(isValidCol(j));
  bool res = rows[i].erase(j);
  if(!res) {
    cerr<<"Warning, entry "<<i<<","<<j<<" doesn't exist"<<endl;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::copy(const MyT& A)
{
  m=A.m;
  n=A.n;
  rows=A.rows;
}

template <class T>
template <class T2>
void SparseMatrixTemplate_RM<T>::copy(const SparseMatrixTemplate_RM<T2>& A)
{
  initialize(A.m,A.n);
  for(int i=0;i<m;i++) {
    typename SparseMatrixTemplate_RM<T2>::ConstRowIterator it;
    for(it=A.rows[i].begin();it!=A.rows[i].end();it++)
      insertEntry(i,it->first,(T)it->second);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::copySubMatrix(int i, int j, const MyT& A)
{
  Assert(i >= 0 && j >= 0);
  Assert(i+A.m <= m);
  Assert(j+A.n <= n);
  for(int p=0;p<A.m;p++) {
    Assert((int)A.rows[p].n == A.n);

    //erase the items between j and j+A.n
    typename RowT::iterator first,last;
    first = rows[i+p].entries.lower_bound(j);
    last = rows[i+p].entries.upper_bound(j+A.n);
    if(first != rows[i+p].end())
      rows[i+p].entries.erase(first,last);

    //insert the new items
    typename RowT::const_iterator q;
    for(q=A.rows[p].begin();q!=A.rows[p].end();q++) {
      Assert(q->first >= 0 && q->first < A.n);
      Assert(q->first+j < n);
      rows[i+p].insert(q->first+j,q->second);
    }
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::copySubMatrix(int i, int j, const MatrixT& A, T zeroTol)
{
  Assert(i >= 0 && j >= 0);
  Assert(i+A.m <= m);
  Assert(j+A.n <= n);
  for(int p=0;p<A.m;p++) {
    for(int q=0;q<A.n;q++) {
      if(FuzzyZero(A(p,q),zeroTol)) 
	rows[i+p].erase(q+j);
      else
	rows[i+p].insert(q+j,A(p,q));
    }
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::swap(MyT& A)
{
  std::swap(rows,A.rows);
  std::swap(m,A.m);
  std::swap(n,A.n);
}

template <class T>
void SparseMatrixTemplate_RM<T>::set(const MatrixT& A,T zeroTol)
{
  resize(A.m,A.n);
  setZero();
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++)
      if(!FuzzyZero(A(i,j),zeroTol)) rows[i].insert(j,A(i,j));
}

template <class T>
void SparseMatrixTemplate_RM<T>::setZero()
{
  for(size_t i=0;i<rows.size();i++)
    rows[i].entries.clear();
}

template <class T>
void SparseMatrixTemplate_RM<T>::setIdentity()
{
  Assert(m == n);
  for(int i=0;i<m;i++) {
    rows[i].entries.clear();
    rows[i].insert(i,1);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::setNegative(const MyT& A)
{
  resize(A.n,A.m);
  setZero();
  for(int i=0;i<A.m;i++) {
    for(ConstRowIterator it=A.rows[i].begin();it!=A.rows[i].end();it++)
      insertEntry(i,it->first,-it->second);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::setTranspose(const MyT& A)
{
  resize(A.n,A.m);
  setZero();
  for(int i=0;i<A.m;i++) {
    for(ConstRowIterator it=A.rows[i].begin();it!=A.rows[i].end();it++)
      insertEntry(it->first,i,it->second);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::setAdjoint(const MyT& A)
{
  setTranspose(A);
}

template <class T>
void SparseMatrixTemplate_RM<T>::get(MatrixT& A) const
{
  A.resize(m,n,Zero);
  for(int i=0;i<m;i++) {
    for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
      A(i,it->first) = it->second;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::mul(const MyT& a, T s)
{
  copy(a);
  inplaceMul(s);
}

template <class T>
void SparseMatrixTemplate_RM<T>::mul(const VectorT& a,VectorT& x) const
{
  if(x.n == 0) x.resize(m);
  if(x.n != m) {
    FatalError("Destination vector has incorrect dimensions");
  }
  if(a.n != n) {
    FatalError("Source vector has incorrect dimensions");
  }
  for(int i=0;i<m;i++) {
    T sum=0;
    for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
      sum += it->second*a(it->first);
    x(i) = sum;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::madd(const VectorT& a,VectorT& x) const
{
  if(x.n != m) {
    FatalError("Destination vector has incorrect dimensions");
  }
  if(a.n != n) {
    FatalError("Source vector has incorrect dimensions");
  }
  for(int i=0;i<m;i++) {
    T sum=0;
    for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
      sum += it->second*a(it->first);
    x(i) += sum;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::mulTranspose(const VectorT& a,VectorT& x) const
{
  if(x.n == 0) x.resize(n);
  if(x.n != n) {
    FatalError("Destination vector has incorrect dimensions");
  }
  if(a.n != m) {
    FatalError("Source vector has incorrect dimensions");
  }
  x.setZero();
  for(int i=0;i<m;i++) {
    for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
      x(it->first) += it->second*a(i);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::maddTranspose(const VectorT& a,VectorT& x) const
{
  if(x.n != n) {
    FatalError("Destination vector has incorrect dimensions");
  }
  if(a.n != m) {
    FatalError("Source vector has incorrect dimensions");
  }
  for(int i=0;i<m;i++) {
    for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
      x(it->first) += it->second*a(i);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::mul(const MatrixT& a,MatrixT& x) const
{
  if(a.m != m) {
    FatalError("A matrix has incorrect # of rows");
  }
  if(x.isEmpty()) x.resize(m,a.n);
  if(m != x.m) {
    FatalError("X matrix has incorrect # of rows");
  }
  if(a.n != x.n) {
    FatalError("X matrix has incorrect # of columns");
  }
  for(int i=0;i<a.n;i++) {
    VectorT ai,xi;
    a.getColRef(i,ai);
    x.getColRef(i,xi);
    mul(ai,xi);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::mulTranspose(const MatrixT& a,MatrixT& x) const
{
  if(a.m != n) {
    FatalError("A matrix has incorrect # of rows");
  }
  if(x.isEmpty()) x.resize(n,a.n);
  if(n != x.m) {
    FatalError("X matrix has incorrect # of rows");
  }
  if(a.n != x.n) {
    FatalError("X matrix has incorrect # of columns");
  }
  for(int i=0;i<a.n;i++) {
    VectorT ai,xi;
    a.getColRef(i,ai);
    x.getColRef(i,xi);
    mulTranspose(ai,xi);
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::copyRow(int i,const VectorT& x,T zeroTol)
{
  Assert(isValidRow(i));
  Assert(x.n == n);
  rows[i].entries.clear();
  for(int j=0;j<x.n;j++)
    if(!FuzzyZero(x(j),zeroTol)) rows[i].insert(j,x(j));
}

template <class T>
void SparseMatrixTemplate_RM<T>::copyRow(int i,const SparseVectorT& x)
{
  Assert(isValidRow(i));
  Assert((int)x.n == n);
  rows[i].entries.clear();
  rows[i].entries = x.entries;
}

template <class T>
void SparseMatrixTemplate_RM<T>::copyCol(int j,const VectorT& x,T zeroTol)
{
  Assert(isValidCol(j));
  Assert(x.n == m);
  for(int i=0;i<x.n;i++) {
    if(FuzzyZero(x(i),zeroTol)) rows[i].erase(j);
    else rows[i].insert(j,x(i));
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::copyCol(int j,const SparseVectorT& x)
{
  Assert(isValidCol(j));
  Assert((int)x.n == m);
  int n=0;
  for(typename SparseVectorT::const_iterator it=x.begin();it!=x.end();it++) {
    for(int i=n;i<it->first;i++) rows[i].erase(j);
    rows[it->first].insert(j,it->second);
    n=it->first+1;
  }
  for(int i=n;i<m;i++)
    rows[i].erase(j);
}

template <class T>
T SparseMatrixTemplate_RM<T>::dotRow(int i, const VectorT& v) const
{
  Assert(isValidRow(i));
  Assert(v.n == n);
  T sum=0;
  for(ConstRowIterator it=rows[i].begin();it!=rows[i].end();it++)
    sum += v(it->first)*it->second;
  return sum;
}
template <class T>
T SparseMatrixTemplate_RM<T>::dotCol(int j, const VectorT& v) const
{
  Assert(isValidCol(j));
  Assert(v.n == m);
  T sum=0;
  for(int i=0;i<m;i++) {
    ConstRowIterator it=rows[i].find(j);
    if(it != rows[i].end()) sum += v(i)*it->second;
  }
  return sum;
}

template <class T>
void SparseMatrixTemplate_RM<T>::inplaceNegative()
{
  for(int i=0;i<m;i++) {
    for(RowIterator it=rows[i].begin();it!=rows[i].end();it++)
      it->second = -it->second;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::inplaceMul(T c)
{
  for(int i=0;i<m;i++) {
    for(RowIterator it=rows[i].begin();it!=rows[i].end();it++)
      it->second *= c;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::inplaceDiv(T c)
{
  for(int i=0;i<m;i++) {
    for(RowIterator it=rows[i].begin();it!=rows[i].end();it++)
      it->second /= c;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::inplaceMulRow(int i,T c)
{
  Assert(isValidRow(i));
  for(RowIterator it=rows[i].begin();it!=rows[i].end();it++)
    it->second *= c;
}

template <class T>
void SparseMatrixTemplate_RM<T>::inplaceMulCol(int j,T c)
{
  Assert(isValidCol(j));
  for(int i=0;i<m;i++) {
    RowIterator it=rows[i].find(j);
    if(it != rows[i].end()) it->second *= c;
  }
}

template <class T>
void SparseMatrixTemplate_RM<T>::eraseZeros(T zeroTol)
{
  for(size_t i=0;i<rows.size();i++) {
    RowT temp;
    temp.n = n;
    typename RowT::const_iterator j;
    for(j=rows[i].begin();j!=rows[i].end();j++)
      if(Abs(j->second) > Abs(zeroTol)) temp.insert(j->first,j->second);
    rows[i].entries.clear();
    rows[i] = temp;
  }
}

template <class T>
bool SparseMatrixTemplate_RM<T>::isValid() const
{
  if(m != (int)rows.size()) return false;
  for(size_t i=0;i<rows.size();i++) {
    if((int)rows[i].n != n) return false;
    if(!rows[i].isValid()) return false;
  }
  return true;
}

template <class T>
size_t SparseMatrixTemplate_RM<T>::numNonZeros() const
{
  size_t nnz=0;
  for(size_t i=0;i<rows.size();i++)
    nnz += rows[i].numEntries();
  return nnz;
}

template <class T>
std::ostream& operator << (std::ostream& out, const SparseMatrixTemplate_RM<T>& A)
{
  out<<A.m<<" "<<A.n<<" "<<A.numNonZeros()<<endl;
  for(size_t i=0;i<A.rows.size();i++) {
    typename SparseMatrixTemplate_RM<T>::ConstRowIterator it;
    for(it=A.rows[i].begin();it!=A.rows[i].end();it++)
      out<<i<<" "<<it->first<<"   "<<it->second<<endl;
  }
  return out;
}

template <class T>
std::istream& operator >> (std::istream& in, SparseMatrixTemplate_RM<T>& A)
{
  int m,n,nnz;
  in >> m >> n >> nnz;
  if(in.bad()) return in;
  A.resize(m,n);
  for(int i=0;i<nnz;i++) {
    int row,col; T val;
    in >> row >> col >> val;
    if(in.bad()) return in;
    A(row,col) = val;
  }
  return in;
}


//specialization for complex
template <> void SparseMatrixTemplate_RM<Complex>::setAdjoint(const MyT& A)
{
  setTranspose(A);
  for(int i=0;i<m;i++)
    for(RowIterator j=rows[i].begin();j!=rows[i].end();j++)
      j->second.inplaceConjugate();
}

template class SparseMatrixTemplate_RM<float>;
template class SparseMatrixTemplate_RM<double>;
template class SparseMatrixTemplate_RM<Complex>;
template ostream& operator << (ostream& out, const SparseMatrixTemplate_RM<float>& v);
template ostream& operator << (ostream& out, const SparseMatrixTemplate_RM<double>& v);
template ostream& operator << (ostream& out, const SparseMatrixTemplate_RM<Complex>& v);
template istream& operator >> (istream& in, SparseMatrixTemplate_RM<float>& v);
template istream& operator >> (istream& in, SparseMatrixTemplate_RM<double>& v);
template istream& operator >> (istream& in, SparseMatrixTemplate_RM<Complex>& v);

template void SparseMatrixTemplate_RM<float>::copy(const SparseMatrixTemplate_RM<double>& a);
template void SparseMatrixTemplate_RM<double>::copy(const SparseMatrixTemplate_RM<float>& a);
template void SparseMatrixTemplate_RM<Complex>::copy(const SparseMatrixTemplate_RM<float>& a);
template void SparseMatrixTemplate_RM<Complex>::copy(const SparseMatrixTemplate_RM<double>& a);

} // namespace Math
