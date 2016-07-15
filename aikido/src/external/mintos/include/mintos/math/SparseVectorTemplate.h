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

#ifndef MATH_SPARSE_VECTOR_TEMPLATE_H
#define MATH_SPARSE_VECTOR_TEMPLATE_H

#include "VectorTemplate.h"
#include "SparseArray.h"

namespace Math {

template <class T>
struct SparseVectorAccessor;

template <class T>
class SparseVectorTemplate : public SparseArray<T>
{
public:
  typedef SparseVectorTemplate<T> MyT;
  typedef SparseArray<T> BaseT;
  typedef typename BaseT::iterator iterator;
  typedef typename BaseT::const_iterator const_iterator;
  typedef VectorTemplate<T> VectorT;

  SparseVectorTemplate() {}
  SparseVectorTemplate(size_t n):BaseT(n) {}
  SparseVectorTemplate(const BaseT& v):BaseT(v) {}

  void print(std::ostream&) const;

  SparseVectorAccessor<T> operator() (int i);
  SparseVectorAccessor<T> operator[] (int i) { return operator()(i); }
  T operator() (int i) const { return get(i); }
  T operator[] (int i) const { return get(i); }
  T get(int i) const;
  inline void set(int i,const T& t) { BaseT::insert(i,t); }

  inline void setZero() { BaseT::entries.clear(); }
  inline void set(const BaseT& v) { BaseT::operator = (v); }
  void set(const VectorT&,T zeroTol=Zero);
  void set(const T*,int n,T zeroTol=Zero);
  void get(T*) const;
  void get(VectorT&) const;
  void inplaceNegative();
  void inplaceMul(T s);
  void inplaceDiv(T s);

  void copy(const MyT&);
  void copySubVector(int i,const MyT&);
  void copySubVector(int i,const VectorT&,T zeroTol=0);
  void swap(MyT&);
  void add(const MyT&, const MyT&);
  void sub(const MyT&, const MyT&);
  void mul(const MyT&, T s);
  void div(const MyT&, T s);

  T dot(const VectorT&) const;
  T dot(const MyT&) const;
  T norm() const;
  T normSquared() const;
  T distance(const MyT&) const;
  T distanceSquared(const MyT&) const;

  inline bool isEmpty() const { return BaseT::empty(); }
  inline bool hasDims(size_t _n) const { return BaseT::n==_n; }

  T minElement(int* index=NULL) const;
  T maxElement(int* index=NULL) const;
  T minAbsElement(int* index=NULL) const;
  T maxAbsElement(int* index=NULL) const;
};

template <class T>
class SparseVectorCompressed 
{
public:
  typedef SparseVectorCompressed<T> MyT;
  typedef VectorTemplate<T> VectorT;

  SparseVectorCompressed();
  SparseVectorCompressed(int n, int num_entries);
  SparseVectorCompressed(const MyT&);
  ~SparseVectorCompressed();
  void init(int n, int num_entries);
  void resize(int n, int num_entries);
  void makeSimilar(const MyT&);
  void cleanup();

  void print(std::ostream&) const;

  const MyT& operator =(const MyT&);
  T operator() (int i) const;

  void setZero();
  void set(const MyT&);
  void set(const VectorT&,T zeroTol=Zero);
  void set(const T*,int n,T zeroTol=Zero);
  void get(T*) const;
  void get(VectorT&) const;
  void inplaceNegative();
  void inplaceMul(T s);
  void inplaceDiv(T s);

  void add(const MyT&, const MyT&);
  void sub(const MyT&, const MyT&);
  void mul(const MyT&, T s);
  void div(const MyT&, T s);

  T dot(const VectorT&) const;
  T dot(const MyT&) const;
  T norm() const;
  T normSquared() const;
  T distance(const MyT&) const;
  T distanceSquared(const MyT&) const;

  bool isValid() const;
  inline bool isEmpty() const { return n == 0; }
  inline bool hasDims(int _n) const { return n==_n; }
  inline bool isValidIndex(int i) const { return 0<=i&&i<n; }

  T minElement(int* index=NULL) const;
  T maxElement(int* index=NULL) const;
  T minAbsElement(int* index=NULL) const;
  T maxAbsElement(int* index=NULL) const;

  /***********************************************************
   * Compressed vector format:
   * There are num_entries nonzero entries .
   * The index of entry i is indices[i] (0<=indices[i]<indices[i+1]<n)
   * The value of entry i is vals[i]
   **********************************************************/
  int* indices;
  T* vals;
  int num_entries;
  int n;
};

template <class T>
struct SparseVectorAccessor
{
  SparseVectorAccessor(SparseVectorTemplate<T>* _vec,int _index)
    :vec(_vec),index(_index)
  {}
  SparseVectorAccessor(const SparseVectorAccessor<T>& rhs)
    :vec(rhs.vec),index(rhs.index)
  {}
  operator T () const
  {
    return vec->get(index); 
  }
  const SparseVectorAccessor<T>& operator = (const T& rhs) {
    vec->set(index,rhs); 
    return *this; 
  }
 
  SparseVectorTemplate<T>* vec;
  int index;
};

///returns true if all elements of x are finite
template <class T>
inline bool IsFinite(const SparseVectorTemplate<T>& x)
{
  for(typename SparseVectorTemplate<T>::const_iterator i=x.begin();i!=x.end();i++)
    if(!IsFinite(i->second)) return false;
  return true;
}

///returns true if any element of x is NaN
template <class T>
inline bool HasNaN(const SparseVectorTemplate<T>& x)
{
  for(typename SparseVectorTemplate<T>::const_iterator i=x.begin();i!=x.end();i++)
    if(IsNaN(i->second)) return false;
  return true;
}

///returns nonzero if any element of x is infininte
template <class T>
inline bool HasInf(const SparseVectorTemplate<T>& x)
{
  for(typename SparseVectorTemplate<T>::const_iterator i=x.begin();i!=x.end();i++)
    if(IsInf(i->second)) return IsInf(i->second);
  return 0;
}

class Complex;
typedef SparseVectorTemplate<float> fSparseVector;
typedef SparseVectorTemplate<double> dSparseVector;
typedef SparseVectorTemplate<Complex> cSparseVector;


} //namespace Math

#endif
