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
#include <mintos/math/VectorTemplate.h>
#include "fastarray.h"
#include "complex.h"
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {

#define CHECKEMPTY() { Assert(!empty()); }
#define CHECKSIZE(_n) { Assert(size()==_n); }
#define CHECKRESIZE(_n) { if(empty()) resize(_n); else Assert(size()==_n); }


template <class T>
VectorTemplate<T>::VectorTemplate()
:vals(NULL),capacity(0),allocated(false),
base(0),stride(1),n(0)
{}

template <class T>
VectorTemplate<T>::VectorTemplate(const MyT& v)
:vals(NULL),capacity(0),allocated(false),
base(0),stride(1),n(0)
{
  copy(v);
}
/*
template <class T>
VectorTemplate<T>::VectorTemplate(const MyT& v)
:vals(v.vals),capacity(v.capacity),allocated(false),
base(v.base),stride(v.stride),n(v.n)
{
}
*/

template <class T>
VectorTemplate<T>::VectorTemplate(int _n)
:vals(NULL),capacity(0),allocated(false),
base(0),stride(0),n(0)
{
	resize(_n);
}

template <class T>
VectorTemplate<T>::VectorTemplate(int _n, T initval)
:vals(NULL),capacity(0),allocated(false),
base(0),stride(0),n(0)
{
	resize(_n, initval);
}

template <class T>
VectorTemplate<T>::VectorTemplate(int _n, const T* _vals)
:vals(NULL),capacity(0),allocated(false),
base(0),stride(0),n(0)
{
	resize(_n);
	copy(_vals);
}

template <class T>
VectorTemplate<T>::VectorTemplate(const std::vector<T>& _vals)
:vals(NULL),capacity(0),allocated(false),
base(0),stride(0),n(0)
{
  copy(_vals);
}


template <class T>
VectorTemplate<T>::~VectorTemplate()
{
  clear();
}

template <class T>
void VectorTemplate<T>::resize(int _n)
{
  Assert(_n >= 0);
  if(size() != _n) {
    if(allocated) {
      if(!isCompact()) {
	cout<<"base "<<base<<endl;
	cout<<"stride "<<stride<<endl;
	cout<<"n "<<n<<endl;
      }
      Assert(isCompact());
    }
    else {
      Assert(empty());
      clear();
    }
    if(_n > capacity) {
      SafeArrayDelete(vals);
      vals = new T[_n];
      capacity = _n;
      if(!vals) {
	FatalError("Not enough memory to allocate vector of size %d",_n);
      }
    }
    base = 0;
    stride = 1;
    n = _n;
    allocated = true;
  }
}

template <class T>
void VectorTemplate<T>::resize(int _n, T initval)
{
	resize(_n);
	if(_n != 0) set(initval);
}

template <class T>
void VectorTemplate<T>::resizePersist(int _n)
{
  Assert(_n >= 0);
  if(size() != _n) {
    if(allocated) {
      if(!isCompact()) {
	cout<<"base "<<base<<endl;
	cout<<"stride "<<stride<<endl;
	cout<<"n "<<n<<endl;
      }
      Assert(isCompact());
    }
    else {
      Assert(empty());
      clear();
    }
    if(_n > capacity) {
      T* oldvals = vals;
      vals = new T[_n];
      capacity = _n;
      if(!vals) {
	FatalError("Not enough memory to allocate vector of size %d",_n);
      }
      //copy n values 
      gen_array_equal(vals, 1, oldvals, stride, n);
      SafeArrayDelete(oldvals);
    }
    base = 0;
    stride = 1;
    n = _n;
    allocated = true;
  }
}

template <class T>
void VectorTemplate<T>::resizePersist(int _n, T initval)
{
  int oldn = n;
  resizePersist(_n);
  if(_n > oldn) {
    gen_array_fill(vals+oldn*stride,stride,initval,_n-oldn);
  }
}


template <class T>
void VectorTemplate<T>::clear()
{
  if(allocated) {
    SafeArrayDelete(vals);
  }
  else {
    vals=NULL;
  }
  capacity = 0;
  allocated = false;
  base=0;
  stride=1;
  n=0;
}

template <class T>
VectorTemplate<T>::operator std::vector<T> () const
{
  std::vector<T> res(n);
  for(int i=0;i<n;i++)
    res[i] = this->operator()(i);
  return res;
}

template <class T>
const VectorTemplate<T>& VectorTemplate<T>::operator = (const MyT& a)
{
  if(this == &a) return *this;
  if(n!=a.n) resize(a.n);
  gen_array_equal(getStart(),stride, a.getStart(),a.stride, n);
  return *this;
}

template <class T>
bool VectorTemplate<T>::operator == (const MyT& v) const
{
  if(this == &v) return true;
  if(size() != v.n) return false;
  return std::equal(begin(),end(),v.begin());
}

template <class T>
void VectorTemplate<T>::swap(MyT& a)
{
  std::swap(vals,a.vals);
  std::swap(capacity,a.capacity);
  std::swap(allocated,a.allocated);
  std::swap(base,a.base);
  std::swap(stride,a.stride);
  std::swap(n,a.n);
}

template <class T>
void VectorTemplate<T>::swapCopy(MyT& a)
{
  CHECKSIZE(a.n);
  T tmp;
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<n;i++,v++,va++) {
    tmp = *v;  *v = *va;  *va = tmp;
  }
}

template <class T>
void VectorTemplate<T>::copy(const MyT& a)
{
  if(this == &a) return;
  CHECKRESIZE(a.n);
  gen_array_equal(getStart(),stride, a.getStart(),a.stride, n);
}

template <class T>
template <class T2>
void VectorTemplate<T>::copy(const std::vector<T2>& val)
{
  CHECKRESIZE((int)val.size());
  VectorIterator<T> k=begin();
  typename std::vector<T2>::const_iterator vk=val.begin();
  for(int i=0;i<n;i++,k++,vk++)
    *k = (T)*vk;
}


template <class T>
template <class T2>
void VectorTemplate<T>::copy(const VectorTemplate<T2>& a)
{
  CHECKRESIZE(a.n);
  VectorIterator<T> k=begin();
  VectorIterator<T2> ak=a.begin();
  for(int i=0;i<n;i++,k++,ak++)
    *k = (T)*ak;
}

template <class T>
void VectorTemplate<T>::copy(const T* _vals)
{
	CHECKEMPTY();
  gen_array_equal(getStart(),stride, _vals,1, n);
}

template <class T>
void VectorTemplate<T>::copySubVector(int i,const MyT& a)
{
  Assert(this != &a);
  Assert(isValidIndex(i));
  Assert(isValidIndex(i+a.n-1));
	gen_array_equal(getStart()+i*stride,stride, a.getStart(),a.stride, a.n);
}

template <class T>
void VectorTemplate<T>::add(const MyT& a, const MyT& b)
{
  Assert(a.size() == b.size());
  CHECKRESIZE(a.n);
	gen_array_add(getStart(),stride, a.getStart(),a.stride, b.getStart(),b.stride, n);
}

template <class T>
void VectorTemplate<T>::sub(const MyT& a, const MyT& b)
{
  Assert(a.size() == b.size());
  CHECKRESIZE(a.n);
	gen_array_sub(getStart(),stride, a.getStart(),a.stride, b.getStart(),b.stride, n);
}

template <class T>
void VectorTemplate<T>::mul(const MyT& a, T c)
{
	CHECKRESIZE(a.n);
	gen_array_mul(getStart(),stride, a.getStart(),a.stride, c, n);
}

template <class T>
void VectorTemplate<T>::div(const MyT& a, T c)
{
	CHECKRESIZE(a.n);
	gen_array_div(getStart(),stride, a.getStart(),a.stride, c, n);
}

template<class T>
void VectorTemplate<T>::axpby(T a,const MyT& x,T b,const MyT& y)
{
  Assert(x.n == y.n);
  CHECKRESIZE(x.n);
  gen_array_axpby(getStart(),stride, a, x.getStart(),x.stride, b, y.getStart(),y.stride,n);
}

template <class T>
void VectorTemplate<T>::inc(const T& c)
{
	CHECKEMPTY();
  gen_array_acc(getStart(),stride, &c,0, n);
}

template <class T>
void VectorTemplate<T>::inc(const MyT& v)
{
  Assert(size() == v.size());
  gen_array_acc(getStart(),stride, v.getStart(),v.stride, n);
}

template <class T>
void VectorTemplate<T>::dec(const MyT& v)
{
	Assert(size() == v.size());
  gen_array_dec(getStart(),stride, v.getStart(),v.stride, n);
}

template <class T>
void VectorTemplate<T>::madd(const MyT& v, T c)
{
	Assert(size() == v.size());
  gen_array_madd(getStart(),stride, v.getStart(),v.stride, c,n);
}





template <class T>
void VectorTemplate<T>::setRef(const MyT& a,int _base,int _stride,int _size)
{
  Assert(this != &a);
  Assert(!allocated);
  vals = a.vals;
  capacity = a.capacity;
  allocated = false;
  base = a.base + a.stride*_base;
  stride = a.stride * _stride;
  if(_size < 0) {
    Assert(stride != 0);
    //max n s.t. (n-1)*_stride < a.n-_base
    //n*_stride < a.n-_base+_stride
    //n*_stride <= a.n-_base+_stride-1
    n = (a.n - _base + _stride-1)/_stride;
  }
  else n=_size;
  Assert(isValid());
}

template <class T>
void VectorTemplate<T>::setRef(T* _vals,int _capacity,int _base,int _stride,int _size)
{
  Assert(!allocated);
  vals = _vals;
  capacity = _capacity;
  allocated = false;
  base = _base;
  stride = _stride;
  if(_size < 0) {
    Assert(stride != 0);
    n = (capacity - base)/stride;
  }
  else n=_size;
  Assert(isValid());
}

template <class T>
void VectorTemplate<T>::set(T c)
{
	CHECKEMPTY();
  gen_array_fill(getStart(),stride,c,n);
}

template <class T>
void VectorTemplate<T>::setZero()
{
	set((T)0);
}

template <class T>
void VectorTemplate<T>::setNegative(const MyT& a)
{
  CHECKRESIZE(a.n);
  gen_array_negate(getStart(),stride, a.getStart(),a.stride, n);
}

template <class T>
void VectorTemplate<T>::setNormalized(const MyT& a)
{
	mul(a, PseudoInv(norm()));
}

template <class T>
void VectorTemplate<T>::setConjugate(const MyT& a)
{
  copy(a);
}


template <class T>
void VectorTemplate<T>::inplaceNegative()
{
  CHECKEMPTY();
  gen_array_negate(getStart(),stride, getStart(),stride, n);
}

template <class T>
void VectorTemplate<T>::inplaceMul(T c)
{
	CHECKEMPTY();
  gen_array_mul(getStart(),stride,c,n);
}

template <class T>
void VectorTemplate<T>::inplaceDiv(T c)
{
	CHECKEMPTY();
  gen_array_div(getStart(),stride,c,n);
}

template <class T>
void VectorTemplate<T>::inplaceNormalize()
{
	inplaceMul(PseudoInv(norm()));
}

template <class T>
void VectorTemplate<T>::inplaceConjugate()
{
  CHECKEMPTY();
}


template <class T>
void VectorTemplate<T>::getCopy(MyT& v) const
{
  v.copy(*this);
}

template <class T>
void VectorTemplate<T>::getCopy(T* _vals) const
{
  CHECKEMPTY();
  gen_array_equal(_vals,1, getStart(),stride, n);
}

template <class T>
void VectorTemplate<T>::getSubVectorCopy(int i,MyT& a) const
{
  Assert(&a != this);
  Assert(isValidIndex(i));
  Assert(isValidIndex(i+a.n-1));
  gen_array_equal(a.getStart(),a.stride, getStart()+stride*i,stride, a.n);
}

template <class T>
void VectorTemplate<T>::getRef(MyT& v,int _base,int _stride,int _size) const
{
  v.setRef(*this,_base,_stride,_size);
}


template <class T>
bool VectorTemplate<T>::isValid() const
{
  if(base < 0) {
    cout<<"VectorTemplate::isValid(): Base is negative"<<endl;
    return false;
  }
  if(n > 0) {
    if(base + stride*(n-1) >= capacity) {
      cout<<"base "<<base<<endl;
      cout<<"stride "<<stride<<endl;
      cout<<"n "<<n<<endl;
      cout<<"VectorTemplate::isValid(): max element exceeds bounds"<<endl;
      return false;
    }
    if(stride < 0) {
      cout<<"VectorTemplate::isValid(): stride is negative"<<endl;
    }
  }
  return true;
}

template <class T>
bool VectorTemplate<T>::isZero(T eps) const
{
  CHECKEMPTY();
  ItT v=begin();
  for(int i=0;i<n;i++,v++)
    if(!FuzzyZero(*v,eps)) return false;
  return true;
}

template <class T>
bool VectorTemplate<T>::isEqual(const MyT& a,T eps) const
{
	CHECKEMPTY();
	Assert(size()==a.size());
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<n;i++,v++,va++)
    if(!FuzzyEquals(*v,*va,eps)) return false;
  return true;
}

template <class T>
T VectorTemplate<T>::dot(const MyT& a) const
{
  CHECKEMPTY();
  Assert(size()==a.size());
  return gen_array_dot(getStart(),stride, a.getStart(),a.stride, n);
}

template <class T>
T VectorTemplate<T>::dotSelf() const 
{
  CHECKEMPTY();
  return gen_array_norm_squared(getStart(),stride,n);
}

template <class T>
T VectorTemplate<T>::norm() const
{
  return Sqrt(normSquared()); 
}

template <class T>
T VectorTemplate<T>::normSquared() const 
{
  return dotSelf();
}

template <class T>
T VectorTemplate<T>::distance(const MyT& a) const
{
  return Sqrt(distanceSquared(a));
}

template <class T>
T VectorTemplate<T>::distanceSquared(const MyT& a) const
{
  CHECKSIZE(a.n);
  ItT v=begin();
  ItT va=a.begin();
  T sum=0;
  for(int i=0;i<n;i++,v++,va++) {
    T d=*v-*va;
    sum += Math::dot(d,d);
  }
  return sum;
}

template <class T>
T VectorTemplate<T>::minElement(int* index) const
{
  CHECKEMPTY();
  ItT v=begin();
  T b=*v;
  if(index) *index=0;
  v++;
  for(int i=1;i<n;i++,v++)
    if(*v < b) {
      b=*v;
      if(index) *index=i;
    }
  return b;
}

template <class T>
T VectorTemplate<T>::maxElement(int *index) const
{
  CHECKEMPTY();
  ItT v=begin();
  T b=*v;
  if(index) *index=0;
  v++;
  for(int i=1;i<n;i++,v++)
    if(*v > b) {
      b=*v;
      if(index) *index=i;
    }
  return b;
}

template <class T>
T VectorTemplate<T>::minAbsElement(int* index) const
{
  CHECKEMPTY();
  ItT v=begin();
  T b=Abs(*v);
  if(index) *index=0;
  v++;
  for(int i=1;i<n;i++,v++)
    if(Abs(*v) < b) {
      b=Abs(*v);
      if(index) *index=i;
    }
  return b;
}

template <class T>
T VectorTemplate<T>::maxAbsElement(int *index) const
{
  CHECKEMPTY();
  ItT v=begin();
  T b=Abs(*v);
  if(index) *index=0;
  v++;
  for(int i=1;i<n;i++,v++)
    if(Abs(*v) > b) {
      b=Abs(*v);
      if(index) *index=i;
    }
  return b;
}





template <class T>
ostream& operator << (ostream& out, const VectorTemplate<T>& v)
{
	out << v.n << "\t";
	for(int i=0; i<v.n; i++)
		out << v[i] << " ";
	return out;
}

template <class T>
istream& operator >> (istream& in, VectorTemplate<T>& v)
{
	int n;
	in >> n;
	if(!in) return in;
  if(n != v.n)
  	v.resize(n);
	for(int i=0; i<v.n; i++)
		in >> v[i];
	return in;
}

template <class T>
void VectorTemplate<T>::componentMul(const MyT& a,const MyT& b)
{
  Assert(a.size()==b.size());
  CHECKRESIZE(a.n);
  T* v=getStart();
  T* va=a.getStart();
  T* vb=b.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride,vb+=b.stride)
    *v = (*va)*(*vb);
}

template <class T>
void VectorTemplate<T>::componentDiv(const MyT& a,const MyT& b)
{
  Assert(a.size()==b.size());
  CHECKRESIZE(a.n);
  T* v=getStart();
  T* va=a.getStart();
  T* vb=b.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride,vb+=b.stride)
    *v = (*va)/(*vb);
}

template <class T>
void VectorTemplate<T>::componentMadd(const MyT& a,const MyT& b)
{
  Assert(a.size()==b.size());
  CHECKRESIZE(a.n);
  T* v=getStart();
  T* va=a.getStart();
  T* vb=b.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride,vb+=b.stride)
    *v += (*va)*(*vb);
}

template <class T>
void VectorTemplate<T>::inplaceComponentMul(const MyT& a)
{
  CHECKSIZE(a.n);
  T* v=getStart();
  T* va=a.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride)
    *v *= (*va);
}

template <class T>
void VectorTemplate<T>::inplaceComponentDiv(const MyT& a)
{
  CHECKSIZE(a.n);
  T* v=getStart();
  T* va=a.getStart();
  for(int i=0;i<n;i++,v+=stride,va+=a.stride)
    *v /= (*va);
}



//template instantiation for Complex
template<> void VectorTemplate<Complex>::setConjugate(const MyT& a)
{
  CHECKEMPTY();
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<n;i++,v++,va++)
    v->setConjugate(*va);
}

template<> void VectorTemplate<Complex>::inplaceConjugate()
{
  CHECKEMPTY();
  ItT v=begin();
  for(int i=0;i<n;i++,v++)
    v->inplaceConjugate();
}

template<> Complex VectorTemplate<Complex>::dotSelf() const
{
  CHECKEMPTY();
	ItT v=begin();
  Real sum=Zero;
  for(int i=0;i<n;i++,v++) 
    sum += v->normSquared();
  return Complex(sum);
}

template<> Complex VectorTemplate<Complex>::minElement(int* index) const
{
  cerr<<"Incomplete"<<endl;
  AssertNotReached();
  return Zero;
}

template<> Complex VectorTemplate<Complex>::maxElement(int* index) const
{
  cerr<<"Incomplete"<<endl;
  AssertNotReached();
  return Zero;
}

template<> Complex VectorTemplate<Complex>::minAbsElement(int* index) const
{
	CHECKEMPTY();
  ItT v=begin();
	Real b=Abs(*v);
	if(index) *index=0;
	for(int i=1;i<n;i++,v++)
	  if(Abs(*v) < b) {
	    b=Abs(*v);
	    if(index) *index=i;
	  }
	return b;
}

template<> Complex VectorTemplate<Complex>::maxAbsElement(int* index) const
{
	CHECKEMPTY();
  ItT v=begin();
	Real b=Abs(*v);
	if(index) *index=0;
	for(int i=1;i<n;i++,v++)
	  if(Abs(*v) > b) {
	    b=Abs(*v);
	    if(index) *index=i;
	  }
	return b;
}

template class VectorTemplate<float>;
template class VectorTemplate<double>;
template class VectorTemplate<Complex>;
template ostream& operator << (ostream& out, const VectorTemplate<float>& v);
template ostream& operator << (ostream& out, const VectorTemplate<double>& v);
template ostream& operator << (ostream& out, const VectorTemplate<Complex>& v);
template istream& operator >> (istream& in, VectorTemplate<float>& v);
template istream& operator >> (istream& in, VectorTemplate<double>& v);
template istream& operator >> (istream& in, VectorTemplate<Complex>& v);

template void VectorTemplate<float>::copy(const VectorTemplate<double>& a);
template void VectorTemplate<double>::copy(const VectorTemplate<float>& a);
template void VectorTemplate<Complex>::copy(const VectorTemplate<float>& a);
template void VectorTemplate<Complex>::copy(const VectorTemplate<double>& a);

template void VectorTemplate<float>::copy(const std::vector<float>& a);
template void VectorTemplate<float>::copy(const std::vector<int>& a);
template void VectorTemplate<float>::copy(const std::vector<double>& a);
template void VectorTemplate<double>::copy(const std::vector<float>& a);
template void VectorTemplate<double>::copy(const std::vector<int>& a);
template void VectorTemplate<double>::copy(const std::vector<double>& a);
template void VectorTemplate<Complex>::copy(const std::vector<int>& a);
template void VectorTemplate<Complex>::copy(const std::vector<float>& a);
template void VectorTemplate<Complex>::copy(const std::vector<double>& a);

} //namespace Math

