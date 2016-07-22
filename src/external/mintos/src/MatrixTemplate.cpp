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
#include <mintos/math/MatrixTemplate.h>
#include "fastarray.h"
#include "complex.h"
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {

const char* MatrixError_IncompatibleDimensions = "Matrices have incompatible dimensions, %d x %d != %d x %d";
const char* MatrixError_ArgIncompatibleDimensions = "Matrix arguments have incompatible dimensions";
const char* MatrixError_DestIncompatibleDimensions = "Matrix destination has incompatible dimensions";
const char* MatrixError_SizeZero = "Matrix has size 0x0";
const char* MatrixError_NotSquare = "Matrix is not square";
const char* MatrixError_NotSymmetric = "Matrix is not square";
const char* MatrixError_InvalidRow = "Matrix row index [%d] out of range";
const char* MatrixError_InvalidCol = "Matrix column index [%d] out of range";

#define CHECKDIMS(am,an) if(!hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,am,an);
#define CHECKARGDIMS(b,am,an) if(!b.hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
#define CHECKDESTDIMS(am,an) if(!hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
#define CHECKEMPTY() if(hasDims(0,0)) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
#define CHECKSQUARE() if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
#define CHECKROW(i) if(!isValidRow(i)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidRow,i);
#define CHECKCOL(j) if(!isValidCol(j)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidCol,j);
#define CHECKMATROW(mat,i) if(!mat.isValidRow(i)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidRow,i);
#define CHECKMATCOL(mat,j) if(!mat.isValidCol(j)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidCol,j);
#define CHECKRESIZE(m,n) if(isEmpty()) { resize(m,n); } else { CHECKDESTDIMS(m,n); }

#define MYGENARGS getStart(),istride,jstride
#define GENARGS(a) a.getStart(),a.istride,a.jstride

template <class T>
MatrixTemplate<T>::MatrixTemplate()
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
}

template <class T>
MatrixTemplate<T>::MatrixTemplate(const MyT& a)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  copy(a);
}

template <class T>
MatrixTemplate<T>::MatrixTemplate(int _m, int _n)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  resize(_m,_n);
}

template <class T>
MatrixTemplate<T>::MatrixTemplate(int _m, int _n, T initval)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  resize(_m,_n);
  set(initval);
}

template <class T>
MatrixTemplate<T>::MatrixTemplate(int _m, int _n, const T* _vals)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  resize(_m,_n);
  copy(_vals);
}

/*
template <class T>
MatrixTemplate<T>::MatrixTemplate(int _m, int _n, T** const _vals)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  resize(_m,_n);
  copy(_vals);
}
*/
template <class T>
MatrixTemplate<T>::MatrixTemplate(int _m, int _n, const VectorT* _rows)
:vals(NULL),capacity(0),allocated(false),
base(0),istride(0),m(0),jstride(0),n(0)
{
  resize(_m,_n);
  copyRows(_rows);
}

template <class T>
MatrixTemplate<T>::~MatrixTemplate()
{
  clear();
}

template <class T>
void MatrixTemplate<T>::clear()
{
  if(allocated)
    SafeArrayDelete(vals);
  vals=NULL;
  capacity=0;
  base=0;
  istride=0;
  m=0;
  jstride=0;
  n=0;
  allocated=false;
}

template <class T>
void MatrixTemplate<T>::resize(int _m, int _n)
{
  Assert(_m>=0 && _n>=0);
  if(!hasDims(_m,_n))
  {
    if(allocated) {
      Assert(isCompact());
    }
    else {
      Assert(isEmpty());
      Assert(vals == NULL);
      clear();
    }
    if(_m*_n > capacity) {
      SafeArrayDelete(vals);
      try {
	vals = new T[_m*_n];
      }
      catch(exception& e) {
	FatalError("Couldn't allocate matrix of size %d x %d, exception %s",_m,_n,e.what());
      }
      if(!vals) {
	FatalError("Not enough memory to allocate matrix of size %d x %d",_m,_n);
      }
      capacity = _m*_n;
    }

    base = 0;
    m=_m;
    n=_n;
    istride = n;
    jstride = 1;
    allocated = true;
  }
}

template <class T>
void MatrixTemplate<T>::resize(int _m, int _n, T initval)
{
  resize(_m, _n);
  if(_m*_n != 0) set(initval);
}

template <class T>
void MatrixTemplate<T>::resizePersist(int _m, int _n)
{
  Assert(_m>=0 && _n>=0);
  if(!hasDims(_m,_n))
  {
    if(allocated) {
      Assert(isCompact());
    }
    else {
      Assert(isEmpty());
      Assert(vals == NULL);
      clear();
    }
    if(_m*_n > capacity) {
      T* oldvals=vals;
      try {
	vals = new T[_m*_n];
      }
      catch(exception& e) {
	FatalError("Couldn't allocate matrix of size %d x %d, exception %s",_m,_n,e.what());
      }
      if(!vals) {
	FatalError("Not enough memory to allocate matrix of size %d x %d",_m,_n);
      }
      //copy
      gen_array2d_equal(vals,_n,1,oldvals,istride,jstride,m,n);
      SafeArrayDelete(oldvals);
      capacity = _m*_n;
    }
    else if(_n != istride) {
      //re-stride the data
      Assert(jstride == 1);
      if(_n < istride)
	//shrink: re-striding can be done in-place in normal order
	gen_array2d_equal(vals,_n,1,vals,istride,jstride,m,_n);
      else {
	//grow: re-striding cannot be done in-place with normal order, but backwards is fine
	for(int i=m-1;i>=0;i--)
	  gen_array_equal(vals+_n*i,1,vals+istride*i,jstride,istride);
      }
    }

    base = 0;
    m=_m;
    n=_n;
    istride = n;
    jstride = 1;
    allocated = true;
  }
}

template <class T>
void MatrixTemplate<T>::resizePersist(int _m, int _n, T initval)
{
  int oldm = m, oldn = n;
  resizePersist(_m, _n);
  if(_m > oldm) {
    //fill in new rows
    gen_array2d_fill(vals+base+istride*oldm,istride,jstride,initval,_m-oldm,n);
  }
  if(_n > oldn) {
    //fill in new cols
    int numRows = Min(m,oldm);
    gen_array2d_fill(vals+base+jstride*oldn,istride,jstride,initval,numRows,_n-oldn);
  }
}


template <class T>
const MatrixTemplate<T>& MatrixTemplate<T>::operator = (const MyT& a)
{
  if(this == &a) return *this;
  if(m!=a.m || n!=a.n) resize(a.m,a.n);
  gen_array2d_equal(MYGENARGS,GENARGS(a),m,n);
  return *this;
}


template <class T>
bool MatrixTemplate<T>::operator == (const MyT& a) const
{
  return isEqual(a,Zero);
}

template <class T>
void MatrixTemplate<T>::operator *= (const MyT& a)
{
  MatrixTemplate<T> tmp(*this);
  mul(tmp, a);
}



template <class T>
void MatrixTemplate<T>::copy(const MyT& a)
{
  if(this == &a) return;
  CHECKRESIZE(a.m,a.n);
  gen_array2d_equal(MYGENARGS,GENARGS(a),m,n);
}

template <class T>
template <class T2> 
void MatrixTemplate<T>::copy(const MatrixTemplate<T2>& a)
{
  CHECKRESIZE(a.m,a.n);
  MatrixIterator<T> k=begin();
  MatrixIterator<T2> ak=a.begin();
  for(int i=0;i<m;i++,k.nextRow(),ak.nextRow())
    for(int j=0;j<n;j++,k.nextCol(),ak.nextCol())
      *k = (T)*ak;
}

template <class T>
void MatrixTemplate<T>::copy(const T* _vals)
{
  CHECKEMPTY();
  gen_array2d_equal(MYGENARGS,
    _vals,n,1,
    m,n);
}

template <class T>
void MatrixTemplate<T>::copyColumns(const T* _vals)
{
  CHECKEMPTY();
  gen_array2d_equal(MYGENARGS,
    _vals,1,n,
    m,n);
}

template <class T>
void MatrixTemplate<T>::copyRows(const VectorT* rows)
{
  CHECKEMPTY();
  for(int i=0; i<m; i++)
  {
    if(rows[i].n != n)
    {
      RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,rows[i].n);
    }
    VectorT temprow;
    getRowRef(i,temprow);
    temprow.copy(rows[i]);
  }
}

template <class T>
void MatrixTemplate<T>::copyCols(const VectorT* cols)
{
  CHECKEMPTY();
  for(int j=0; j<n; j++)
  {
    if(cols[j].n != m)
    {
      RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,cols[j].n,-1);
    }
    VectorT tempcol;
    getColRef(j,tempcol);
    tempcol.copy(cols[j]);
  }
}

template <class T>
void MatrixTemplate<T>::copySubMatrix(int i, int j, const MyT& a)
{
  CHECKROW(i);
  CHECKCOL(j);
  CHECKROW(i+a.m-1);
  CHECKCOL(j+a.n-1);

  T* v=getStart()+i*istride+j*jstride;
  gen_array2d_equal(v,istride,jstride,GENARGS(a),a.m,a.n);
}

template <class T>
void MatrixTemplate<T>::swap(MyT& a)
{
  std::swap(vals,a.vals);
  std::swap(allocated,a.allocated);
  std::swap(capacity,a.capacity);
  std::swap(m,a.m);
  std::swap(n,a.n);
  std::swap(base,a.base);
  std::swap(istride,a.istride);
  std::swap(jstride,a.jstride);
}

template <class T>
void MatrixTemplate<T>::swapCopy(MyT& a)
{
  CHECKDIMS(a.m,a.n);
  T tmp;
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow()) {
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol()) {
      tmp=*v; *v=*va; *va=tmp;
    }
  }
}

template <class T>
void MatrixTemplate<T>::add(const MyT& a, const MyT& b)
{
  CHECKARGDIMS(b,a.m,a.n);
  CHECKRESIZE(a.m,a.n);

  gen_array2d_add(MYGENARGS,GENARGS(a),GENARGS(b),m,n);
}

template <class T>
void MatrixTemplate<T>::sub(const MyT& a, const MyT& b)
{
  CHECKARGDIMS(b,a.m,a.n);
  CHECKRESIZE(a.m,a.n);

  gen_array2d_sub(MYGENARGS,GENARGS(a),GENARGS(b),m,n);
}

template <class T>
void MatrixTemplate<T>::mul(const MyT& a, const MyT& b)
{
  if(b.m != a.n)
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  CHECKRESIZE(a.m,b.n);

  gen_array2d_multiply(MYGENARGS,GENARGS(a),GENARGS(b),
    m,a.n,n);
}

template <class T>
void MatrixTemplate<T>::mulTransposeA(const MyT& a, const MyT& b)
{
  if(b.m != a.m)
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  CHECKRESIZE(a.n,b.n);

  gen_array2d_multiply_transposeA(MYGENARGS,GENARGS(a),GENARGS(b),
    m,a.m,n);
}

template <class T>
void MatrixTemplate<T>::mulTransposeB(const MyT& a, const MyT& b)
{
  if(b.n != a.n)
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  CHECKRESIZE(a.m,b.m);

  gen_array2d_multiply_transposeB(MYGENARGS,GENARGS(a),GENARGS(b),
    m,a.n,n);
}

template <class T>
void MatrixTemplate<T>::mul(const VectorT& a, VectorT& b) const
{
  if(n != a.n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  }
  if(b.n == 0)
  {
    b.resize(m);
  }
  else if(b.n != m)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
  }

  gen_array2d_vector_multiply(b.getStart(),b.stride,
    getStart(),istride,jstride, 
    a.getStart(),a.stride, 
    m, n);
}

template <class T>
void MatrixTemplate<T>::mulTranspose(const VectorT& a, VectorT& b) const
{
  if(m != a.n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  }
  if(b.n == 0)
  {
    b.resize(n);
  }
  else if(b.n != n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
  }

  gen_array2d_vector_multiply_transpose(b.getStart(),b.stride,
    getStart(),istride,jstride, 
    a.getStart(),a.stride, 
    m, n);
}

template <class T>
void MatrixTemplate<T>::mul(const MyT& a, T c)
{
  CHECKRESIZE(a.m,a.n);
  gen_array2d_mul(MYGENARGS,GENARGS(a),c,
    m,n);
}

template <class T>
void MatrixTemplate<T>::div(const MyT& a, T c)
{
  mul(a,Inv(c));
}

template <class T>
void MatrixTemplate<T>::inc(const MyT& a)
{
  CHECKDIMS(a.m,a.n);
  gen_array2d_acc(MYGENARGS,GENARGS(a),m,n);
}

template <class T>
void MatrixTemplate<T>::dec(const MyT& a)
{
  CHECKDIMS(a.m,a.n);
  gen_array2d_dec(MYGENARGS,GENARGS(a),m,n);
}

template <class T>
void MatrixTemplate<T>::madd(const MyT& a, T c)
{
  CHECKDIMS(a.m,a.n);
  gen_array2d_madd(MYGENARGS,GENARGS(a),c,m,n);
}

template <class T>
void MatrixTemplate<T>::madd(const VectorT& a, VectorT& b) const
{
  if(n != a.n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  }
  if(b.n == 0)
  {
    b.resize(m);
  }
  else if(b.n != m)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
  }

  gen_array2d_vector_madd(b.getStart(),b.stride,
    getStart(),istride,jstride, 
    a.getStart(),a.stride, 
    m, n);
}

template <class T>
void MatrixTemplate<T>::maddTranspose(const VectorT& a, VectorT& b) const
{
  if(m != a.n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
  }
  if(b.n == 0)
  {
    b.resize(n);
  }
  else if(b.n != n)
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
  }

  gen_array2d_vector_madd_transpose(b.getStart(),b.stride,
    getStart(),istride,jstride, 
    a.getStart(),a.stride, 
    m, n);
}

template <class T>
void MatrixTemplate<T>::setRef(const MyT& mat,int i,int j,int _istride,int _jstride,int _m,int _n)
{
  Assert(this != &mat);
  Assert(!allocated);
  vals = mat.vals;
  capacity = mat.capacity;
  allocated = false;
  base = mat.base+i*mat.istride+j*mat.jstride;
  istride = _istride*mat.istride;
  jstride = _jstride*mat.jstride;
  if(mat.isEmpty()) {
    Assert(_m <= 0 && _n <= 0);
    if(_m < 0) m = mat.m;
    else m=0;
    if(_n < 0) n = mat.n;
    else n=0;
    Assert(isValid());
    return;
  }
  if(_m < 0) {
    Assert(istride != 0);
    m = (mat.m-i+_istride-1) / _istride;
  }
  else
    m = _m;
  if(_n < 0) {
    Assert(jstride != 0);
    n = (mat.n-j+_jstride-1) / _jstride;
  }
  else
    n = _n;
  Assert(isValid());
}

template <class T>
void MatrixTemplate<T>::setRef(T* _vals,int length,int offset,int _istride,int _jstride,int _m,int _n)
{
  Assert(!allocated);
  vals = _vals;
  capacity = length;
  allocated = false;
  base = offset;
  istride = _istride;
  jstride = _jstride;
  if(_m < 0) {
    Assert(istride != 0);
    m = (capacity-base) / istride;
  }
  else
    m = _m;
  if(_n < 0) {
    Assert(jstride != 0);
    n = istride / jstride;
  }
  else
    n = _n;
  Assert(isValid());
}

template <class T>
void MatrixTemplate<T>::setRefTranspose(const MyT& mat)
{
  Assert(this != &mat);
  Assert(!allocated);
  vals = mat.vals;
  capacity = mat.capacity;
  allocated = false;
  base = mat.base;
  istride = mat.jstride;
  jstride = mat.istride;
  m = mat.n;
  n = mat.m;
  Assert(isValid());
}

template <class T>
void MatrixTemplate<T>::set(T c)
{
  CHECKEMPTY();
  gen_array2d_fill(MYGENARGS,c,m,n);
}

template <class T>
void MatrixTemplate<T>::setZero()
{
  CHECKEMPTY();
  set(T(0));
}

template <class T>
void MatrixTemplate<T>::setIdentity()
{
  CHECKEMPTY();
  CHECKSQUARE();
  
  gen_array2d_identity(MYGENARGS,m,n);
}

template <class T>
void MatrixTemplate<T>::setNegative(const MyT& a)
{
  CHECKRESIZE(a.m,a.n);
  gen_array2d_negate(MYGENARGS,GENARGS(a),m,n);
}

template <class T>
void MatrixTemplate<T>::setTranspose(const MyT& a)
{
  if(this == &a) {
    inplaceTranspose();
    return;
  }
  CHECKRESIZE(a.n,a.m);
  gen_array2d_transpose(MYGENARGS,GENARGS(a),m,n);
}

template <class T>
void MatrixTemplate<T>::setAdjoint(const MyT& a)
{
  setTranspose(a);
}

template <class T>
void MatrixTemplate<T>::setInverse(const MyT& m)
{
  if(!isSquare())
  {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
  }
  fprintf(stderr,"Inverse not done yet");
  AssertNotReached();
}

template <class T>
void MatrixTemplate<T>::inplaceNegative()
{
  CHECKEMPTY();
  gen_array2d_negate(MYGENARGS,MYGENARGS,m,n);
}

template <class T>
void MatrixTemplate<T>::inplaceMul(T c)
{
  CHECKEMPTY();
  gen_array2d_mul(MYGENARGS,c,m,n);
}

template <class T>
void MatrixTemplate<T>::inplaceDiv(T c)
{
  CHECKEMPTY();
  gen_array2d_div(MYGENARGS,c,m,n);
}

template <class T>
void MatrixTemplate<T>::inplaceTranspose()
{
  CHECKEMPTY();
  CHECKSQUARE();

  T* rowi = getStart();
  T* coli = getStart();
  for(int i=0; i<m; i++,rowi+=istride,coli+=jstride)
  {
    gen_array_swap(rowi,jstride,coli,istride,i-1);
  }
}

template <class T>
void MatrixTemplate<T>::inplaceAdjoint()
{
  inplaceTranspose();
}

template <class T>
void MatrixTemplate<T>::inplaceInverse()
{
  MatrixTemplate<T> tmp(*this);
  setInverse(tmp);
}


template <class T>
T* MatrixTemplate<T>::getRowPtr(int i) const
{
  return getStart()+i*istride;
}

template <class T>
T* MatrixTemplate<T>::getColPtr(int j) const
{
  return getStart()+j*jstride;
}

template <class T>
MatrixIterator<T> MatrixTemplate<T>::begin() const
{
  return ItT(vals+base,istride,jstride);
}

template <class T>
MatrixIterator<T> MatrixTemplate<T>::end() const
{
  if(isRowMajor())
    return ItT(vals+base+istride*(m-1)+jstride*n,istride,jstride); 
  else
    return ItT(vals+base+istride*m+jstride*(n-1),istride,jstride); 
}

template <class T>
void MatrixTemplate<T>::getSubMatrixCopy(int i, int j, MyT& a) const
{
  CHECKROW(i);
  CHECKCOL(j);
  CHECKROW(i+a.m-1);
  CHECKCOL(j+a.n-1);

  gen_array2d_equal(GENARGS(a),
    getStart()+i*istride+j*jstride,istride,jstride,
    a.m,a.n);
}

template <class T>
bool MatrixTemplate<T>::isValid() const 
{
  if(vals == NULL) {
    if(capacity != 0) {
      cout<<"Invalid capacity on empty matrix"<<endl;
      return false;
    }
    if(m > 0 && n > 0) {
      cout<<"Invalid size on empty matrix"<<endl;
      return false;
    }
    return true;
  }
  if(istride < 0 || jstride < 0) {
    cout<<"Invalid strides "<<istride<<", "<<jstride<<endl;
    return false;
  }
  //check non-overlap of rows/cols
  if(istride > jstride) {
    if(jstride*(n-1) >= istride) {
      cout<<"J-row overlaps with I-row"<<endl;
      return false;
    }
  }
  else if(jstride < istride) {
    if(istride*(m-1) >= jstride) { 
      cout<<"I-row overlaps with J-row"<<endl;
      return false;
    }
  }
  else if(istride==jstride) {
    bool ok=false;
    if(m == 0 && n == 0) ok=true;
    if(istride == 1 && (m<=1 || n<=1)) ok=true;
    if(!ok) {
      cout<<"Equal i-stride and j-stride?"<<endl;
      cout<<"dims "<<m<<"x"<<n<<endl;
      return false;
    }
  }
  if(base+(m-1)*istride+(n-1)*jstride>=capacity) {
    cout<<"Overloaded capacity: "<<base+(m-1)*istride+(n-1)*jstride<<" vs "<<capacity<<endl;
    return false;
  }
  if(base < 0) {
    cout<<"Negative base"<<endl;
    return false;
  }
  return true;
}

template <class T>
bool MatrixTemplate<T>::isIdentity() const
{
  CHECKEMPTY();
  if(!isSquare()) return false;

  ItT v=begin();
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(!FuzzyEquals((T)Delta(i,j),*v))
        return false;
  return true;
}

template <class T>
bool MatrixTemplate<T>::isDiagonal() const
{
  CHECKEMPTY();
  if(!isSquare()) return false;

  ItT v=begin();
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(i!=j && !FuzzyZero(*v))
        return false;
  return true;
}

template <class T>
bool MatrixTemplate<T>::isSymmetric() const
{
  CHECKEMPTY();
  if(!isSquare()) return false;

  T* rowi=getStart();
  T* coli=getStart();
  for(int i=0;i<m;i++,rowi+=istride,coli+=jstride) {
    T* rowij = rowi;
    T* colij = coli;
    for(int j=0;j<i;j++,rowij+=jstride,colij+=istride)
      if(!FuzzyEquals(*rowij,*colij))
        return false;
  }
  return true;
}

template <class T>
bool MatrixTemplate<T>::isZero(T eps) const
{
  CHECKEMPTY();

  ItT v=begin();
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(!FuzzyZero(*v,eps))
        return false;
  return true;
}

template <class T>
bool MatrixTemplate<T>::isEqual(const MyT& a,T eps) const
{
  CHECKEMPTY();
  CHECKDIMS(a.m,a.n);

  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol())
      if(!FuzzyEquals(*v,*va,eps))
        return false;
  return true;
}

template <class T>
bool MatrixTemplate<T>::isInvertible() const
{
  if(isEmpty())
  {
    FatalError(MatrixError_SizeZero);
  }
  if(!isSquare())
    return false;

  return !FuzzyZero(determinant());
}

template <class T>
bool MatrixTemplate<T>::isOrthogonal() const
{
  CHECKEMPTY();
  if(!isSquare())
    return false;

  VectorT rowi,rowj;
  for(int i=0; i<m; i++)
  {
    getRowRef(i,rowi);
    if(!FuzzyEquals(rowi.norm(), (T)One))
      return false;
    for(int j=0; j<i; j++) {
      getRowRef(j,rowj);
      if(!FuzzyEquals(rowi.dot(rowj), (T)Zero))
        return false;
    }
  }
  return true;
}

template <class T>
bool MatrixTemplate<T>::isDiagonallyDominant() const
{
  if(!isSquare()) return false;
  //diagonally dominant
  for(int i=0;i<m;i++) {
    T* rowi = getRowPtr(i);
    Real sum = Zero;
    for(int j=0;j<n;j++,rowi+=jstride)
      if(j!=i) sum += (Real)Abs(*rowi);
    if(sum > (Real)Abs(rowi[i*jstride])) return false;
  }
  return true;
}



template <class T>
T MatrixTemplate<T>::trace() const
{
  if(isEmpty()) return Zero;
  CHECKSQUARE();
  VectorT diag;
  getDiagRef(0,diag);
  T tr=Zero;
  for(int i=0;i<m;i++) tr += diag[i];
  return tr;
}

template <class T>
T MatrixTemplate<T>::determinant() const
{
  if(isEmpty()) return Zero;
  if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);

  fprintf(stderr,"Haven't completed the determinant\n");
  AssertNotReached();
  /*
  LU_Decomposition(*this, L,U, P);
  return diagonalProduct(L) * diagonalProduct(U) * signature(P);
  */
  return Zero;
}

template <class T>
T MatrixTemplate<T>::diagonalProduct() const
{
  if(isEmpty()) return One;
  if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
  VectorT diag;
  getDiagRef(0,diag);
  T dp=One;
  for(int i=0;i<m;i++) dp *= diag[i];
  return dp;
}

template <class T>
T MatrixTemplate<T>::minElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  T val=*v;
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(*v < val) {
       val = *v;
       if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template <class T>
T MatrixTemplate<T>::maxElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  T val=*v;
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(*v > val) {
       val = *v;
       if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template <class T>
T MatrixTemplate<T>::minAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  T val=Abs(*v);
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(Abs(*v) < val) {
        val = Abs(*v);
        if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template <class T>
T MatrixTemplate<T>::maxAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  T val=Abs(*v);
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(Abs(*v) > val) {
        val = Abs(*v);
        if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template <class T>
ostream& operator << (ostream& out, const MatrixTemplate<T>& mat)
{
  out << mat.m << " " << mat.n << "\t";
  for(int i=0; i<mat.m; i++) {
    for(int j=0; j<mat.n; j++)
      out << mat(i,j) << " ";
    out << endl;
  }
  return out;
}

template <class T>
istream& operator >> (istream& in, MatrixTemplate<T>& mat)
{
  int m,n;
  in >> m >> n;
  mat.resize(m,n);
  for(int i=0; i<m; i++) {
    for(int j=0; j<n; j++)
      in >> mat(i,j);
  }
  return in;
}


template <class T>
void MatrixTemplate<T>::getRowRef(int i, VectorT& v) const
{
  v.setRef(vals,capacity,base+i*istride,jstride,n);
}

template <class T>
void MatrixTemplate<T>::getColRef(int j, VectorT& v) const
{
  v.setRef(vals,capacity,base+j*jstride,istride,m);
}

template <class T>
void MatrixTemplate<T>::getDiagRef(int d, VectorT& v) const
{
  if(d >= 0) {  //upper diagonals
    v.setRef(vals,capacity,base+jstride*d,istride+jstride,::Min(n-d,m));
  }
  else { //lower diagonals
    d = -d;
    v.setRef(vals,capacity,base+istride*d,istride+jstride,::Min(m-d,n));
  }
}



template <class T>
void MatrixTemplate<T>::componentMul(const MyT& a,const MyT& b)
{
  Assert(a.hasDims(b.m,b.n));
  CHECKRESIZE(a.m,a.n);
  ItT v=begin();
  ItT va=a.begin();
  ItT vb=b.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow(),vb.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol(),vb.nextCol())
      *v = (*va)*(*vb);
}

template <class T>
void MatrixTemplate<T>::componentDiv(const MyT& a,const MyT& b)
{
Assert(a.hasDims(b.m,b.n));
  CHECKRESIZE(a.m,a.n);
  ItT v=begin();
  ItT va=a.begin();
  ItT vb=b.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow(),vb.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol(),vb.nextCol())
      *v = (*va)/(*vb);
}

template <class T>
void MatrixTemplate<T>::componentMadd(const MyT& a,const MyT& b)
{
  Assert(a.hasDims(b.m,b.n));
  CHECKRESIZE(a.m,a.n);
  ItT v=begin();
  ItT va=a.begin();
  ItT vb=b.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow(),vb.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol(),vb.nextCol())
      *v += (*va)*(*vb);
}

template <class T>
void MatrixTemplate<T>::inplaceComponentMul(const MyT& a)
{
  CHECKDIMS(a.m,a.n);
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol())
      *v *= (*va);
}

template <class T>
void MatrixTemplate<T>::inplaceComponentDiv(const MyT& a)
{
  CHECKDIMS(a.m,a.n);
  ItT v=begin();
  ItT va=a.begin();
  for(int i=0;i<m;i++,v.nextRow(),va.nextRow())
    for(int j=0;j<n;j++,v.nextCol(),va.nextCol())
      *v /= (*va);
}


//specialization for complex

template<> void MatrixTemplate<Complex>::setAdjoint(const MyT& a)
{
  setTranspose(a);
  ItT v=begin();
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      v->inplaceConjugate();
}

template<> void MatrixTemplate<Complex>::inplaceAdjoint()
{
  inplaceTranspose();
  ItT v=begin();
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      v->inplaceConjugate();
}

template<> Complex MatrixTemplate<Complex>::minElement(int*_i,int*_j) const
{
  AssertNotReached();
  return Zero;
}

template<> Complex MatrixTemplate<Complex>::maxElement(int*_i,int*_j) const
{
  AssertNotReached();
  return Zero;
}

template<> Complex MatrixTemplate<Complex>::minAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  Real val=Abs(*v);
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(Abs(*v) < val) {
        val = Abs(*v);
        if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template<> Complex MatrixTemplate<Complex>::maxAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  ItT v=begin();
  Real val=Abs(*v);
  for(int i=0;i<m;i++,v.nextRow())
    for(int j=0;j<n;j++,v.nextCol())
      if(Abs(*v) > val) {
        val = Abs(*v);
        if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

template class MatrixTemplate<float>;
template class MatrixTemplate<double>;
template class MatrixTemplate<Complex>;
template ostream& operator << (ostream& out, const MatrixTemplate<float>& v);
template ostream& operator << (ostream& out, const MatrixTemplate<double>& v);
template ostream& operator << (ostream& out, const MatrixTemplate<Complex>& v);
template istream& operator >> (istream& in, MatrixTemplate<float>& v);
template istream& operator >> (istream& in, MatrixTemplate<double>& v);
template istream& operator >> (istream& in, MatrixTemplate<Complex>& v);

template void MatrixTemplate<float>::copy(const MatrixTemplate<double>& a);
template void MatrixTemplate<double>::copy(const MatrixTemplate<float>& a);
template void MatrixTemplate<Complex>::copy(const MatrixTemplate<float>& a);
template void MatrixTemplate<Complex>::copy(const MatrixTemplate<double>& a);

} //namespace Math
