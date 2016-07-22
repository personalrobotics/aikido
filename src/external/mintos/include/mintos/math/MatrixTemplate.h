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

#ifndef MATH_MATRIX_TEMPLATE_H
#define MATH_MATRIX_TEMPLATE_H

#include "VectorTemplate.h"
#include <mintos/misc/errors.h>

namespace Math {

/** @ingroup Math
 * @brief An iterator through MatrixTemplate elements.
 *
 * Operates similarly to VectorIterator, but instead of ++ and --,
 * rows and columns are iterated explicitly with the nextRow(),nextCol(),
 * prevRow(), and prevCol() methods.
 */
template <class T>
class MatrixIterator
{
public:
  typedef MatrixIterator<T> MyT;
  inline MatrixIterator() :ptr(NULL),rowPtr(NULL),istride(0),jstride(0) {}
  inline MatrixIterator(const MyT& i) :ptr(i.ptr),rowPtr(i.rowPtr),istride(i.istride),jstride(i.jstride) {}
  inline MatrixIterator(T* _ptr,int _istride,int _jstride) :ptr(_ptr),rowPtr(_ptr),istride(_istride),jstride(_jstride) {}
  inline T& operator*() { return *ptr; }
  inline T* operator->() { return ptr; }
  inline MyT& nextRow() { rowPtr+=istride; ptr=rowPtr; return *this; }
  inline MyT& prevRow() { rowPtr-=istride; ptr=rowPtr; return *this; }
  inline MyT& nextCol() { ptr+=jstride; return *this; }
  inline MyT& prevCol() { ptr-=jstride; return *this; }
  //inline MyT& operator +=(int i) { ptr+=i*stride; return *this; }
  //inline MyT& operator -=(int i) { ptr-=i*stride; return *this; }
  inline bool operator !=(const MyT& i) { return ptr!=i.ptr; }
  inline bool operator ==(const MyT& i) { return ptr==i.ptr; }
  inline bool operator < (const MyT& i) { return ptr<i.ptr; }
  inline bool operator > (const MyT& i) { return ptr>i.ptr; }

  T *ptr, *rowPtr;
  int istride,jstride;
};

/** @ingroup Math
 * @brief A matrix over the field T.
 *
 * Much like VectorTemplate, this is a basic matrix class used throughout
 * the math library.  Element i,j is accessed via parentheses A(i,j).
 * The matrix is m x n, such that the range of element indices is 
 * [0,A.m)x[0,A.n).
 *
 * The operational modes of MatrixTemplate follow those of VectorTemplate, 
 * such as being able to handle both self-allocated data and referenced data.
 * This can also operate in both row-major and column-major mode
 * using non-contiguous data access.  Element (i,j) accesses
 * data[base+i*istride+j*jstride], so by changing the values of istride and
 * jstride the desired effect can be achieved.
 *
 * A handy tool for transposing a matrix without manipulating data
 * is the setRefTranspose() method.  If B.setRefTranspose(A) is called,
 * then B(i,j) accesses the same data as A(j,i).
 *
 * Also, it is handy to access rows, columns, or diagonals of the matrix
 * as if they were vectors.  Dozens of methods are provided for operations
 * on rows, columns, and diagonals.
 */
template <class T>
class MatrixTemplate
{
public:
  typedef class MatrixTemplate<T> MyT; 
  typedef class MatrixIterator<T> ItT; 
  typedef class VectorTemplate<T> VectorT; 

  MatrixTemplate();
  MatrixTemplate(const MyT&);
  MatrixTemplate(int m, int n);
  MatrixTemplate(int m, int n, T initval);
  MatrixTemplate(int m, int n, const T* vals);
  MatrixTemplate(int m, int n, const VectorT* rows);
  ~MatrixTemplate();

  inline T* getPointer() const { return vals; }
  inline int getCapacity() const { return capacity; }
  inline T* getStart() const { return vals+base; }
  T* getRowPtr(int i) const;
  T* getColPtr(int j) const;
  ItT begin() const;
  ItT end() const;
  inline int numRows() const { return m; }
  inline int numCols() const { return n; }

  void resize(int m, int n);
  void resize(int m, int n, T initval);
  void resizePersist(int m, int n);
  void resizePersist(int m, int n, T initval);
  void clear();

  const MyT& operator = (const MyT&);
  bool operator == (const MyT&) const;
  inline bool operator != (const MyT& a) const { return !operator==(a); }
  inline const T& operator() (int,int) const;
  inline T& operator() (int,int);
  inline void operator += (const MyT& a) { inc(a); }
  inline void operator -= (const MyT& a) { dec(a); }
  inline void operator *= (T c) { inplaceMul(c); }
  inline void operator /= (T c) { inplaceDiv(c); }
  //NOTE: this is slow...
  void operator *= (const MyT&);

  void copy(const MyT&);
  template <class T2> void copy(const MatrixTemplate<T2>&);
  void copy(const T* vals);
  void copyColumns(const T* vals);
  void copyRows(const VectorT* rows);
  void copyCols(const VectorT* cols);
  void copySubMatrix(int i, int j, const MyT&);
  void swap(MyT&);
  void swapCopy(MyT&);
  void add(const MyT&, const MyT&);
  void sub(const MyT&, const MyT&);
  void mul(const MyT&, const MyT&);
  void mulTransposeA(const MyT& a, const MyT& b);
  void mulTransposeB(const MyT& a, const MyT& b);
  void mul(const VectorT&, VectorT&) const;
  void mulTranspose(const VectorT&, VectorT&) const;
  void mul(const MyT&, T);
  void div(const MyT&, T);
  void inc(const MyT&);
  void dec(const MyT&);
  void madd(const MyT&, T);
  void madd(const VectorT&, VectorT&) const;
  void maddTranspose(const VectorT&, VectorT&) const;

  void setRef(const MyT&,int i=0,int j=0,int istride=1,int jstride=1,int m=-1,int n=-1);
  void setRef(T* vals,int length,int offset=0,int istride=1,int jstride=1,int m=-1,int n=-1);
  void setRefTranspose(const MyT&);
  void set(T c);
  void setZero();
  void setIdentity();
  void setNegative(const MyT&);
  void setTranspose(const MyT&);
  void setAdjoint(const MyT&);
  void setInverse(const MyT&);      //uses LU decomposition

  void inplaceNegative();
  void inplaceMul(T);
  void inplaceDiv(T);
  void inplaceTranspose();
  void inplaceAdjoint();
  void inplaceInverse();

  void getSubMatrixCopy(int i, int j, MyT&) const;

  inline bool isRef() const { return !allocated; }
  inline bool hasDims(int _m,int _n) const { return _m==m&&_n==n; }
  inline bool isEmpty() const { return vals==NULL; }
  inline bool isValidRow(int i) const { return i >= 0 && i < m; }
  inline bool isValidCol(int j) const { return j >= 0 && j < n; }
  inline bool isCompact() const { return (istride==n&&jstride==1); }
  inline bool isRowMajor() const { return istride>jstride; }
  inline bool isColMajor() const { return jstride>istride; }
  inline bool isSquare() const { return m == n; }
  bool isValid() const;
  bool isZero(T eps=0) const;
  bool isEqual(const MyT& a,T eps=0) const;
  bool isIdentity() const;
  bool isDiagonal() const;
  bool isSymmetric() const;
  //bool isLowerTriangular() const;
  //bool isUpperTriangular() const;
  bool isDiagonallyDominant() const;
  bool isOrthogonal() const;
  bool isInvertible() const;

  T trace() const;
  T determinant() const;
  T diagonalProduct() const;
  T minElement(int*i=NULL,int*j=NULL) const;
  T maxElement(int*i=NULL,int*j=NULL) const;
  T minAbsElement(int*i=NULL,int*j=NULL) const;
  T maxAbsElement(int*i=NULL,int*j=NULL) const;

  void getRowRef(int i, VectorT&) const;
  void getColRef(int j, VectorT&) const;
  void getDiagRef(int d, VectorT&) const;
  inline VectorT row(int i) const { VectorT a; getRowRef(i,a); return a; }
  inline VectorT col(int j) const { VectorT a; getColRef(j,a); return a; }
  inline VectorT diag(int d) const { VectorT a; getDiagRef(d,a); return a; }
  inline void getRowCopy(int i, VectorT& b) const { VectorT a; getRowRef(i,a); b.copy(a); }
  inline void getColCopy(int j, VectorT& b) const { VectorT a; getColRef(j,a); b.copy(a); }
  inline void getDiagCopy(int d, VectorT& b) const { VectorT a; getDiagRef(d,a); b.copy(a); }
  inline void setRow(int i, T c) { VectorT a; getRowRef(i,a); a.set(c); }
  inline void setCol(int j, T c) { VectorT a; getColRef(j,a); a.set(c); }
  inline void setDiag(int d, T c) { VectorT a; getDiagRef(d,a); a.set(c); }
  inline void copyRow(int i, const VectorT& b) { VectorT a; getRowRef(i,a); a.copy(b); }
  inline void copyCol(int j, const VectorT& b) { VectorT a; getColRef(j,a); a.copy(b); }
  inline void copyDiag(int d, const VectorT& b) { VectorT a; getDiagRef(d,a); a.copy(b); }
  inline void copyRow(int i, const T* b) { VectorT a; getRowRef(i,a); a.copy(b); }
  inline void copyCol(int j, const T* b) { VectorT a; getColRef(j,a); a.copy(b); }
  inline void copyDiag(int d, const T* b) { VectorT a; getDiagRef(d,a); a.copy(b); }
  inline void incRow(int i,const VectorT& b) { VectorT a; getRowRef(i,a); a.inc(b); }
  inline void incCol(int j,const VectorT& b) { VectorT a; getColRef(j,a); a.inc(b); }
  inline void incDiag(int d,const VectorT& b) { VectorT a; getDiagRef(d,a); a.inc(b); }
  inline void mulRow(int i,T c) { VectorT a; getRowRef(i,a); a.inplaceMul(c); }
  inline void mulCol(int j,T c) { VectorT a; getColRef(j,a); a.inplaceMul(c); }
  inline void mulDiag(int d,T c) { VectorT a; getDiagRef(d,a); a.inplaceMul(c); }
  inline void maddRow(int i,const VectorT& b,T c) { VectorT a; getRowRef(i,a); a.mul(b,c); }
  inline void maddCol(int j,const VectorT& b,T c) { VectorT a; getColRef(j,a); a.madd(b,c); }
  inline void maddDiag(int d,const VectorT& b,T c) { VectorT a; getDiagRef(d,a); a.madd(b,c); }
  inline T dotRow(int i,const VectorT& b) const { VectorT a; getRowRef(i,a); return a.dot(b); }
  inline T dotCol(int j,const VectorT& b) const { VectorT a; getColRef(j,a); return a.dot(b); }

  inline void incRow(int i,const MyT& m,int im) { VectorT a; m.getRowRef(im,a); incRow(i,a); }
  inline void incCol(int j,const MyT& m,int jm) { VectorT a; m.getColRef(jm,a); incCol(j,a); }
  inline void maddRow(int i,const MyT& m,int im,T c) { VectorT a; m.getRowRef(im,a); maddRow(i,a,c); }
  inline void maddCol(int j,const MyT& m,int jm,T c) { VectorT a; m.getColRef(jm,a); maddCol(j,a,c); }
  inline T dotRow(int i,const MyT& m,int im) const { VectorT a; m.getRowRef(im,a); return dotRow(i,a); }
  inline T dotCol(int j,const MyT& m,int jm) const { VectorT a; m.getColRef(jm,a); return dotCol(j,a); }

  void componentMul(const MyT& a,const MyT& b);
  void componentDiv(const MyT& a,const MyT& b);
  void componentMadd(const MyT& a,const MyT& b);
  void inplaceComponentMul(const MyT& c);
  void inplaceComponentDiv(const MyT& c);

private:
  //read only
  T* vals;
  int capacity;
  bool allocated;

public:
  //alterable
  int base,istride,m,jstride,n;
};

///returns true if all elements of A are finite
template <class T>
inline bool IsFinite(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(!IsFinite(A(i,j))) return false;
  return true;
}

///returns true if any element of A is NaN
template <class T>
inline bool HasNaN(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(IsNaN(A(i,j))) return true;
  return false;
}

///returns nonzero if any element of A is infinite
template <class T>
inline int HasInf(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(IsInf(A(i,j))) return IsInf(A(i,j));
  return 0;
}

class Complex;
typedef class MatrixTemplate<float> fMatrix;
typedef class MatrixTemplate<double> dMatrix;
typedef class MatrixTemplate<Complex> cMatrix;

template <class T>
std::ostream& operator << (std::ostream&, const MatrixTemplate<T>&);
template <class T>
std::istream& operator >> (std::istream&, MatrixTemplate<T>&);

extern const char* MatrixError_IncompatibleDimensions;
extern const char* MatrixError_ArgIncompatibleDimensions;
extern const char* MatrixError_DestIncompatibleDimensions;
extern const char* MatrixError_SizeZero;
extern const char* MatrixError_NotSquare;
extern const char* MatrixError_NotSymmetric;
extern const char* MatrixError_InvalidRow;
extern const char* MatrixError_InvalidCol;

template <class T>
inline const T& MatrixTemplate<T>::operator() (int i,int j) const
{
#ifdef _DEBUG
  if(!isValidRow(i))
    FatalError(MatrixError_InvalidRow);
  if(!isValidCol(j))
    FatalError(MatrixError_InvalidCol);
#endif
  return vals[base+i*istride+j*jstride];
}

template <class T>
inline T& MatrixTemplate<T>::operator() (int i,int j)
{
#ifdef _DEBUG
  if(!isValidRow(i))
    FatalError(MatrixError_InvalidRow);
  if(!isValidCol(j))
    FatalError(MatrixError_InvalidCol);
#endif
  return vals[base+i*istride+j*jstride];
}

} //namespace Math

namespace std
{
  template<class T> inline void swap(Math::MatrixTemplate<T>& a, Math::MatrixTemplate<T>& b)
  {
    a.swap(b);
  }
} //namespace std


#endif
