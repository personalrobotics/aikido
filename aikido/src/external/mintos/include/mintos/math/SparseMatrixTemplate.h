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

#ifndef MATH_SPARSE_MATRIX_TEMPLATE_H
#define MATH_SPARSE_MATRIX_TEMPLATE_H

#include <mintos/math/MatrixTemplate.h>
#include <mintos/math/VectorTemplate.h>
#include "SparseVectorTemplate.h"
#include <vector>
#include <iostream>

/** @file math/SparseMatrixTemplate.h
 * @brief Several sparse matrix classes.
 *
 * Conventions:
 * - A non-zero element is called an entry.
 * - initialize() clears all existing entries before resizing.
 * - resize() does not clear existing entries.  Note: if any columns
 *   have entries with index higher than the new n, they are not cleared!
 * - clear() erases the entire structure.
 * - setZero() erases all entries, leaving the rest of the structure intact
 * - The (i,j) operator automatically inserts a 0 entry if it does not
 *   find it in the matrix.
 * - Low-level manipulation is done using the insertEntry()/getEntry()
 *   /eraseEntry() methods.
 * - get(Matrix) returns a dense matrix.  set(Matrix) sets the sparse
 *   matrix from a dense one, setting elements Aij to 0 if |Aij| <= eps
 * - The matrices are written in the following format:
 *   m n nnz [that is, the # of nonzero entries]
 *   [for each entry (i,j) = v]
 *   i j v
 *
 * Individual functionality varies.
 */

namespace Math {

/** @ingroup Optimization
 * @brief Row-major sparse matrix.
 *
 * That is, the rows are stored in contiguous order, with each row
 * a sparse array of entries.
 *
 * mulTranspose operations may not be cache-friendly!
 */
template <class T>
class SparseMatrixTemplate_RM
{
 public:
  typedef SparseMatrixTemplate_RM<T> MyT;
  typedef VectorTemplate<T> VectorT;
  typedef MatrixTemplate<T> MatrixT;
  typedef SparseVectorTemplate<T> SparseVectorT;

  SparseMatrixTemplate_RM();
  SparseMatrixTemplate_RM(int m,int n);
  SparseMatrixTemplate_RM(const MyT&);
  void initialize(int m, int n);
  void resize(int m, int n);
  void clear();

  T& operator () (int i, int j);
  void insertEntry(int i, int j, const T&);
  T* getEntry(int i, int j);  ///< returns NULL if the entry does not exist
  const T* getEntry(int i,int j) const;
  void eraseEntry(int i, int j);

  void copy(const MyT& m);
  template <class T2>
  void copy(const SparseMatrixTemplate_RM<T2>& m);  //cast
  void copySubMatrix(int i, int j, const MyT&);
  void copySubMatrix(int i, int j, const MatrixT&,T zeroTol=0);
  void swap(MyT&);
  void set(const MatrixT& m,T zeroTol=0);
  void setZero();
  void setIdentity();
  void setNegative(const MyT&);
  void setTranspose(const MyT&);
  void setAdjoint(const MyT&);
  void getCopy(MyT& m) const { m.copy(*this); }
  void get(MatrixT& m) const;
  void getTranspose(MyT& m) const { m.setTranspose(*this); }

  void mul(const MyT&, T s);
  void mul(const VectorT& y,VectorT& x) const;           //x = this*y
  void mulTranspose(const VectorT& y,VectorT& x) const;  //x = this^t*y
  void madd(const VectorT& y,VectorT& x) const;          //x = x+this*y
  void maddTranspose(const VectorT& y,VectorT& x) const; //x = x+this^t*y
  void mul(const MatrixT& y,MatrixT& x) const;           //x = this*y
  void mulTranspose(const MatrixT& y,MatrixT& x) const;  //x = this^t*y
  void copyRow(int i,const VectorT& x,T zeroTol=0);
  void copyRow(int i,const SparseVectorT& x);
  void copyCol(int j,const VectorT& x,T zeroTol=0);
  void copyCol(int j,const SparseVectorT& x);
  T dotRow(int i, const VectorT&) const;
  T dotCol(int j, const VectorT&) const;
  void inplaceNegative();
  void inplaceMul(T c);
  void inplaceDiv(T c);
  void inplaceMulRow(int i,T c);
  void inplaceMulCol(int i,T c);
  void eraseZeros(T zeroTol=0);

  bool isValid() const;
  inline bool isEmpty() const { return m==0&&n==0; }
  inline bool isSquare() const { return m==n; }
  inline bool isValidRow(int i) const { return 0<=i&&i<m; }
  inline bool isValidCol(int j) const { return 0<=j&&j<n; }
  inline bool isValidIndex(int i,int j) const { return 0<=i&&i<m&&0<=j&&j<n; }
  size_t numNonZeros() const;

  typedef SparseArray<T> RowT;
  typedef typename SparseArray<T>::iterator RowIterator;
  typedef typename SparseArray<T>::const_iterator ConstRowIterator;

  std::vector<RowT> rows;
  int m,n;
};

/** @ingroup Optimization
 * @brief Row-major, compressed-row sparse matrix.
 *
 * Like the above, except the rows are all fixed in a compressed,
 * contiguous block of index/value pairs.  The number of nonzero
 * entries (and their locations) must be known in advance.
 */
template <class T>
class SparseMatrixTemplate_CR
{
 public:
  typedef SparseMatrixTemplate_CR<T> MyT;
  typedef VectorTemplate<T> VectorT;
  typedef MatrixTemplate<T> MatrixT;

  SparseMatrixTemplate_CR();
  ~SparseMatrixTemplate_CR();
  void initialize(int m, int n, int num_entries);
  void resize(int m, int n, int num_entries);
  void clear();

  T* getEntry(int i,int j);
  const T* getEntry(int i,int j) const;

  void copy(const MyT&);
  template <class T2>
  void copy(const SparseMatrixTemplate_CR<T2>&);
  void set(const MatrixT&,T zeroTol=Zero);
  void getCopy(MyT& m) const { m.copy(*this); }
  void get(MatrixT&) const;

  void mul(const MyT&, T s);
  void mul(const VectorT& y, VectorT& x) const;		 //x = this*y;
  void mulTranspose(const VectorT& y, VectorT& x) const; //x = this^t*y
  void madd(const VectorT& y,VectorT& x) const;          //x = x+this*y
  void maddTranspose(const VectorT& y,VectorT& x) const; //x = x+this^t*y
  void mul(const MatrixT& w, MatrixT& v) const;            //dense matrix multiply v = this*w
  void mulTranspose(const MatrixT& w, MatrixT& v) const; //w = this^t*w
  T dotRow(int i, const VectorT&) const;
  T dotCol(int j, const VectorT&) const;
  //assumes lower triangle is filled, upper is transpose
  T dotSymmL(int i, const VectorT&) const;
  void inplaceMul(T c);
  void inplaceDiv(T c);
  void inplaceMulRow(int i,T c);
  void inplaceMulCol(int i,T c);

  bool isValid() const;
  inline bool isEmpty() const { return m == 0 && n == 0; }
  inline bool hasDims(int M, int N) const { return m == M && n == N; }
  inline bool isSquare() const { return m == n; }
  inline bool isValidRow(int i) const { return i >= 0 && i < m; }
  inline bool isValidCol(int j) const { return j >= 0 && j < n; }
  inline bool isValidIndex(int i,int j) const { return isValidRow(i)&&isValidCol(j); }

  inline int* rowIndices(int i) const { return col_indices + row_offsets[i]; }
  inline T* rowValues(int i) const { return val_array + row_offsets[i]; }
  inline int numRowEntries(int i) const { return row_offsets[i+1]-row_offsets[i]; }

  /***************************************************************
   * Compressed row format:
   * row_offsets indexes into the column index array, by row, and is size m+1.
   * The number of entries per row can be determined by subtraction from
   *   the next value (the last value in row_offsets is num_entries)
   * The col_indices array marks the column of the corresponding
   *   element in the value array, and each column is sorted in
   *   increasing order.
   ****************************************************************/
  int* row_offsets;
  int* col_indices;
  T* val_array;

  int m,n;
  int num_entries;

  static void self_test();
  static void self_test(int m, int n, int nnz);
};

///returns true if all elements of x are finite
template <class T>
inline bool IsFinite(const SparseMatrixTemplate_RM<T>& x)
{
  for(size_t i=0;i<x.rows.size();i++) {
    typename SparseMatrixTemplate_RM<T>::ConstRowIterator j;
    for(j=x.rows[i].begin();j!=x.rows[i].end();j++)
      if(!IsFinite(j->second)) return false;
  }
  return true;
}


class Complex;
typedef SparseMatrixTemplate_RM<float> fSparseMatrix_RM;
typedef SparseMatrixTemplate_RM<double> dSparseMatrix_RM;
typedef SparseMatrixTemplate_RM<Complex> cSparseMatrix_RM;
typedef SparseMatrixTemplate_CR<float> fSparseMatrix_CR;
typedef SparseMatrixTemplate_CR<double> dSparseMatrix_CR;
typedef SparseMatrixTemplate_CR<Complex> cSparseMatrix_CR;

template <class T>
std::ostream& operator << (std::ostream&, const SparseMatrixTemplate_RM<T>&);
template <class T>
std::istream& operator >> (std::istream&, SparseMatrixTemplate_RM<T>&);
template <class T>
std::ostream& operator << (std::ostream&, const SparseMatrixTemplate_CR<T>&);

} //namespace Math

#endif
