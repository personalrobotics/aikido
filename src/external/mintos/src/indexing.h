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
#ifndef MATH_INDEXING_H
#define MATH_INDEXING_H

#include <mintos/math/VectorTemplate.h>
#include <mintos/math/MatrixTemplate.h>
#include <mintos/math/SparseVectorTemplate.h>
#include <mintos/math/SparseMatrixTemplate.h>
#include <vector>

/** @ingroup Math
 * @file math/indexing.h
 * @brief Utilities for matrix/vector access / manipulation with
 * index vectors.
 *
 * Be careful, little error checking is performed.  Each index
 * must be valid for the matrix/vector operated on.
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

///Sets A[indices] = fill
template <class T>
inline void SetElements(VectorTemplate<T>& A,const std::vector<int>& indices,T fill)
{
  for(size_t i=0;i<indices.size();i++)
    A(indices[i]) = fill;
}

/** @brief Sets A[indices] = fill
 *
 * The fill vector is of size indices.size().  For copying from
 * a vector to another using the same set of indices, use
 * CopyElements().
 */
template <class T>
inline void SetElements(VectorTemplate<T>& A,const std::vector<int>& indices,const VectorTemplate<T>& fill)
{
  Assert((int)indices.size() == fill.n);
  for(size_t i=0;i<indices.size();i++)
    A(indices[i]) = fill(i);
}

/// Sets B = A[indices]
template <class T>
inline void GetElements(const VectorTemplate<T>& A,const std::vector<int>& indices,VectorTemplate<T>& B)
{
  Assert((int)indices.size() == B.n);
  for(size_t i=0;i<indices.size();i++)
    B(i) = A(indices[i]);
}

/// Sets A[aindices] = B[bindices]
template <class T>
inline void CopyElements(VectorTemplate<T>& A,const std::vector<int>& aindices,
			 const VectorTemplate<T>& B,const std::vector<int>& bindices)
{
  Assert(aindices.size() == bindices.size());
  for(size_t i=0;i<aindices.size();i++) 
    A(aindices[i]) = B(bindices[i]);
}

///Removes the indexed elements of A (shrinking A)
template <class T>
void RemoveElements(VectorTemplate<T>& A,const std::vector<int>& indices)
{
  for(size_t i=0;i<indices.size();i++) {
    int start=indices[i]+1;
    int end=(i+1==indices.size() ? A.n : indices[i+1]);
    Assert(end >= start);
    Assert(start > 0 && end <= A.n);
    for(int j=start;j<end;j++) {
      A(j-i-1) = A(j);
    }
  }
  A.n = A.n - (int)indices.size();
}

///Removes the indexed elements of A (shrinking A)
template <class T>
void RemoveElements(SparseArray<T>& A,const std::vector<int>& indices)
{
  if(indices.empty()) return;
  SparseArray<T> res(A.n-(int)indices.size());
  int curIndex=indices[0];
  int curOffset=0;
  for(typename SparseArray<T>::const_iterator i=A.begin();i!=A.end();i++) {
    while(i->first>curIndex && curOffset < (int)indices.size()) {
      curOffset++;
      curIndex=indices[curOffset];
    }
    if(i->first != curIndex) {
      Assert(i->first >= curOffset);
      res.push_back(i->first-curOffset,i->second);
    }
  }
  A = res;
}

/** @brief The inverse of RemoveElements.
 *
 * Expands A such that the elements indexed by indices are set to fill.
 * A must be able to expand to size A.n + indices.size()
 */
template <class T>
void AddElements(VectorTemplate<T>& A,const std::vector<int>& indices,T fill)
{
  Assert(A.getCapacity() >= A.base + A.stride*(A.n+(int)indices.size()));
  A.n = A.n + (int)indices.size();
  Assert(A.isValid());
  for(int i=(int)indices.size()-1;i>=0;i--) {
    int start=indices[i]+1;
    int end=(i+1==(int)indices.size() ? A.n : indices[i+1]);
    Assert(end >= start);
    Assert(start > 0 && end <= A.n);
    for(int j=end-1;j>=start;j--) {
      A(j) = A(j-i-1);
    }
    A(indices[i]) = fill;
  }
}

///Sets the indexed rows of A to fill 
template <class T>
inline void SetRows(MatrixTemplate<T>& A,const std::vector<int>& indices,T fill)
{
  for(size_t i=0;i<indices.size();i++)
    A.setRow(indices[i],fill);
}



/// @brief Sets A[r,c] = fill for all r in rows and all c in columns
template <class T>
inline void SetElements(MatrixTemplate<T>& A,const std::vector<int>& rows,const std::vector<int>& cols,T fill)
{
  for(size_t i=0;i<rows.size();i++)
    for(size_t j=0;j<cols.size();j++)
      A(rows[i],cols[j]) = fill;
}


/** @brief Sets A[r,c] = fill for all r in rows and all c in columns
 *
 * The fill matrix is of size rows.size() x cols.size().  For copying from
 * a matrix to another using the same set of indices, use
 * CopyElements().
 */
template <class T>
inline void SetElements(MatrixTemplate<T>& A,const std::vector<int>& rows,const std::vector<int>& cols,const MatrixTemplate<T>& fill)
{
  Assert((int)rows.size() == fill.m);
  Assert((int)cols.size() == fill.n);
  for(size_t i=0;i<rows.size();i++)
    for(size_t j=0;j<cols.size();j++)
      A(rows[i],cols[j]) = fill(i,j);
}

/// Sets B[i,j] = A[rows[i],cols[j]] for all i and j
template <class T>
inline void GetElements(const MatrixTemplate<T>& A,const std::vector<int>& rows,const std::vector<int>& cols,MatrixTemplate<T>& B)
{
  Assert((int)rows.size() == B.m);
  Assert((int)cols.size() == B.n);
  for(size_t i=0;i<rows.size();i++)
    for(size_t j=0;j<cols.size();j++)
      B(i,j) = A(rows[i],cols[j]);
}

/// Sets A[ar,ac] = B[br,bc] for all (ar,br) in arows x brows, (ac,bc) in acols x bcols
template <class T>
inline void CopyElements(MatrixTemplate<T>& A,
			 const std::vector<int>& arows,const std::vector<int>& acols,
			 const MatrixTemplate<T>& B,
			 const std::vector<int>& brows,const std::vector<int>& bcols)
{
  Assert(arows.size() == brows.size());
  Assert(acols.size() == bcols.size());
  for(size_t i=0;i<arows.size();i++) 
    for(size_t j=0;j<acols.size();j++) 
      A(arows[i],acols[j]) = B(brows[i],bcols[j]);
}

///Removes the indexed rows and columns of A (shrinking A)
template <class T>
void RemoveElements(MatrixTemplate<T>& A,const std::vector<int>& rows,const std::vector<int>& cols)
{
  for(size_t i=0;i<rows.size();i++) {
    int rstart=rows[i]+1;
    int rend=(i+1==rows.size() ? A.m : rows[i+1]);
    Assert(rend >= rstart);
    Assert(rstart > 0 && rend <= A.m);
    for(size_t j=0;j<cols.size();j++) {
      int cstart=cols[j]+1;
      int cend=(j+1==cols.size() ? A.n : cols[i+1]);
      Assert(cend >= cstart);
      Assert(cstart > 0 && cend <= A.n);
      for(int k=rstart;k<rend;k++) 
	for(int l=cstart;l<cend;l++) 
	  A(k-i-1,l-j-1) = A(k,l);
    }
  }
  A.m = A.m - (int)rows.size();
  A.n = A.n - (int)cols.size();
}


///Sets the indexed columns of A to fill 
template <class T>
inline void SetColumns(MatrixTemplate<T>& A,const std::vector<int>& indices,T fill)
{
  for(size_t i=0;i<indices.size();i++)
    A.setCol(indices[i],fill);
}

///Sets the indexed rows of A to the vector fill 
template <class T>
inline void SetRows(MatrixTemplate<T>& A,const std::vector<int>& indices,const VectorTemplate<T>& fill)
{
  for(size_t i=0;i<indices.size();i++)
    A.copyRow(indices[i],fill);
}

///Sets the indexed columns of A to the vector fill 
template <class T>
inline void SetColumns(MatrixTemplate<T>& A,const std::vector<int>& indices,const VectorTemplate<T>& fill)
{
  for(size_t i=0;i<indices.size();i++)
    A.copyColumn(indices[i],fill);
}

/** @brief Sets the indexed rows of A to the columns of the matrix fill
 *
 * The fill matrix is indices.size() by A.n.  For copying from
 * a matrix to another using the same set of indices, use
 * CopyRows().
 */
template <class T>
inline void SetRows(MatrixTemplate<T>& A,const std::vector<int>& indices,const MatrixTemplate<T>& fill)
{
  VectorTemplate<T> filli;
  for(size_t i=0;i<indices.size();i++) {
    fill.getRowRef(i,filli);
    A.copyRow(indices[i],filli);
  }
}

/** @brief Sets the indexed columns of A to the columns of the matrix fill
 *
 * The fill matrix is A.m by indices.size().  For copying from
 * a matrix to another using the same set of indices, use
 * CopyColumns().
 */
template <class T>
inline void SetColumns(MatrixTemplate<T>& A,const std::vector<int>& indices,const MatrixTemplate<T>& fill)
{
  VectorTemplate<T> filli;
  for(size_t i=0;i<indices.size();i++) {
    fill.getColRef(i,filli);
    A.copyColumn(indices[i],filli);
  }
}

/** @brief Copies the indexed rows in A into B
 *
 * B has dimensions indices.size() by A.n.
 */
template <class T>
inline void GetRows(const MatrixTemplate<T>& A,const std::vector<int>& indices,MatrixTemplate<T>& B)
{
  VectorTemplate<T> Bi;
  for(size_t i=0;i<indices.size();i++) {
    B.getRowRef(i,Bi);
    A.getRowCopy(indices[i],Bi);
  }
}

/** @brief Copies the indexed columns in A into B
 *
 * B has dimensions A.m by indices.size()
 */
template <class T>
inline void GetColumns(const MatrixTemplate<T>& A,const std::vector<int>& indices,MatrixTemplate<T>& B)
{
  VectorTemplate<T> Bi;
  for(size_t i=0;i<indices.size();i++) {
    B.getColRef(i,Bi);
    A.getColCopy(indices[i],Bi);
  }
}

///Removes the rows of A indexed by the <em>sorted</em> list indices
template <class T>
void RemoveRows(MatrixTemplate<T>& A,const std::vector<int>& indices)
{
  VectorTemplate<T> src,dest;
  for(size_t i=0;i<indices.size();i++) {
    int start=indices[i]+1;
    int end=(i+1==indices.size() ? A.m : indices[i+1]);
    Assert(end >= start);
    Assert(start > 0 && end <= A.m);
    for(int j=start;j!=end;j++) {
      A.getRowRef(j,src);
      A.getRowRef(j-i-1,dest);
      dest.copy(src);
    }
  }
  A.m = A.m - (int)indices.size();
}

///Removes the columns of A indexed by the <em>sorted</em> indices
template <class T>
void RemoveColumns(MatrixTemplate<T>& A,const std::vector<int>& indices)
{
  VectorTemplate<T> src,dest;
  for(size_t i=0;i<indices.size();i++) {
    int start=indices[i]+1;
    int end=(i+1==indices.size() ? A.n : indices[i+1]);
    Assert(end >= start);
    Assert(start > 0 && end <= A.n);
    for(int j=start;j<end;j++) {
      A.getColRef(j,src);
      A.getColRef(j-i-1,dest);
      dest.copy(src);
    }
  }
  A.n = A.n - (int)indices.size();
}

///Removes the rows of A indexed by the <em>sorted</em> list indices
template <class T>
void RemoveRows(SparseMatrixTemplate_RM<T>& A,const std::vector<int>& indices)
{
  for(size_t i=0;i<indices.size();i++) {
    int start=indices[i]+1;
    int end=(i+1==indices.size() ? A.m : indices[i+1]);
    Assert(end >= start);
    Assert(start > 0 && end <= A.m);
    for(int j=start;j!=end;j++) {
      //copy row j to j-i-1
      A.rows[j-i-1].entries.clear();
      A.rows[j-i-1].entries=A.rows[j].entries();
    }
  }
  A.m = A.m - (int)indices.size();
}

///Removes the columns of A indexed by the <em>sorted</em> indices
template <class T>
void RemoveColumns(SparseMatrixTemplate_RM<T>& A,const std::vector<int>& indices)
{
  for(int i=0;i<A.m;i++)
    RemoveElements(A.rows[i],indices);
  A.n = A.n - (int)indices.size();
}

/*@}*/

} //namespace Math

#endif




