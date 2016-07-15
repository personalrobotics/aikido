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
#include <mintos/math/DiagonalMatrix.h>
#include "complex.h"
#include <mintos/misc/utils.h>
#include <mintos/misc/errors.h>

namespace Math {

template <class T>
DiagonalMatrixTemplate<T>::DiagonalMatrixTemplate()
{}

template <class T>
DiagonalMatrixTemplate<T>::DiagonalMatrixTemplate(const BaseT& diagVals)
:BaseT(diagVals)
{}

template <class T>
DiagonalMatrixTemplate<T>::DiagonalMatrixTemplate(int m)
:BaseT(m)
{}

template <class T>
DiagonalMatrixTemplate<T>::DiagonalMatrixTemplate(int m, T diagVal)
:BaseT(m,diagVal)
{}

template <class T>
DiagonalMatrixTemplate<T>::DiagonalMatrixTemplate(int m, const T* diagVals)
:BaseT(m,diagVals)
{}

template <class T>
void DiagonalMatrixTemplate<T>::operator *= (const MyT& m)
{
  inplaceComponentMul(m);
}

template <class T>
void DiagonalMatrixTemplate<T>::copyDiagonal(const MatrixT& m)
{
	if(!m.isSquare())
	{
		FatalError(MatrixError_ArgIncompatibleDimensions);
	}
	if(BaseT::n == 0)
	{
		resize(m.m);
	}
	else if(BaseT::n != m.m)
	{
		FatalError(MatrixError_DestIncompatibleDimensions);
	}
  m.getDiagCopy(0,*this);
}

template <class T>
void DiagonalMatrixTemplate<T>::mulMatrix(const MyT& a, const MyT& b)
{
  componentMul(a,b);
}

template <class T>
void DiagonalMatrixTemplate<T>::mulVector(const VectorT& a, VectorT& b) const
{
  b.componentMul(a,*this);
}


template <class T>
void DiagonalMatrixTemplate<T>::mulInverse(const VectorT& a, VectorT& b) const
{
  b.componentDiv(a,*this);
}

template <class T>
void DiagonalMatrixTemplate<T>::mulPseudoInverse(const VectorT& a, VectorT& b) const
{
  if(BaseT::n != a.n) FatalError(MatrixError_ArgIncompatibleDimensions);
  if(b.n == 0)
    b.resize(this->n);
  else if(b.n != this->n) FatalError(MatrixError_DestIncompatibleDimensions);
  
  ItT v=this->begin();
  VectorIterator<T> va=a.begin(),vb=b.begin();
  for(int i=0; i<this->n; i++, v++,va++,vb++)
    *vb = *va * PseudoInv(*v);
}

template <class T>
void DiagonalMatrixTemplate<T>::preMultiply(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.m);
  x.resize(this->n,a.n);
  ItT v=this->begin();
  MyT xrow,arow;
  for(int i=0;i<this->n;i++,v++) {
    x.getRowRef(i,xrow);
    a.getRowRef(i,arow);
    xrow.mul(arow,*v);
  }
}

template <class T>
void DiagonalMatrixTemplate<T>::postMultiply(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.n);
  x.resize(a.m,this->n);
  MyT xrow,arow;
  for(int i=0;i<a.m;i++) {
    x.getRowRef(i,xrow);
    a.getRowRef(i,arow);
    xrow.componentMul(arow,*this);
  }
}

template <class T>
void DiagonalMatrixTemplate<T>::preMultiplyTranspose(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.n);
  x.resize(this->n,a.m);
  ItT v=this->begin();
  MyT xrow,acol;
  for(int i=0;i<this->n;i++,v++) {
    x.getRowRef(i,xrow);
    a.getColRef(i,acol);
    xrow.mul(acol,*v);
  }
}

template <class T>
void DiagonalMatrixTemplate<T>::postMultiplyTranspose(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.m);
  x.resize(a.n,this->n);
  MyT xrow,acol;
  for(int i=0;i<a.n;i++) {
    x.getRowRef(i,xrow);
    a.getColRef(i,acol);
    xrow.componentMul(acol,*this);
  }
}

template <class T>
void DiagonalMatrixTemplate<T>::preMultiplyInverse(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.m);
  x.resize(this->n,a.n);
  ItT v=this->begin();
  MyT xrow,arow;
  for(int i=0;i<this->n;i++,v++) {
    x.getRowRef(i,xrow);
    a.getRowRef(i,arow);
    xrow.div(arow,*v);
  }
}

template <class T>
void DiagonalMatrixTemplate<T>::postMultiplyInverse(const MatrixT& a,MatrixT& x) const
{
  Assert(this->n == a.n);
  x.resize(a.m,this->n);
  MyT xrow,arow;
  for(int i=0;i<a.m;i++) {
    x.getRowRef(i,xrow);
    a.getRowRef(i,arow);
    xrow.componentDiv(arow,*this);
  }
}


template <class T>
void DiagonalMatrixTemplate<T>::setIdentity()
{
  this->set(One);
}

template <class T>
void DiagonalMatrixTemplate<T>::setInverse(const MyT& a)
{
  if(this->empty())
    resize(a.n);
  else if(this->size() != a.size()) {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,this->n,this->n,a.n,a.n);
  }
  ItT v=this->begin();
  ItT va=a.begin();
  for(int i=0; i<this->n; i++,v++,va++)
    *v = Inv(*va);
}

template <class T>
void DiagonalMatrixTemplate<T>::setPseudoInverse(const MyT& a)
{
  if(this->empty())
		resize(a.n);
  else if(this->size() != a.size()) {
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,this->n,this->n,a.n,a.n);
  }
  ItT v=this->begin();
  ItT va=a.begin();
  for(int i=0; i<this->n; i++,v++,va++)
    *v = PseudoInv(*va);
}

template <class T>
void DiagonalMatrixTemplate<T>::inplaceInverse()
{
  if(this->empty())
    FatalError(MatrixError_SizeZero);

  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    *v = Inv(*v);
}

template <class T>
void DiagonalMatrixTemplate<T>::inplacePseudoInverse()
{
	if(this->empty())
		FatalError(MatrixError_SizeZero);

  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    *v = PseudoInv(*v);
}

template <class T>
bool DiagonalMatrixTemplate<T>::isZero(T eps) const
{
  if(this->empty())
    FatalError(MatrixError_SizeZero);

  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    if(!FuzzyZero(*v,eps))
      return false;
  return true;
}

template <class T>
bool DiagonalMatrixTemplate<T>::isIdentity(T eps) const
{
	if(this->empty())
		FatalError(MatrixError_SizeZero);

  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    if(!FuzzyEquals(*v,(T)1,eps))
      return false;
  return true;
}

template <class T>
bool DiagonalMatrixTemplate<T>::isInvertible(T eps) const
{
	if(this->empty())
		FatalError(MatrixError_SizeZero);

  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    if(FuzzyZero(*v,eps))
      return false;
  return true;
}

template <class T>
T DiagonalMatrixTemplate<T>::determinant() const
{
  if(this->empty())
    FatalError(MatrixError_SizeZero);
  
  T det = One;
  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    det *= (*v);
  return det;
}

template <class T>
T DiagonalMatrixTemplate<T>::trace() const
{
  if(this->empty())
    FatalError(MatrixError_SizeZero);
  T trace=Zero;
  ItT v=this->begin();
  for(int i=0; i<this->n; i++,v++)
    trace+=(*v);
  return trace;
}

template class DiagonalMatrixTemplate<float>;
template class DiagonalMatrixTemplate<double>;
template class DiagonalMatrixTemplate<Complex>;

} //namespace Math
