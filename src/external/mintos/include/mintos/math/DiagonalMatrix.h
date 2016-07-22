#ifndef MATH_DIAGONAL_MATRIX_TEMPLATE_H
#define MATH_DIAGONAL_MATRIX_TEMPLATE_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"

namespace Math {

/** @ingroup Math
 * @brief A templated diagonal matrix, represented by a 
 * vector of the entries on the diagonal.
 */
template <class T>
class DiagonalMatrixTemplate : public VectorTemplate<T>
{
public:
  typedef class DiagonalMatrixTemplate<T> MyT; 
  typedef class VectorTemplate<T> BaseT; 
  typedef class VectorIterator<T> ItT;
  typedef class MatrixTemplate<T> MatrixT; 
  typedef class VectorTemplate<T> VectorT; 

  DiagonalMatrixTemplate();
  DiagonalMatrixTemplate(const BaseT&);
  DiagonalMatrixTemplate(int m);
  DiagonalMatrixTemplate(int m, T diagVal);
  DiagonalMatrixTemplate(int m, const T* diagVals);
  
  inline int numCols() const { return this->n; }
  inline int numRows() const { return this->n; }

  inline const MyT& operator = (const MyT& m) { BaseT::operator = (m); return *this; }

  inline void operator += (const MyT& m) { BaseT::operator +=(m); }
  inline void operator -= (const MyT& m) { BaseT::operator -=(m); }
  void operator *= (const MyT&);
  inline void operator *= (T c) { BaseT::operator *=(c); }
  inline void operator /= (T c) { BaseT::operator /=(c); }

  void copyDiagonal(const MatrixT& m);
  void mulMatrix(const MyT& a, const MyT& b); //m=a*b
  void mulVector(const VectorT&, VectorT&) const;  //out=m*in
  void mulInverse(const VectorT&, VectorT&) const; //out=m^-1*in
  void mulPseudoInverse(const VectorT&, VectorT&) const;
  void preMultiply(const MatrixT& a,MatrixT& x) const; //x=m*a
  void postMultiply(const MatrixT& a,MatrixT& x) const; //x=a*m
  void preMultiplyTranspose(const MatrixT& a,MatrixT& x) const; //x=m*at
  void postMultiplyTranspose(const MatrixT& a,MatrixT& x) const; //x=at*m
  void preMultiplyInverse(const MatrixT& a,MatrixT& x) const; //x=m^-1*a
  void postMultiplyInverse(const MatrixT& a,MatrixT& x) const; //x=a*m^-1

  void setIdentity();
  void setInverse(const MyT&);
  void setPseudoInverse(const MyT&);

  void inplaceInverse();
  void inplacePseudoInverse();

  bool isZero(T eps=Zero) const;
  bool isIdentity(T eps=Zero) const;
  bool isInvertible(T eps=Zero) const;
  T determinant() const;
  T trace() const;
};

  class Complex;
  typedef DiagonalMatrixTemplate<float> fDiagonalMatrix;
  typedef DiagonalMatrixTemplate<double> dDiagonalMatrix;
  typedef DiagonalMatrixTemplate<Complex> cDiagonalMatrix;
  typedef DiagonalMatrixTemplate<Real> DiagonalMatrix;

}

#endif
