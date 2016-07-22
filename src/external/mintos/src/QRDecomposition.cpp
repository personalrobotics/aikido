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
#include "QRDecomposition.h"
#include "backsubstitute.h"
#include "Householder.h"
#include "MatrixPrinter.h"
#include <mintos/misc/errors.h>
using namespace std;

namespace Math {


template <class T>
bool QRDecomposition<T>::set(const MatrixT& A)
{
  QR.copy(A);
  tau.resize(Min(A.m,A.n));

  for (int i=0;i<Min(A.m,A.n);i++) {
    /* Compute the Householder transformation to reduce the j-th
       column of the matrix to a multiple of the j-th unit vector */
    
    VectorT c_full,c;
    QR.getColRef(i,c_full);
    c.setRef(c_full,i);
    T tau_i = HouseholderTransform (c);
    tau(i)=tau_i;
    
    /* Apply the transformation to the remaining columns and
       update the norms */
    
    if (i+1 < A.n) {
      MatrixT m;
      m.setRef(QR,i,i+1);
      HouseholderPreMultiply (tau_i, c, m);
    }
  }
  return true;
}

/* Solves the system A x = b using the QR factorisation,

 *  R x = Q^T b
 *
 * to obtain x. Based on SLATEC code. 
 */

template <class T>
void QRDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
  if(x.n == 0) x.resize(QR.n);
  Assert(QR.m == b.n);
  Assert(QR.n == x.n);

  /* compute rhs = Q^T b */
  VectorT rhs;
  QtMul(b,rhs);
  if(QR.m == QR.n) {
    /* Solve R x = rhs, storing x in-place */
    UBackSubstitute(QR,rhs,x);
    //UBackSubstitute(QR,x,x);
  }
  else if(QR.m > QR.n) {
    //solve the top part of R x = rhs
    MatrixT R1; R1.setRef(QR,0,0,1,1,QR.n,QR.n);
    VectorT rhs1; rhs1.setRef(rhs,0,1,QR.n);
    UBackSubstitute(R1,rhs1,x);
  }
  else {
    cerr<<"What do we do with m < n?"<<endl;
    MatrixPrinter mprint(QR); mprint.mode = MatrixPrinter::AsciiShade;
    cerr<<mprint<<endl;
    //solve the left part of R x = rhs
    MatrixT R1; R1.setRef(QR,0,0,1,1,QR.m,QR.m);
    VectorT x1; x1.setRef(x,0,1,QR.m);
    UBackSubstitute(R1,rhs,x1);
    getchar();
  }
}


/* Find the least squares solution to the overdetermined system 
 *
 *   A x = b 
 *  
 * for M >= N using the QR factorization A = Q R. 
 */

template <class T>
void QRDecomposition<T>::leastSquares(const VectorT& b, VectorT& x, VectorT& residual) const
{
  if(x.n == 0) x.resize(QR.n);
  Assert(QR.m >= QR.n);
  Assert(QR.m == b.n);
  Assert(QR.n == x.n);
  Assert(QR.m == residual.n);

  MatrixT R;
  R.setRef(QR,0,0,1,1,QR.n,QR.n);
  VectorT c;
  c.setRef(residual,0,1,QR.n);

  /* compute rhs = Q^T b */ 
  QtMul(b,residual);
  
  /* Solve R x = rhs */
  UBackSubstitute(R,c,x);
  
  /* Compute residual = b - A x = Q (Q^T b - R x) */
  c.setZero();
  QMul(residual,residual);
}

template <class T>
void QRDecomposition<T>::RBackSub(const VectorT& b,VectorT& x) const
{
  UBackSubstitute(QR,b,x);
}


// Form the product v=Q^T b 
template <class T>
void QRDecomposition<T>::QtMul(const VectorT& b, VectorT& v) const
{
  Assert(tau.n == Min(QR.m,QR.n));
  Assert(b.n == QR.m);
  v.copy(b);

  for (int i=0;i<Min(QR.m,QR.n); i++) {
    VectorT c,h,w;
    QR.getColRef(i,c);
    h.setRef(c,i);
    w.setRef(v,i);
    HouseholderApply(tau(i),h,w);
  }
}

// Form the product v=Q b 
template <class T>
void QRDecomposition<T>::QMul(const VectorT& b, VectorT& v) const
{
  Assert (tau.n == Min(QR.m,QR.n));
  Assert(b.n == QR.m);
  v.copy(b);

  for (int i = Min(QR.m,QR.n); i > 0 && i--;) {
    VectorT c,h,w;
    QR.getColRef(i,c);
    h.setRef(c,i);
    w.setRef(v,i);
    HouseholderApply (tau(i),h,w);
  }
}


template <class T>
void QRDecomposition<T>::getQ(MatrixT& Q) const
{
  Assert(tau.n == Min(QR.m,QR.n));
  Q.resize(QR.m,QR.m);

  int i;
  Q.setIdentity();
  for (i=Min(QR.m,QR.n);i>0 && i--;) {
    VectorT c,h;
    QR.getColRef(i,c);
    h.setRef(c,i);
    MatrixT m;
    m.setRef(Q,i,i);
    HouseholderPreMultiply(tau(i),h,m);
  }
}

template <class T>
void QRDecomposition<T>::getR(MatrixT& R) const
{
  Assert(tau.n == Min(QR.m,QR.n));
  R.resize(QR.m,QR.n);

  int i,j;
  for (i=0;i<QR.m;i++) {
    for (j=0;j<i && j<QR.n;j++)
      R(i,j)=0;
    for (j=i;j<QR.n;j++)
      R(i,j) = QR(i,j);
  }
}


/* Update a QR factorisation for A= Q R ,  A' = A + u v^T,

 * Q' R' = QR + u v^T
 *       = Q (R + Q^T u v^T)
 *       = Q (R + w v^T)
 *
 * where w = Q^T u.
 *
 * Algorithm from Golub and Van Loan, "Matrix Computations", Section
 * 12.5 (Updating Matrix Factorizations, Rank-One Changes)  
 */
#if 0
void QR_update (Matrix& Q, Matrix& R,
		VectorT& w, const VectorT& v)
{
  Assert(Q.m == R.m && Q.n == R.m);
  Assert(w.n == R.m);
  Assert(v.n == R.n);

  int j, k;
  T w0;

  /* Apply Given's rotations to reduce w to (|w|, 0, 0, ... , 0)
     
  J_1^T .... J_(n-1)^T w = +/- |w| e_1
  
  simultaneously applied to R,  H = J_1^T ... J^T_(n-1) R
  so that H is upper Hessenberg.  (12.5.2) */
  
  for (k = R.m - 1; k > 0; k--) {
    double c, s;
    double wk = w(k);
    double wkm1 = w(k-1);
    
    create_givens (wkm1, wk, &c, &s);
    apply_givens_vec (w, k - 1, k, c, s);
    apply_givens_qr (R.m, R.n, Q, R, k - 1, k, c, s);
  }
  
  w0 = w(0);
  
  /* Add in w v^T  (Equation 12.5.3) */
  for (j = 0; j < R.n; j++)
    R(0,j) += w0 * v(j);
  
  /* Apply Givens transformations R' = G_(n-1)^T ... G_1^T H
     Equation 12.5.4 */
  
  for (k=1; k<Min(R.m,R.n+1); k++) {
    double c, s;
    double diag = R(k-1,k-1);
    double offdiag = R(k,k-1);
    
    create_givens (diag, offdiag, &c, &s);
    apply_givens_qr (R.m, R.n, Q, R, k - 1, k, c, s);
    
    R(k,k-1)=0;
  } 
}
#endif

/*
void QR_QRsolve (Matrix& Q, Matrix& R, const VectorT& b, VectorT& x)
{
  Q.mulTranspose(b,x);
  UBackSubstitute(R,x,x);
}
*/



template <class T>
bool NRQRDecomposition<T>::set(const MatrixT& A)
{
  Assert(A.isSquare());
  int n=A.n;
  VectorT d(n);
  c.resize(n);
  QR.copy(A);

  int i,j,k;
  T scale,sigma,sum,tau;
  singular=false; 
  for (k=0;k<n-1;k++) {
    scale=0;
    for (i=k;i<n;i++) 
      scale=Max(scale,Abs(this->R(i,k)));
    if (scale == 0) { //Singular case. 
      singular=true;
      c(k)=d(k)=0.0;
    }
    else { //Form Qk and Qk*A. 
      for (i=k;i<n;i++) 
	QR(i,k) /= scale;
      for (sum=0,i=k;i<n;i++) 
	sum += Sqr(QR(i,k));
      sigma=SIGN(sqrt(sum),QR(k,k));
      QR(k,k) += sigma; 
      c(k)=sigma*QR(k,k);
      d(k) = -scale*sigma; 
      for (j=k+1;j<n;j++) {
	for (sum=0,i=k;i<n;i++)
	  sum += QR(i,k)*QR(i,j);
	tau=sum/c(k);
	for (i=k;i<n;i++) 
	  QR(i,j) -= tau*QR(i,k);
      }
    }
  }
  d(n-1)=QR(n-1,n-1);
  if (d(n-1) == 0) singular=true;
  return true;
}

//solve Qx=b i.e. x=Qtb
template <class T>
void NRQRDecomposition<T>::QBackSub(const VectorT& x,VectorT& b) const
{
  b.copy(x);
  int i,j;
  int n=QR.n;
  T sum,tau;
  for (j=0;j<n-1;j++) { //Form QT*b. 
    for (sum=0,i=j;i<n;i++) 
      sum += QR(i,j)*b(i);
    tau=sum/c(j);
    for (i=j;i<n;i++)
      b(i) -= tau*QR(i,j);
  }
}

template <class T>
void NRQRDecomposition<T>::backSub(const VectorT& x,VectorT& b) const
{
  VectorT y;
  QBackSub(x,y);
  UBackSubstitute(QR,y,b);
}

template <class T>
void NRQRDecomposition<T>::getQ(MatrixT& Q) const
{
  int n=c.n;
  Q.resize(n,n);
  VectorT Qi;
  Q.set(0);
  for(int i=0;i<n;i++) {
    Q.getRowRef(i,Qi);
    Qi(i)=1;
    QBackSub(Qi,Qi);
  }
}

template class QRDecomposition<float>;
template class QRDecomposition<double>;

}
