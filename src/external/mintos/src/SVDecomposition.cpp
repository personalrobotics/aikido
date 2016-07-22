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
#include <mintos/math/SVDecomposition.h>
#include "misc.h"
#include "QRDecomposition.h"
#include "ASCIIShade.h"
#include <algorithm>
#include <mintos/misc/errors.h>
#include <vector>

namespace Math {

#define CHECKNAN(x) {  \
    if(IsNaN(x)) { fprintf(stderr,"Error in SVD: NaN encountered\n"); return false; } \
    if(IsInf(x)) { fprintf(stderr,"Error in SVD: inf encountered\n"); return false; } \
  }


#define SIGN(a,b) ((b) >= 0.0 ? Abs(a) : -Abs(a))


template <class T>
SVDecomposition<T>::SVDecomposition()
:maxIters(100),epsilon(Epsilon)
{}

template <class T>
SVDecomposition<T>::SVDecomposition(const MatrixT& A)
:maxIters(100),epsilon(Epsilon)
{
	set(A);
}

template <class T>
bool SVDecomposition<T>::set(const MatrixT& A)
{
	int m = A.m;
	int n = A.n;
	U = A;
	W.resize(n);
	V.resize(n,n);

/*
Given a matrix a[1..m,1..n], this routine computes its singular value decomposition, A =
U.W.Vt. The matrix U replaces a on output. The diagonal matrix of singular values W is output
as a vector w[1..n]. The matrix V (not the transpose V T ) is output as v[1..n,1..n].
*/

	int flag,i,its,j,jj,k,l,nm;
	T anorm,c,f,g,h,s,scale,x,y,z;

	VectorT rv1(n);
	g=scale=anorm=0; //Householder reduction to bidiagonal form.
	for (i=0;i<n;i++) {
		l=i+1;
		rv1[i]=scale*g;
		g=s=scale=0.0;
		if (i < m) {
			for (k=i;k<m;k++) scale += Abs(U(k,i));
			if (!FuzzyZero(scale,epsilon)) {
			    for (k=i;k<m;k++) {
					U(k,i) /= scale;
					s += U(k,i)*U(k,i);
					}
				f=U(i,i);
				g = -SIGN(Sqrt(s),f);
				h=f*g-s; 
				U(i,i)=f-g;
				for (j=l;j<n;j++) {
					for (s=0.0,k=i;k<m;k++) s += U(k,i)*U(k,j);
					f=s/h;
					CHECKNAN(f);
					for (k=i;k<m;k++) U(k,j) += f*U(k,i);
				}
				for (k=i;k<m;k++) U(k,i) *= scale;
			}
		}
		W[i]=scale *g;
		g=s=scale=0.0;
		if (i < m && i+1 != n) {
			for (k=l;k<n;k++) scale += Abs(U(i,k));
			if (!FuzzyZero(scale,epsilon)) {
			    for (k=l;k<n;k++) {
					U(i,k) /= scale;
					s += U(i,k)*U(i,k);
					}
				f=U(i,l);
				g = -SIGN(Sqrt(s),f);
				h=f*g-s;
				U(i,l)=f-g;
				for (k=l;k<n;k++) {
				  rv1[k]=U(i,k)/h;
				  CHECKNAN(rv1[k]);
				}
				for (j=l;j<m;j++) {
					for (s=0.0,k=l;k<n;k++) s += U(j,k)*U(i,k);
					for (k=l;k<n;k++) U(j,k) += s*rv1[k];
				}
				for (k=l;k<n;k++) U(i,k) *= scale;
			}
		}
		anorm=Max(anorm,(Abs(W[i])+Abs(rv1[i])));
	}
	for (i=n-1;i>=0;i--) { //Accumulation of right-hand transformations.
		if (i < n-1) {
			if (!FuzzyZero(g,epsilon)) {
				for (j=l;j<n;j++) { //Double division to avoid possible underflow.
				  V(j,i)=(U(i,j)/U(i,l))/g;
				  CHECKNAN(V(j,i));
				}
				for (j=l;j<n;j++) {
					for (s=0.0,k=l;k<n;k++) s += U(i,k)*V(k,j);
					for (k=l;k<n;k++) V(k,j) += s*V(k,i);
				}
			}
			for (j=l;j<n;j++) V(i,j)=V(j,i)=0.0;
		}
		V(i,i)=1.0;
		g=rv1[i];
		l=i;
	}
  for (i=::Min(m,n)-1;i>=0;i--) { //Accumulation of left-hand transformations.
		l=i+1;
		g=W[i];
		for (j=l;j<n;j++) U(i,j)=0.0;
		if (!FuzzyZero(g,epsilon)) {
			g=Inv(g);
			for (j=l;j<n;j++) {
				for (s=0.0,k=l;k<m;k++) s += U(k,i)*U(k,j);
				f=(s/U(i,i))*g;
				CHECKNAN(f);
				for (k=i;k<m;k++) U(k,j) += f*U(k,i);
			}
			for (j=i;j<m;j++) U(j,i) *= g;
		} else for (j=i;j<m;j++) U(j,i)=0.0;
		++U(i,i);
	}

	for (k=n-1;k>=0;k--) { //Diagonalization of the bidiagonal form: Loop over singular values, and over allowed iterations.
		for (its=1;its<=maxIters;its++) {
			flag=1;
			for (l=k;l>=0;l--) { //Test for splitting.
				nm=l-1; //Note that rv1[1] is always zero.
				if ((Abs(rv1[l])+anorm) == anorm) {
					flag=0;
					break;
				}
				if ((Abs(W[nm])+anorm) == anorm) break;
			}
			if (flag) {
				c=0.0; //Cancellation of rv1[l], if l > 1.
				s=1.0;
				for (i=l;i<=k;i++) {
					f=s*rv1[i];
					rv1[i]=c*rv1[i];
					if ((Abs(f)+anorm) == anorm) break;
					g=W[i];
					h=pythag(f,g);
					CHECKNAN(h);
					W[i]=h;
					h=Inv(h);
					CHECKNAN(h);
					c=g*h;
					s = -f*h;
					for (j=0;j<m;j++) {
						y=U(j,nm);
						z=U(j,i);
						U(j,nm)=y*c+z*s;
						U(j,i)=z*c-y*s;
					}
				}
			}
			z=W[k];
			if (l == k) { //Convergence.
				if (z < 0.0) { //Singular value is made nonnegative.
					W[k] = -z;
					for (j=0;j<n;j++) V(j,k) = -V(j,k);
				}
				break;
			}
			if (its == maxIters) 
			{
			  //printf("no convergence in %d svdcmp iterations\n",maxIters);
				return false;
			}
			x=W[l]; //Shift from bottom 2-by-2 minor.
			nm=k-1;
			y=W[nm];
			g=rv1[nm];
			h=rv1[k];
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			CHECKNAN(f);
			g=pythag(f,(T)1);
			CHECKNAN(g);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			CHECKNAN(f);
			c=s=1.0; //Next QR transformation:
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1[i];
				y=W[i];
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				CHECKNAN(z);
				rv1[j]=z;
				c=f/z;
				s=h/z;
				CHECKNAN(c); CHECKNAN(s);
				f=x*c+g*s;
				g = g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=0;jj<n;jj++) {
					x=V(jj,j);
					z=V(jj,i);
					V(jj,j)=x*c+z*s;
					V(jj,i)=z*c-x*s;
				}
				z=pythag(f,h);
				CHECKNAN(z);
				W[j]=z; //Rotation can be arbitrary if z = 0.
				if (!FuzzyZero(z,epsilon)) {
					z=Inv(z);
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=0;jj<m;jj++) {
					y=U(jj,j);
					z=U(jj,i);
					U(jj,j)=y*c+z*s;
					U(jj,i)=z*c-y*s;
				}
			}
		rv1[l]=0.0;
		rv1[k]=f;
		W[k]=x;
		}
	}
	return true;
}

using namespace std;

template <class T>
void SVDecomposition<T>::setIdentity(int m,int n)
{
  U.resize(m,n,0);
  W.resize(n,0);
  V.resize(n,n);
  int nsv=::Min(m,n);
  for(int i=0;i<nsv;i++) {
    U(i,i) = 1;
    W(i) = 1;
  }
  V.setIdentity();
}

template <class T>
void SVDecomposition<T>::setZero(int m,int n)
{
  U.resize(m,n,0);
  W.resize(n,0);
  V.resize(n,n);
  int nsv=::Min(m,n);
  for(int i=0;i<nsv;i++) {
    U(i,i) = 1;
  }
  V.setIdentity();
}

template <class T>
void SVDecomposition<T>::resize(int m,int n)
{
  U.resize(m,n);
  W.resize(n);
  V.resize(n,n);
}


template <class T>
int SVDecomposition<T>::getRank() const
{
  int nnz=0;
  for(int i=0;i<W.n;i++)
    if(W[i] > epsilon) nnz++;
  return nnz;
}

template <class T>
int SVDecomposition<T>::getNull() const
{
  return U.n-getRank();
}

template <class T>
void SVDecomposition<T>::backSub(const VectorT& b, VectorT& x) const
{
  if(x.n == 0) x.resize(U.n);
  Assert(x.n == U.n); 
  Assert(b.n == U.m); 
  
  VectorT tmp(U.n);
  for (int j=0;j<U.n;j++) { //Calculate Ut*b.
    if (W[j] > epsilon) { //Nonzero result only if wj is nonzero.
      tmp[j] = U.dotCol(j,b) / W[j];
    }
    else 
      tmp[j]=0;
  }
  //Matrix multiply by V to get answer.
  V.mul(tmp,x);
}

template <class T>
void SVDecomposition<T>::dampedBackSub(const VectorT& b,T lambda, VectorT& x) const
{
  if(x.n == 0) x.resize(U.n);
  Assert(x.n == U.n); 
  Assert(b.n == U.m); 
  
  VectorT tmp(U.n);
  for (int j=0;j<U.n;j++) { //Calculate Ut*b.
    tmp[j]=U.dotCol(j,b)/(W[j]+lambda);
  }
  V.mul(tmp,x);
}

template <class T>
void SVDecomposition<T>::getInverse(MatrixT& Ainv) const
{
  //A = U.W.Vt
  //A^-1 = V.W^-1.Ut
  Ainv.resize(U.n,U.m);
  VectorT tmp(U.n),Ainvi;
  int i,j;
  for(i=0; i<U.m; i++) {
    T s;
    for (j=0;j<U.n;j++) { //Calculate tmp=W^-t*Ut*delta(i).
      s=0.0;
      if (W[j] > epsilon) { //Nonzero result only if wj is nonzero.
	s = U(i,j) / W[j];
      }
      tmp[j]=s;
    }
    //MatrixT multiply by V to get column of Ainv
    Ainv.getColRef(i,Ainvi);
    V.mul(tmp,Ainvi);
  }
}

template <class T>
void SVDecomposition<T>::getDampedPseudoInverse(MatrixT& Ainv,T lambda) const
{
  //A = U.W.Vt
  //A^-1 = V.W^-1.Ut
  Ainv.resize(U.n,U.m);
  VectorT tmp(U.n),Ainvi;
  int i,j;
  for(i=0; i<U.m; i++)	{
    for (j=0;j<U.n;j++) { //Calculate tmp=W^-t*Ut*delta(i).
      tmp[j] = U(i,j) / (W[j]+lambda);
    }
    //MatrixT multiply by V to get column of Ainv
    Ainv.getColRef(i,Ainvi);
    V.mul(tmp,Ainvi);
  }
}

template <class T>
void SVDecomposition<T>::nullspaceComponent(const VectorT& x,VectorT& xNull) const
{
  /*
  VectorT Ui;
  for(int i=0;i<U.n;i++) {
    if(W(i) < 1e-6) continue;
    U.getCol(i,Ui);
    for(int j=i;j<U.n;j++) {
      if(W(j) < 1e-6) continue;
      Real UUij = U.dotCol(j,Ui); 
      if(!FuzzyEquals(UUij,Delta(i,j))) {
	cout<<i<<" "<<j<<endl;
	cout<<"Ack, UUij = "<<UUij<<" != "<<Delta(i,j)<<endl;
	cout<<W(i)<<" "<<W(j)<<endl;
      }
      Assert(FuzzyEquals(UUij,Delta(i,j)));
    }
    }*/

  VectorT y;
  V.mulTranspose(x,y);
  for(int i=0;i<W.n;i++)
    if(W(i) < epsilon) y(i)=Zero;
  V.mul(y,xNull);
  /*
  //this must be equivalent to the psuedoinverse method
  VectorT xNull2;
  V.mulTranspose(x,y);
  W.mulVectorT(y,y);
  VectorT z;
  U.mul(y,z);
  backSub(z,xNull2);
  xNull2-=xNull;
  cout<<"Difference between 2 approaches: "; xNull2.print();
  Assert(xNull2.maxAbsElement() < 1e-4);
  */
}

template <class T>
void SVDecomposition<T>::getNullspace(MatrixT& N) const
{
  N.resize(U.n,getNull());
//want all c s.t. U.W.Vt.c = 0
//if W.v = 0, U.W.v = 0, so c = V.v satisfies the criteria
//A v that satisfies W.v is delta(i) where W(i) = 0, so we pull out all
//columns of V that correspond to zero elements of W
  int nz=0;
  for(int j=0;j<U.n;j++) {
    if(W[j] <= epsilon) {
      VectorT Vj;
      V.getColRef(j,Vj);
      N.copyCol(nz,Vj);
      nz++;
    }
  }
}

struct IndexAndPriority
{
  inline bool operator < (const IndexAndPriority& b) const { return priority<b.priority; }
  int index;
  double priority;
};

template <class T>
void SVDecomposition<T>::sortSVs()
{
  std::vector<IndexAndPriority> order;
  for(int i=0;i<W.n;i++) {
    IndexAndPriority temp;
    temp.index = i;
    temp.priority = -Abs(W(i));
    order.push_back(temp);
  }
  //sorts from high to low
  std::sort(order.begin(),order.end());

  //permute the entries of W, cols of V, and cols of U
  MatrixT tempU(U.m,U.n),tempV(V.m,V.n);
  DiagonalMatrixT tempW(W.n);
  for(int i=0;i<W.n;i++) {
    tempW(i)=W(order[i].index);
    VectorT tempv,v;
    tempU.getColRef(i,tempv);
    U.getColRef(order[i].index,v);
    tempv.copy(v);

    tempv.clear(); v.clear();
    tempV.getColRef(i,tempv);
    V.getColRef(order[i].index,v);
    tempv.copy(v);
  }
  swap(tempU,U);
  swap(tempW,W);
  swap(tempV,V);
}









template <class T>
RobustSVD<T>::RobustSVD()
  :zeroElementEpsilon(Epsilon),preMultiply(true),postMultiply(false)
{
  svd.maxIters = 500;
}

template <class T>
RobustSVD<T>::RobustSVD(const MatrixT& A)
  :zeroElementEpsilon(Epsilon),preMultiply(true),postMultiply(false)
{
  svd.maxIters = 500;
  set(A);
}


template <class T>
bool RobustSVD<T>::setConditioned(const MatrixT& A)
{
  MatrixT Atemp;  
  calcConditioning(A);
  if(preMultiply) {
    Pre.preMultiplyInverse(A,Atemp);
    if(postMultiply) Post.postMultiplyInverse(Atemp,Atemp);
  }
  else if(postMultiply) Post.postMultiplyInverse(A,Atemp);
  else Atemp.copy(A);

  for(int i=0;i<A.m;i++) {
    for(int j=0;j<A.n;j++) {
      if(Abs(Atemp(i,j)) <= zeroElementEpsilon)
	Atemp(i,j) = 0;
    }
  }

  return svd.set(Atemp);
}

template<> bool RobustSVD<double>::set(const MatrixT& A)
{
  MatrixT Atemp;  
  calcConditioning(A);
  if(preMultiply) {
    Pre.preMultiplyInverse(A,Atemp);
    if(postMultiply) Post.postMultiplyInverse(Atemp,Atemp);
  }
  else if(postMultiply) Post.postMultiplyInverse(A,Atemp);
  else Atemp.copy(A);

  for(int i=0;i<A.m;i++) {
    for(int j=0;j<A.n;j++) {
      if(Abs(Atemp(i,j)) <= zeroElementEpsilon)
	Atemp(i,j) = 0;
    }
  }

  if(svd.set(Atemp)) 
    return true;

  //cout<<"Couldn't set SVD of conditioned matrix "<<endl;
  //OutputASCIIShade(cout,Atemp); cout<<endl;
  //getchar();

  QRDecomposition<double> QR;
  if(QR.set(Atemp)) {
    MatrixT R;   //should we use Atemp as storage?
    QR.getR(R);
    RobustSVD<double> Rsvd;
    Rsvd.zeroElementEpsilon = zeroElementEpsilon;
    Rsvd.preMultiply = false;
    Rsvd.postMultiply = true;
    Rsvd.svd.maxIters = svd.maxIters;
    Rsvd.svd.epsilon = svd.epsilon;

    //avoid some memory allocation
    Rsvd.svd.W.setRef(svd.W);
    Rsvd.svd.V.setRef(svd.V);
    if(Rsvd.setConditioned(R)) { 
      //cout<<"Robust svd of QR decomposed matrix succeeded!"<<endl;
      //A' = Q.R = Q.U.W.Vt.Post => U' = Q.U, Post'' = Post.Post'
      MatrixT Q;
      QR.getQ(Q);
      svd.U.mul(Q,Rsvd.svd.U);
      Post.mulMatrix(Post,Rsvd.Post);

      /*
      { //check
	MatrixT temp,temp2;
	getInverse(temp);
	temp2.mul(A,temp);
	for(int i=0;i<A.m;i++) if(temp2(i,i) != 0) temp2(i,i) -= 1;
	cout<<"Robust SVD Error "<<temp2.maxAbsElement()<<endl;
      }
      getchar();
      */
      return true;
    }
  }
  return false;
}

template<> bool RobustSVD<float>::set(const MatrixT& A)
{
  RobustSVD<double> dsvd;
  dsvd.zeroElementEpsilon = zeroElementEpsilon;
  dsvd.preMultiply = preMultiply;
  dsvd.postMultiply = postMultiply;
  dsvd.svd.epsilon = svd.epsilon;
  dsvd.svd.maxIters = svd.maxIters;
  dMatrix dA; dA.copy(A);
  if(!dsvd.set(dA)) return false;
  
  Pre.resize(A.m);
  Post.resize(A.n);
  Pre.copy(dsvd.Pre);
  Post.copy(dsvd.Post);

  svd.U.resize(A.m,A.n);
  svd.W.resize(A.n);
  svd.V.resize(A.n,A.n);
  svd.U.copy(dsvd.svd.U);
  svd.W.copy(dsvd.svd.W);
  svd.V.copy(dsvd.svd.V);
  return true;
}

template <class T>
void RobustSVD<T>::setIdentity(int m,int n)
{
  Pre.resize(m,1);
  Post.resize(n,1);
  svd.setIdentity(m,n);
}

template <class T>
void RobustSVD<T>::setZero(int m,int n)
{
  Pre.resize(m,1);
  Post.resize(n,1);
  svd.setZero(m,n);
}

template <class T>
void RobustSVD<T>::resize(int m,int n)
{
  Pre.resize(m);
  svd.resize(m,n);
  Post.resize(m);
}

template <class T>
int RobustSVD<T>::getRank() const
{
  return svd.getRank();
}

template <class T>
int RobustSVD<T>::getNull() const
{
  return svd.getNull();
}

template <class T>
void RobustSVD<T>::backSub(const VectorT& b, VectorT& x) const
{
  VectorT temp;
  Pre.mulInverse(b,temp);
  svd.backSub(temp,x);
  Post.mulInverse(x,x);
}

template <class T>
void RobustSVD<T>::dampedBackSub(const VectorT& b,T lambda, VectorT& x) const
{
  VectorT temp;
  Pre.mulInverse(b,temp);
  svd.dampedBackSub(temp,lambda,x);
  Post.mulInverse(x,x);  
}

template <class T>
void RobustSVD<T>::nullspaceComponent(const VectorT& x,VectorT& xNull) const
{
  VectorT temp;
  Post.mulVector(x,temp);
  svd.nullspaceComponent(temp,xNull);
  Post.mulInverse(xNull,xNull);
}

template <class T>
void RobustSVD<T>::getInverse(MatrixT& Ainv) const
{
  //A = Pre.U.W.Vt.Post
  //A^-1 = Post^-1.V.W^-1.Ut.Pre^-1
  svd.getInverse(Ainv);
  Pre.postMultiplyInverse(Ainv,Ainv);
  Post.preMultiplyInverse(Ainv,Ainv);
}

template <class T>
void RobustSVD<T>::getDampedPseudoInverse(MatrixT& Ainv,T lambda) const
{
  //A = Pre.U.W.Vt.Post
  //A^-1 = Post^-1.V.W^-1.Ut.Pre^-1
  svd.getDampedPseudoInverse(Ainv,lambda);
  Pre.postMultiplyInverse(Ainv,Ainv);
  Post.preMultiplyInverse(Ainv,Ainv);
}

template <class T>
void RobustSVD<T>::getNullspace(MatrixT& N) const
{
  svd.getNullspace(N);
  Post.preMultiplyInverse(N,N);
}

template <class T>
void RobustSVD<T>::calcConditioning(const MatrixT& A)
{
  Pre.resize(A.m);
  Post.resize(A.n);
  if(preMultiply && postMultiply) {
    cout<<"RobustSVD: Warning, using both pre/postmultiply aren't done yet"<<endl;
    for(int i=0;i<A.m;i++) {
      Pre(i) = 0;
      for(int j=0;j<A.n;j++) 
	Pre(i) = Max(Pre(i),Abs(A(i,j)));
      if(Pre(i) == 0) Pre(i) = 1;   //entire row is zero
    }
    Post.set(One);
  }
  else if(preMultiply) {
    for(int i=0;i<A.m;i++) {
      Pre(i) = 0;
      for(int j=0;j<A.n;j++) 
	Pre(i) = Max(Pre(i),Abs(A(i,j)));
      if(Pre(i) == 0) Pre(i) = 1;   //entire row is zero
    }
    Post.set(One);
  }
  else if(postMultiply) {
    Pre.set(One);
    for(int j=0;j<A.n;j++) {
      Post(j) = 0;
      for(int i=0;i<A.m;i++) 
	Post(j) = Max(Post(j),Abs(A(i,j)));
      if(Post(j) == 0) Post(j) = 1;   //entire col is zero
    }
  }
  else {
    //cout<<"RobustSVD: Warning, neither pre nor postmultiply are set"<<endl;
    Pre.set(One);
    Post.set(One);
  }
  //cout<<"Premultiply: "; OutputASCIIShade(cout,Pre); cout<<endl;
  //cout<<"Postmultiply: "; OutputASCIIShade(cout,Post); cout<<endl;
}

template class SVDecomposition<float>;
template class SVDecomposition<double>;
template class RobustSVD<float>;
template class RobustSVD<double>;

} //namespace Math
