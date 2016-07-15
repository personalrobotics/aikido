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
#ifndef MATH_FAST_ARRAY_H
#define MATH_FAST_ARRAY_H

#include <mintos/misc/utils.h>

namespace Math {

template <class T>
inline T* array_create(int n)
{
  if(n != 0)
    return (T*)malloc(sizeof(T)*n);
  return NULL;
}

template <class T>
inline void array_delete(T* v)
{
  if(v)
    free(v);
}

template <class T>
inline void array_equal(T* a, const T* b, int n)
{
  for(int i=0;i<n;i++,a++,b++)
    *a = *b;
}

template <class T>
inline T* array_copy(T* a, int n)
{
  T* x = array_create<T>(n);
  array_equal(x,a,n);
  return x;
}

template <class T>
inline void array_fill(T* a, const T& c, int n)
{
  for(int i=0;i<n;i++,a++)
    *a = c;
}

template <class T>
inline void array_zero(T* a, int n)
{
  array_fill(a,(T)0,n);
}

template <class T>
inline void array_negate(T* x, const T* a, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x = -*a;
}

template<class T>
inline void array_add(T* x, const T* a, const T* b, int n)
{
  for(int i=0;i<n;i++,x++,a++,b++)
    *x = *a+*b;
}

template<class T>
inline void array_sub(T* x, const T* a, const T* b, int n)
{
  for(int i=0;i<n;i++,x++,a++,b++)
    *x = *a-*b;
}

template<class T>
inline void array_mul(T* x, const T* a, T s, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x = *a*s;
}

template<class T>
inline void array_div(T* x, const T* a, T s, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x = *a/s;
}

template<class T>
inline void array_acc(T* x, const T* a, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x += *a;
}

template<class T>
inline void array_dec(T* x, const T* a, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x -= *a;
}

template<class T>
inline void array_madd(T* x, const T* a, const T& s, int n)
{
  for(int i=0;i<n;i++,x++,a++)
    *x += *a*s;
}

template<class T>
inline void array_axpby(T* z, T a, const T* x, T b, const T* y, int n)
{
  for(int i=0;i<n;i++,z++,x++,y++)
    *z = a*(*x) + b*(*y);
}

template<class T>
inline void array_mul(T* a, const T& s, int n)
{
  for(int i=0;i<n;i++,a++)
    *a *= s;
}

template<class T>
inline void array_div(T* a, const T& s, int n)
{
  for(int i=0;i<n;i++,a++)
    *a /= s;
}

template<class T>
inline T array_sum_product(const T* a, const T* b, int n)
{
  T sum = 0;
  for(int i=0;i<n;i++,a++,b++)
    sum += (*a)*(*b);
  return sum;
}

template<class T>
inline T array_dot(const T* a, const T* b, int n)
{
  T sum = 0;
  for(int i=0;i<n;i++,a++,b++)
    sum += dot(*a,*b);
  return sum;
}

template<class T>
inline T array_norm_squared(const T* a, int n)
{
  T sum = 0;
  for(int i=0;i<n;i++,a++)
    sum += dot(*a,*a);
  return sum;
}

//generalized arrays:
//x(i) = x0[i*stride]
template<class T>
inline void gen_array_fill(T* x, int xstride, T c, int n)
{
  for(int i=0;i<n;i++,x+=xstride)
    *x = c;
}

template<class T>
inline void gen_array_mul(T* x, int xstride, T c, int n)
{
  for(int i=0;i<n;i++,x+=xstride)
    *x *= c;
}

template<class T>
inline void gen_array_div(T* x, int xstride, T c, int n)
{
  for(int i=0;i<n;i++,x+=xstride)
    *x /= c;
}

template<class T>
inline void gen_array_negate(T* x, int xstride, const T* a, int astride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x = -*a;
}

template<class T>
inline void gen_array_equal(T* x, int xstride, const T* a, int astride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x = *a;
}

template<class T>
inline void gen_array_swap(T* x, int xstride, T* y, int ystride, int n)
{
  T tmp;
  for(int i=0;i<n;i++,x+=xstride,y+=ystride) {
    tmp = *x;
    *x = *y;
    *y = tmp;
  }
}

template<class T>
inline void gen_array_mul(T* x, int xstride, const T* a, int astride, T c, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x = *a*c;
}

template<class T>
inline void gen_array_div(T* x, int xstride, const T* a, int astride, T c, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x = *a/c;
}

template<class T>
inline void gen_array_add(T* x, int xstride, const T* a, int astride, const T* b, int bstride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride,b+=bstride)
    *x = *a+*b;
}

template<class T>
inline void gen_array_sub(T* x, int xstride, const T* a, int astride, const T* b, int bstride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride,b+=bstride)
    *x = *a-*b;
}

template<class T>
inline void gen_array_acc(T* x, int xstride, const T* a, int astride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x += *a;
}

template<class T>
inline void gen_array_dec(T* x, int xstride, const T* a, int astride, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x -= *a;
}

template<class T>
inline void gen_array_madd(T* x, int xstride, const T* a, int astride, T s, int n)
{
  for(int i=0;i<n;i++,x+=xstride,a+=astride)
    *x += *a*s;
}

template<class T>
inline void gen_array_axpby(T* z, int zstride, T a, const T* x, int xstride, T b, const T* y, int ystride, int n)
{
  for(int i=0;i<n;i++,z+=zstride,x+=xstride,y+=ystride)
    *z = a*(*x)+b*(*y);
}

template<class T>
inline T gen_array_sum_product(const T* a, int astride, const T* b, int bstride, int n)
{
  T sum=0;
  for(int i=0;i<n;i++,a+=astride,b+=bstride)
    sum += (*a)*(*b);
  return sum;
}

template<class T>
inline T gen_array_dot(const T* a, int astride, const T* b, int bstride, int n)
{
  T sum=0;
  for(int i=0;i<n;i++,a+=astride,b+=bstride)
    sum += dot(*a,*b);
  return sum;
}

template<class T>
inline T gen_array_norm_squared(const T* a, int astride, int n)
{
  T sum = 0;
  for(int i=0;i<n;i++,a+=astride)
    sum += (*a)*(*a);
  return sum;
}



// 2d arrays:
// Assumed to have dimensions m x n
// The data layout is [m pointers][big block of m*n data]
template <class T>
inline T** array2d_create(int m, int n)
{
  T** array;
  int i;
  if(n != 0 && m != 0)
  {
    int ofs = m*sizeof(T*);
    array = (T**)malloc(m*sizeof(T*) + m*n*sizeof(T));
    unsigned char* bytes = (unsigned char*) array;
    for(i=0; i<m; i++)
    {
      //array+m = offset to block
      //i*n = offset in block
      array[i] = (T*)((bytes + ofs) + i*n*(sizeof(T)));
    }
    return array;
  }
  return NULL;
}

template <class T>
inline void array2d_delete(T** v)
{
  if(v)
    free(v);
}


//X = 0
template <class T>
inline void array2d_fill(T** X, const T& c, int m, int n)
{
  array_fill(X[0], c, m*n);
}

//X = I
template <class T>
inline void array2d_identity(T** X, int m, int n)
{
  array_zero(X[0], m*n);
  gen_array_fill(X[0],n+1,One,::Min(m,n));
}


//X = A
template <class T>
inline void array2d_equal(T** X, T** const A, int m, int n)
{
  array_equal(X[0],A[0],m*n);
}

//X = A
template <class T>
inline T** array2d_copy(T** A, int m, int n)
{
  T** X = array2d_create<T>(m,n);
  array2d_equal(X,A,m,n);
  return X;
}

//X = A^t.  X is mxn, A is nxm
template <class T>
inline void array2d_transpose(T** X, T** const A, int m, int n)
{
  int i;
  for(i=0; i<m; i++)
    gen_array_equal(X[i],1,&A[0][i],m,n);
  /*
    for(j=0; j<n; j++)
      X[i][j] = A[j][i];
      */
}

//X = A+B
template <class T>
inline void array2d_add(T** x, T** const a, T** const b, int m, int n)
{
  array_add(x[0],a[0],b[0],m*n);
}

//X = A-B
template <class T>
inline void array2d_sub(T** x, T** const a, T** const b, int m, int n)
{
  array_sub(x[0],a[0],b[0],m*n);
}

//X = A*B.  X is mxp, A is mxn, B is nxp
template <class T>
inline void array2d_mul(T** x, T** const a, T** const b, int m, int n, int p)
{
  int i,j;//,k;
  for(i=0; i<m; i++)
  {
    for(j=0; j<p; j++)
    {
      x[i][j] = gen_array_sum_product(a[i],1,&b[0][j],p,n);
      /*
      x[i][j] = 0;
      for(k=0; k<n; k++)
        x[i][j] += a[i][k]*b[k][j];
        */
    }
  }
}

//X = A^t*B.  X is mxp, A is nxm, B is nxp
template <class T>
inline void array2d_multiply_transposeA(T** x, T** const a, T** const b, int m, int n, int p)
{
  int i,j;//,k;
  for(i=0; i<m; i++)
  {
    for(j=0; j<p; j++)
    {
      x[i][j] = gen_array_sum_product(&a[0][i],m,&b[0][j],p,n);
      /*
      x[i][j] = 0;
      for(k=0; k<n; k++)
        x[i][j] += a[k][i]*b[k][j];
      */
    }
  }
}

//X = A*B^t.  X is mxp, A is mxn, B is pxn
template <class T>
inline void array2d_multiply_transposeB(T** x, T** const a, T** const b, int m, int n, int p)
{
  int i,j;
  for(i=0; i<m; i++)
  {
    for(j=0; j<p; j++)
    {
      /*
      x[i][j] = 0;
      for(int k=0; k<n; k++)
        x[i][j] += a[i][k]*b[j][k];
      */
      x[i][j] = array_sum_product(a[i],b[j],n);
    }
  }
}

//x = A*b
template <class T>
inline void array2d_vector_multiply(T* x, T** const a, const T* b, int m, int n)
{
  int i;
  for(i=0; i<m; i++)
    x[i] = array_sum_product(a[i],b,n);
}

//x = A^t*b
template <class T>
inline void array2d_vector_multiply_transpose(T* x, T** const a, const T* b, int m, int n)
{
  /*
  int i,j;
  for(i=0; i<n; i++)
    x[i] = 0;
  for(i=0; i<m; i++)
    for(j=0;j<n;j++)
      x[j] += a[i][j]*b[i];
      */
  for(int i=0;i<n;i++)
    x[i] = gen_array_sum_product(&a[0][i],n,b,1,m);
}

//x += A*b
template <class T>
inline void array2d_vector_madd(T* x, T** const a, const T* b, int m, int n)
{
  int i;
  for(i=0; i<m; i++)
    x[i] += array_sum_product(a[i],b,n);
}

//x += A^t*b
template <class T>
inline void array2d_vector_madd_transpose(T* x, T** const a, const T* b, int m, int n)
{
  /*
  int i,j;
  for(i=0; i<m; i++)
    for(j=0;j<n;j++)
      x[j] += a[i][j]*b[i];
      */
  for(int i=0;i<n;i++)
    x[i] += gen_array_sum_product(&a[0][i],n,b,1,m);
}

//returns the dot product <(row i of a),v>
template <class T>
inline T array2d_row_dot(T** const a, int i, const T* v, int m,int n)
{
  return array_dot(a[i], v, n);
}

//returns the dot product <(col i of a),v>
template <class T>
inline T array2d_col_dot(T** const a, int i, const T* v, int m,int n)
{
  gen_array_dot(&a[0][i],n,v,1,m);
}



template <class T>
inline T* gen_array2d_entry(T* X,int xis,int xjs,int i,int j)
{
  return X+xis*i+xjs*j;
}

//returns ptr to row i (stride is xjs)
template <class T>
inline T* gen_array2d_row(T* X,int xis,int xjs,int i)
{
  return X+xis*i;
}

//returns ptr to col j (stride is xis)
template <class T>
inline T* gen_array2d_col(T* X,int xis,int xjs,int j)
{
  return X+xjs*j;
}

//X = c
template <class T>
inline void gen_array2d_fill(T* X,int xis,int xjs,const T& c,int m, int n)
{
  for(int i=0;i<m;i++,X+=xis)
    gen_array_fill(X,xjs,c,n);
}

//X = I
template <class T>
inline void gen_array2d_identity(T* X,int xis,int xjs, int m, int n)
{
  gen_array2d_fill(X,xis,xjs,T(0),m,n);
  gen_array_fill(X,xis+xjs,T(1),m);
}


//X = A
template <class T>
inline void gen_array2d_equal(T* X,int xis,int xjs,
                              const T* A,int ais,int ajs, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais)
    gen_array_equal(X,xjs,A,ajs,n);
}

//X = A^t.  X is mxn, A is nxm
template <class T>
inline void gen_array2d_transpose(T* X,int xis,int xjs,
                                  const T* A,int ais,int ajs, int m, int n)
{
  gen_array2d_equal(X,xis,xjs,A,ajs,ais,m,n);
}

//X = A+B
template <class T>
inline void gen_array2d_add(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            const T* B,int bis,int bjs, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais,B+=bis) {
    gen_array_add(X,xjs,A,ajs,B,bjs,n);
  }
}

//X = A-B
template <class T>
inline void gen_array2d_sub(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            const T* B,int bis,int bjs, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais,B+=bis) {
    gen_array_sub(X,xjs,A,ajs,B,bjs,n);
  }
}

//X = cA
template <class T>
inline void gen_array2d_mul(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            const T& c, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais)
    gen_array_mul(X,xjs,A,ajs,c,n);
}

//X *= c
template <class T>
inline void gen_array2d_mul(T* X,int xis,int xjs,
                            const T& c, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis)
    gen_array_mul(X,xjs,c,n);
}

//X /= c
template <class T>
inline void gen_array2d_div(T* X,int xis,int xjs,
                            const T& c, int m, int n)
{
  for(int i=0;i<m;i++,X+=xis)
    gen_array_div(X,xjs,c,n);
}


//X += A
template <class T>
inline void gen_array2d_acc(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais) {
    gen_array_acc(X,xjs,A,ajs,n);
  }
}

//X += A
template <class T>
inline void gen_array2d_dec(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais) {
    gen_array_dec(X,xjs,A,ajs,n);
  }
}

//X += cA
template <class T>
inline void gen_array2d_madd(T* X,int xis,int xjs,
                            const T* A,int ais,int ajs,
                            const T& c,
                            int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais) {
    gen_array_madd(X,xjs,A,ajs,c,n);
  }
}

//X = -A
template <class T>
inline void gen_array2d_negate(T* X,int xis,int xjs,
                              const T* A,int ais,int ajs,
                              int m, int n)
{
  for(int i=0;i<m;i++,X+=xis,A+=ais) {
    gen_array_negate(X,xjs,A,ajs,n);
  }
}

//X = A*B.  X is mxp, A is mxn, B is nxp
template <class T>
inline void gen_array2d_multiply(T* X,int xis,int xjs,
                                 const T* A,int ais,int ajs,
                                 const T* B,int bis,int bjs,
                                 int m,int n,int p)
{
  int i,j,k;
  T sum;
  for(i=0;i<m;i++, X+=xis,A+=ais)
  {
    T* Xi=X;
    const T* Bj=B;
    for(j=0;j<p;j++, Xi+=xjs,Bj+=bjs)
    {
      const T* Aij=A;
      const T* Bij=Bj;
      sum=0;
      for(k=0;k<n;k++, Aij+=ajs,Bij+=bis)
        sum += (*Aij)*(*Bij);
        //sum += A[i*ais+k*ajs]*B[k*bis+j*bjs];
      *Xi = sum;
      //X[i*xis+j*xjs] = sum;
    }
  }
}

//X = A^t*B.  X is mxp, A is nxm, B is nxp
template <class T>
inline void gen_array2d_multiply_transposeA(T* X,int xis,int xjs,
                                            const T* A,int ais,int ajs,
                                            const T* B,int bis,int bjs,
                                            int m,int n,int p)
{
  gen_array2d_multiply(X,xis,xjs,
    A,ajs,ais,
    B,bis,bjs,
    m,n,p);
}

//X = A*B^t.  X is mxp, A is mxn, B is pxn
template <class T>
inline void gen_array2d_multiply_transposeB(T* X,int xis,int xjs,
                                            const T* A,int ais,int ajs,
                                            const T* B,int bis,int bjs,
                                            int m,int n,int p)
{
  gen_array2d_multiply(X,xis,xjs,
    A,ais,ajs,
    B,bjs,bis,
    m,n,p);
}

//x = A*b
template <class T>
inline void gen_array2d_vector_multiply(T* x,int xs,
                                        const T* A,int ais,int ajs,
                                        const T* b,int bs,
                                        int m, int n)
{
  for(int i=0;i<m;i++, x+=xs,A+=ais)
    *x = gen_array_sum_product(A,ajs, b,bs, n);
}

//x = A^t*b
template <class T>
inline void gen_array2d_vector_multiply_transpose(T* x,int xs,
                                        const T* A,int ais,int ajs,
                                        const T* b,int bs,
                                        int m, int n)
{
  gen_array2d_vector_multiply(x,xs,A,ajs,ais,b,bs,n,m);
}

//x += A*b
template <class T>
inline void gen_array2d_vector_madd(T* x,int xs,
                                    const T* A,int ais,int ajs,
                                    const T* b,int bs,
                                    int m, int n)
{
  for(int i=0;i<m;i++, x+=xs,A+=ais)
    *x += gen_array_sum_product(A,ajs, b,bs, n);
}

//x += A^t*b
template <class T>
inline void gen_array2d_vector_madd_transpose(T* x,int xs,
                                    const T* A,int ais,int ajs,
                                    const T* b,int bs,
                                    int m, int n)
{
  gen_array2d_vector_madd(x,xs,A,ajs,ais,b,bs,n,m);
}


} //namespace Math

#endif
