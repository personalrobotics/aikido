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
#include "primitives.h"
#include <stdio.h>

namespace Math3D {

Vector2::Vector2()
{}

Vector2::Vector2(const Vector2& v)
:x(v.x), y(v.y)
{}

Vector2::Vector2(Real _x)
:x(_x), y(_x)
{}

Vector2::Vector2(Real _x, Real _y)
:x(_x), y(_y)
{}

Vector2::Vector2(const Real data[2])
:x(data[0]), y(data[1])
{}






Vector3::Vector3()
{}

Vector3::Vector3(const Vector3& v)
:x(v.x), y(v.y), z(v.z)
{}

Vector3::Vector3(Real _x)
:x(_x), y(_x), z(_x)
{}

Vector3::Vector3(Real _x, Real _y, Real _z)
:x(_x), y(_y), z(_z)
{}

Vector3::Vector3(const Real data[3])
:x(data[0]), y(data[1]), z(data[2])
{}





Vector4::Vector4()
{}

Vector4::Vector4(const Vector4& v)
:x(v.x), y(v.y), z(v.z), w(v.w)
{}

Vector4::Vector4(Real _x)
:x(_x), y(_x), z(_x), w(_x)
{}

Vector4::Vector4(Real _x, Real _y, Real _z, Real _w)
:x(_x), y(_y), z(_z), w(_w)
{}

Vector4::Vector4(const Real data[4])
:x(data[0]), y(data[1]), z(data[2]), w(data[3])
{}

Vector4::Vector4(const Vector3& v)
:x(v.x), y(v.y), z(v.z), w(One)
{}






Matrix2::Matrix2()
{}

Matrix2::Matrix2(const Matrix2& m)
{
	set(m);
}

Matrix2::Matrix2(Real x)
{
	set(x);
}

Matrix2::Matrix2(const Real m[2][2])
{
	set(m);
}

Matrix2::Matrix2(const Real m[4])
{
	set(m);
}

Matrix2::Matrix2(const Vector2& xb, const Vector2& yb)
{
	set(xb,yb);
}

bool Matrix2::operator == (const Matrix2& a) const
{
	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
			if(data[i][j] != a.data[i][j])
				return false;
	}
	return true;
}

bool Matrix2::operator != (const Matrix2& a) const
{
	for(int i=0; i<2; i++)
	{
		for(int j=0; j<2; j++)
			if(data[i][j] == a.data[i][j])
				return false;
	}
	return true;
}





Matrix3::Matrix3()
{}

Matrix3::Matrix3(const Matrix3& m)
{
	set(m);
}

Matrix3::Matrix3(Real x)
{
	set(x);
}

Matrix3::Matrix3(const Real m[3][3])
{
	set(m);
}

Matrix3::Matrix3(const Real m[9])
{
	set(m);
}

Matrix3::Matrix3(const Vector3& xb, const Vector3& yb, const Vector3& zb)
{
	set(xb,yb,zb);
}


bool Matrix3::operator == (const Matrix3& a) const
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
			if(data[i][j] != a.data[i][j])
				return false;
	}
	return true;
}

bool Matrix3::operator != (const Matrix3& a) const
{
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
			if(data[i][j] == a.data[i][j])
				return false;
	}
	return true;
}

void Matrix3::mul(const Matrix3& a, const Matrix3& b)
{
	int i,j,k;
	Real sum;
	Real dat[3][3];
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			sum = Zero;
			for(k=0; k<3; k++)
				sum += b.data[i][k]*a.data[k][j];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}

void Matrix3::mulTransposeA(const Matrix3& a, const Matrix3& b)
{
	int i,j,k;
	Real sum;
	Real dat[3][3];
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			sum = Zero;
			for(k=0; k<3; k++)
				sum += b.data[i][k]*a.data[j][k];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}

void Matrix3::mulTransposeB(const Matrix3& a, const Matrix3& b)
{
	int i,j,k;
	Real sum;
	Real dat[3][3];
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			sum = Zero;
			for(k=0; k<3; k++)
				sum += b.data[k][i]*a.data[k][j];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}




#define SWAP(a,b) { tmp = (a) ; (a) = (b) ; (b) = tmp; }

static void rowswap(Matrix3 &m, int i, int j) {
  register Real tmp;
  SWAP(m(i,0),m(j,0));
  SWAP(m(i,1),m(j,1));
  SWAP(m(i,2),m(j,2));
}

bool Matrix3::setInverse(const Matrix3& a)
{
  Matrix3 _a(a);
  Matrix3& _b = *this;
  _b.setIdentity();

  int i,j,i1;
  for(j=0;j<3;j++) {
    i1 = j;
    for(i=j+1;i<3;i++)
      if (Abs(_a(i,j)) > Abs(_a(i1,j)))
	i1 = i;
    rowswap(_a,i1,j);
    rowswap(_b,i1,j);
    if (FuzzyZero(_a(j,j))){
      fprintf(stderr, "Taking inverse of singular Matrix3\n");
      return false;
    }
    register Real div = Inv(_a(j,j));
    _b(j,0) *= div;
    _b(j,1) *= div;
    _b(j,2) *= div;
    _a(j,0) *= div;
    _a(j,1) *= div;
    _a(j,2) *= div;
    for(i=0;i<3;i++) {
      if (i != j) {
	register Real tmp = _a(i,j);
	_b(i,0) -= tmp*_b(j,0);
	_b(i,1) -= tmp*_b(j,1);
	_b(i,2) -= tmp*_b(j,2);
	_a(i,0) -= tmp*_a(j,0);
	_a(i,1) -= tmp*_a(j,1);
	_a(i,2) -= tmp*_a(j,2);
      }
    }
  }
  return true;
}


Real Matrix3::determinant() const
{
	Real a = data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1]);
	Real b = data[0][1] * (data[1][2] * data[2][0] - data[1][0] * data[2][2]);
	Real c = data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
	return a + b + c;
}

Real Matrix3::cofactor(int i, int j) const
{
	const static int rotation [] = { 0, 1, 2, 0, 1, 2};
	int i1,i2,j1,j2;
	i1 = rotation[(i+1)];
	i2 = rotation[(i+2)];
	j1 = rotation[(j+1)];
	j2 = rotation[(j+2)];
	return data[i1][j1]*data[i2][j2] - data[i1][j2]*data[i2][j1];
}


Matrix4::Matrix4()
{}

Matrix4::Matrix4(const Matrix4& m)
{
	set(m);
}

Matrix4::Matrix4(Real x)
{
	set(x);
}

Matrix4::Matrix4(const Real m[4][4])
{
	set(m);
}

Matrix4::Matrix4(const Real m[16])
{
	set(m);
}

Matrix4::Matrix4(const Vector3& xb, const Vector3& yb, const Vector3& zb, const Vector3& trans)
{
	set(xb,yb,zb,trans);
}

Matrix4::Matrix4(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w)
{
	set(x,y,z,w);
}

Matrix4::Matrix4(const Matrix3& m)
{
	set(m);
}

Matrix4::Matrix4(const Matrix3& m, const Vector3& trans)
{
	set(m,trans);
}

Matrix4::Matrix4(const Vector3& trans)
{
	setTranslate(trans);
}

Matrix4::operator Matrix3() const
{
	Matrix3 tmp;
	get(tmp);
	return tmp;
}

bool Matrix4::operator == (const Matrix4& a) const
{
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
			if(data[i][j] != a.data[i][j])
				return false;
	}
	return true;
}

bool Matrix4::operator != (const Matrix4& a) const
{
	for(int i=0; i<4; i++)
	{
		for(int j=0; j<4; j++)
			if(data[i][j] == a.data[i][j])
				return false;
	}
	return true;
}

void Matrix4::mul(const Matrix4& a, const Matrix4& b)
{
	int i,j,k;
	Real sum;
	Real dat[4][4];
	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			sum = Zero;
			for(k=0; k<4; k++)
				sum += b.data[i][k]*a.data[k][j];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}

void Matrix4::mulTransposeA(const Matrix4& a, const Matrix4& b)
{
	int i,j,k;
	Real sum;
	Real dat[4][4];
	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			sum = Zero;
			for(k=0; k<4; k++)
				sum += b.data[i][k]*a.data[j][k];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}

void Matrix4::mulTransposeB(const Matrix4& a, const Matrix4& b)
{
	int i,j,k;
	Real sum;
	Real dat[4][4];
	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			sum = Zero;
			for(k=0; k<4; k++)
				sum += b.data[k][i]*a.data[k][j];	//column major
			dat[i][j] = sum;
		}
	}
	set(dat);
}

#define MATSWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}



bool Matrix4::setInverse(const Matrix4& a)
{
	Real tmp[12]; /* temp array for pairs */
	Real src[16]; /* array of transpose source matrix */
	Real dst[16]; /* array of destination matrix */
	Real det; /* determinant */

	int i;

	/* transpose matrix */
	for (i = 0; i < 4; i++) {
		src[i] = a.data[i][0];
		src[i + 4] = a.data[i][1];
		src[i + 8] = a.data[i][2];
		src[i + 12] = a.data[i][3];
	}

	/* calculate pairs for first 8 elements (cofactors) */
	tmp[0] = src[10] * src[15];
	tmp[1] = src[11] * src[14];
	tmp[2] = src[9] * src[15];
	tmp[3] = src[11] * src[13];
	tmp[4] = src[9] * src[14];
	tmp[5] = src[10] * src[13];
	tmp[6] = src[8] * src[15];
	tmp[7] = src[11] * src[12];
	tmp[8] = src[8] * src[14];
	tmp[9] = src[10] * src[12];
	tmp[10] = src[8] * src[13];
	tmp[11] = src[9] * src[12];

	/* calculate first 8 elements (cofactors) */
	dst[0] = tmp[0]*src[5] + tmp[3]*src[6] + tmp[4]*src[7];
	dst[0] -= tmp[1]*src[5] + tmp[2]*src[6] + tmp[5]*src[7];
	dst[1] = tmp[1]*src[4] + tmp[6]*src[6] + tmp[9]*src[7];
	dst[1] -= tmp[0]*src[4] + tmp[7]*src[6] + tmp[8]*src[7];
	dst[2] = tmp[2]*src[4] + tmp[7]*src[5] + tmp[10]*src[7];
	dst[2] -= tmp[3]*src[4] + tmp[6]*src[5] + tmp[11]*src[7];
	dst[3] = tmp[5]*src[4] + tmp[8]*src[5] + tmp[11]*src[6];
	dst[3] -= tmp[4]*src[4] + tmp[9]*src[5] + tmp[10]*src[6];
	dst[4] = tmp[1]*src[1] + tmp[2]*src[2] + tmp[5]*src[3];
	dst[4] -= tmp[0]*src[1] + tmp[3]*src[2] + tmp[4]*src[3];
	dst[5] = tmp[0]*src[0] + tmp[7]*src[2] + tmp[8]*src[3];
	dst[5] -= tmp[1]*src[0] + tmp[6]*src[2] + tmp[9]*src[3];
	dst[6] = tmp[3]*src[0] + tmp[6]*src[1] + tmp[11]*src[3];
	dst[6] -= tmp[2]*src[0] + tmp[7]*src[1] + tmp[10]*src[3];
	dst[7] = tmp[4]*src[0] + tmp[9]*src[1] + tmp[10]*src[2];
	dst[7] -= tmp[5]*src[0] + tmp[8]*src[1] + tmp[11]*src[2];

	/* calculate pairs for second 8 elements (cofactors) */
	tmp[0] = src[2]*src[7];
	tmp[1] = src[3]*src[6];
	tmp[2] = src[1]*src[7];
	tmp[3] = src[3]*src[5];
	tmp[4] = src[1]*src[6];
	tmp[5] = src[2]*src[5];
	tmp[6] = src[0]*src[7];
	tmp[7] = src[3]*src[4];
	tmp[8] = src[0]*src[6];
	tmp[9] = src[2]*src[4];
	tmp[10] = src[0]*src[5];
	tmp[11] = src[1]*src[4];

	/* calculate second 8 elements (cofactors) */
	dst[8] = tmp[0]*src[13] + tmp[3]*src[14] + tmp[4]*src[15];
	dst[8] -= tmp[1]*src[13] + tmp[2]*src[14] + tmp[5]*src[15];
	dst[9] = tmp[1]*src[12] + tmp[6]*src[14] + tmp[9]*src[15];
	dst[9] -= tmp[0]*src[12] + tmp[7]*src[14] + tmp[8]*src[15];
	dst[10] = tmp[2]*src[12] + tmp[7]*src[13] + tmp[10]*src[15];
	dst[10]-= tmp[3]*src[12] + tmp[6]*src[13] + tmp[11]*src[15];
	dst[11] = tmp[5]*src[12] + tmp[8]*src[13] + tmp[11]*src[14];
	dst[11]-= tmp[4]*src[12] + tmp[9]*src[13] + tmp[10]*src[14];
	dst[12] = tmp[2]*src[10] + tmp[5]*src[11] + tmp[1]*src[9];
	dst[12]-= tmp[4]*src[11] + tmp[0]*src[9] + tmp[3]*src[10];
	dst[13] = tmp[8]*src[11] + tmp[0]*src[8] + tmp[7]*src[10];
	dst[13]-= tmp[6]*src[10] + tmp[9]*src[11] + tmp[1]*src[8];
	dst[14] = tmp[6]*src[9] + tmp[11]*src[11] + tmp[3]*src[8];
	dst[14]-= tmp[10]*src[11] + tmp[2]*src[8] + tmp[7]*src[9];
	dst[15] = tmp[10]*src[10] + tmp[4]*src[8] + tmp[9]*src[9];
	dst[15]-= tmp[8]*src[9] + tmp[11]*src[10] + tmp[5]*src[8];

	/* calculate determinant */
	det=src[0]*dst[0]+src[1]*dst[1]+src[2]*dst[2]+src[3]*dst[3];

	/* calculate matrix inverse */
	if(det == Zero) return false;
	det = Inv(det);
	for (i = 0; i < 16; i++)
		dst[i] *= det;

	for (i = 0; i < 4; i++) {
		data[i][0] = dst[i*4];
		data[i][1] = dst[i*4+1];
		data[i][2] = dst[i*4+2];
		data[i][3] = dst[i*4+3];
	}

	return true;

	/*
  int i, j, k, l, ll;
  int icol=0, irow=0;
  int indxc[4], indxr[4], ipiv[4];
  Real big, dum, pivinv;

  set(a);

   // Gauss-jordan elimination with full pivoting.  Yes, folks, a 
   // GL Matrix4 is inverted like any other, since the identity is 
   // still the identity.
   
   // from numerical recipies in C second edition, pg 39
 
  for(j=0;j<=3;j++) ipiv[j] = 0;
  for(i=0;i<=3;i++) {
    big=0.0;
    for (j=0;j<=3;j++) {
      if(ipiv[j] != 1) {
	for (k=0;k<=3;k++) {
	  if(ipiv[k] == 0) {
	    if(fabs(data[j][k]) >= big) {
	      big = (Real) fabs(data[j][k]);
               irow=j;
               icol=k;
             }
           } else if (ipiv[k] > 1) {
	     printf("Error in gauss-jordan inversion\n");
	     return false;
	   }
         } 
       }
    }
     ++(ipiv[icol]);
     if (irow != icol) {
       for (l=0;l<=3;l++) swap(data[irow][l],data[icol][l]);
     }
     indxr[i]=irow;
     indxc[i]=icol;
     if(data[icol][icol] == Zero) {
       printf("Error in gauss-jordan inversion\n");
       return false;
     }
     pivinv = One / data[icol][icol];
     data[icol][icol]=One;
     for (l=0;l<=3;l++) data[icol][l] *= pivinv;
     for (ll=0;ll<=3;ll++) {
       if (ll != icol) {
         dum=data[ll][icol];
         data[ll][icol]=Zero;
         for (l=0;l<=3;l++) data[ll][l] -= data[icol][l]*dum;
       }
     }
   }
   for (l=3;l>=0;l--) {
     if (indxr[l] != indxc[l]) {
       for (k=0;k<=3;k++) {
         swap(data[k][indxr[l]],data[k][indxc[l]]);
       }
     }
   }
   return true;
   */

/*
	Real det = m.determinant();
	if(det == Zero)
		return false;
	Real detinv = One/det;

	int i,j;
	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			data[i][j] = m.cofactor(j,i) * detinv;
		}
	}
	return true;*/
}


Real Matrix4::determinant() const
{
	Real a = data[0][0] * cofactor(0,0);
	Real b = data[0][1] * cofactor(0,1);
	Real c = data[0][2] * cofactor(0,2);
	Real d = data[0][3] * cofactor(0,3);
	return a - b + c - d;
}

Real Matrix4::cofactor(int i, int j) const
{
	static int rotation [] = { 0, 1, 2, 3, 0, 1, 2};
	int i1,i2,i3,j1,j2,j3;
	i1 = rotation[(i+1)];
	i2 = rotation[(i+2)];
	i3 = rotation[(i+3)];
	j1 = rotation[(j+1)];
	j2 = rotation[(j+2)];
	j3 = rotation[(j+3)];
	Real a = data[i1][j1] * (data[i2][j2] * data[i3][j3] - data[i2][j3] * data[i3][j2]);
	Real b = data[i1][j2] * (data[i2][j3] * data[i3][j1] - data[i2][j1] * data[i3][j3]);
	Real c = data[i1][j3] * (data[i2][j1] * data[i3][j2] - data[i2][j2] * data[i3][j1]);
	return a - b + c;
}




RigidTransform::RigidTransform()
{}

RigidTransform::RigidTransform(const RigidTransform& x)
:R(x.R), t(x.t)
{}

RigidTransform::RigidTransform(const Matrix3& _R, const Vector3& _t)
:R(_R), t(_t)
{}

RigidTransform::RigidTransform(const Vector3& x, const Vector3& y, const Vector3& z, const Vector3& _t)
:R(x,y,z), t(_t)
{}

RigidTransform::RigidTransform(const Matrix4& m)
{
	m.get(R,t);
}


bool RigidTransform::isValid(Real eps) const
{
	return FuzzyEquals(R.determinant(),One,eps);
}


RigidTransform2D::RigidTransform2D()
{
}

RigidTransform2D::RigidTransform2D(const RigidTransform2D& rhs)
:R(rhs.R),t(rhs.t)
{
}

RigidTransform2D::RigidTransform2D(const Matrix3& m)
{
	set(m);
}

RigidTransform2D::RigidTransform2D(const Matrix2& _R,const Vector2& _t)
:R(_R),t(_t)
{
}

RigidTransform2D::RigidTransform2D(Real theta,const Vector2& t)
{
	set(theta,t);
}


bool RigidTransform2D::isValid(Real eps) const
{
	return FuzzyEquals(R.determinant(),One,eps);
}




std::ostream& operator << (std::ostream& out, const Vector2& v)
{
	out << v.x << " " << v.y;
	return out;
}

std::istream& operator >> (std::istream& in, Vector2& v)
{
	in >> v.x >> v.y;
	return in;
}

std::ostream& operator << (std::ostream& out, const Vector3& v)
{
	out << v.x << " " << v.y << " " << v.z;
	return out;
}

std::istream& operator >> (std::istream& in, Vector3& v)
{
	in >> v.x >> v.y >> v.z;
	return in;
}

std::ostream& operator << (std::ostream& out, const Vector4& v)
{
	out << v.x << " " << v.y << " " << v.z << " " << v.w;
	return out;
}

std::istream& operator >> (std::istream& in, Vector4& v)
{
	in >> v.x >> v.y >> v.z >> v.w;
	return in;
}

std::ostream& operator << (std::ostream& out, const Matrix2& m)
{
	out << m(0,0) << " " << m(0,1) << std::endl;
	out << m(1,0) << " " << m(1,1);
	return out;
}

std::istream& operator >> (std::istream& in, Matrix2& m)
{
	in >> m(0,0) >> m(0,1);
	in >> m(1,0) >> m(1,1);
	return in;
}

std::ostream& operator << (std::ostream& out, const Matrix3& m)
{
	out << m(0,0) << " " << m(0,1) << " " << m(0,2) << std::endl;
	out << m(1,0) << " " << m(1,1) << " " << m(1,2) << std::endl;
	out << m(2,0) << " " << m(2,1) << " " << m(2,2);
	return out;
}

std::istream& operator >> (std::istream& in, Matrix3& m)
{
	in >> m(0,0) >> m(0,1) >> m(0,2);
	in >> m(1,0) >> m(1,1) >> m(1,2);
	in >> m(2,0) >> m(2,1) >> m(2,2);
	return in;
}

std::ostream& operator << (std::ostream& out, const Matrix4& m)
{
	out << m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(0,3) << std::endl;
	out << m(1,0) << " " << m(1,1) << " " << m(1,2) << " " << m(1,3) << std::endl;
	out << m(2,0) << " " << m(2,1) << " " << m(2,2) << " " << m(2,3) << std::endl;
	out << m(3,0) << " " << m(3,1) << " " << m(3,2) << " " << m(3,3);
	return out;
}

std::istream& operator >> (std::istream& in, Matrix4& m)
{
	in >> m(0,0) >> m(0,1) >> m(0,2) >> m(0,3);
	in >> m(1,0) >> m(1,1) >> m(1,2) >> m(1,3);
	in >> m(2,0) >> m(2,1) >> m(2,2) >> m(2,3);
	in >> m(3,0) >> m(3,1) >> m(3,2) >> m(3,3);
	return in;
}

std::ostream& operator << (std::ostream& out, const RigidTransform& x)
{
	out << x.R << std::endl;
	out << x.t;
	return out;
}

std::istream& operator >> (std::istream& in, RigidTransform& x)
{
	in >> x.R;
	in >> x.t;
	return in;
}

std::ostream& operator << (std::ostream& out, const RigidTransform2D& x)
{
	out << x.R << std::endl;
	out << x.t;
	return out;
}

std::istream& operator >> (std::istream& in, RigidTransform2D& x)
{
	in >> x.R;
	in >> x.t;
	return in;
}



} //namespace Math3D
