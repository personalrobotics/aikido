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
#ifndef MATH_COMPLEX_H
#define MATH_COMPLEX_H

#include <mintos/math/math.h>
#include <mintos/misc/errors.h>
#include <iostream>

namespace Math {

class Complex;
class Quaternion;

/** @ingroup Math
 * @brief Complex number class (x + iy).
 */
class Complex 
{
 public:
  Complex();
  Complex(const Complex&);
  Complex(Real x);
  Complex(Real x, Real y);

  inline bool operator == (const Complex&) const;
  inline bool operator != (const Complex&) const;
  inline const Complex& operator =  (const Complex&);
  inline void operator += (const Complex&);
  inline void operator -= (const Complex&);
  inline void operator *= (const Complex&);
  inline void operator /= (const Complex&);
  inline const Complex& operator =  (Real);
  inline void operator += (Real);
  inline void operator -= (Real);
  inline void operator *= (Real);
  inline void operator /= (Real);

  inline void add(const Complex& a, const Complex& b);
  inline void sub(const Complex& a, const Complex& b);
  inline void mul(const Complex& a, const Complex& b);
  inline void div(const Complex& a, const Complex& b);
  inline void madd(const Complex& a, const Complex& b);
  inline void addConjugateB(const Complex& a, const Complex& b);
  inline void subConjugateB(const Complex& a, const Complex& b);
  inline void mulConjugateB(const Complex& a, const Complex& b);
  inline void divConjugateB(const Complex& a, const Complex& b);
  inline void maddConjugateB(const Complex& a, const Complex& b);
  inline void add(const Complex& a, Real);
  inline void sub(const Complex& a, Real);
  inline void mul(const Complex& a, Real);
  inline void div(const Complex& a, Real);
  inline void madd(const Complex& a, Real);

  inline void set(const Complex&);
  inline void set(Real x);
  inline void set(Real x, Real y);
  inline void setZero();
  inline void setNegative(const Complex&);
  inline void setNormalized(const Complex&);
  inline void setConjugate(const Complex&);
  inline void setPolar(Real r, Real theta);
  inline bool setInverse(const Complex&);
  void setExp (const Complex&);
  bool setLog (const Complex&, int n = 0);
  void setPow (const Complex&, Real);
  void setPow (const Complex&, const Complex&);

  inline void get(Complex&) const;
  inline void get(Real& x, Real& y) const;
  inline void getNegative(Complex&) const;
  inline void getNormalized(Complex&) const;
  inline void getConjugate(Complex&) const;
  inline void getPolar(Real& r, Real& theta);
  inline bool getInverse(Complex&) const;

  inline void inplaceNegative();
  inline void inplaceScale(Real);
  inline void inplaceNormalize();
  inline void inplaceConjugate();
  inline bool inplaceInverse();

  inline bool isInvertible() const;
  inline Real norm() const;
  inline Real normSquared() const;
  inline Real arg() const;

  inline const Real& Re() const;
  inline const Real& Im() const;
  inline Real& Re();
  inline Real& Im();

  Real x, y;
};

inline Real dot(const Complex& a, const Complex& b);
inline Complex operator - (const Complex& a);
inline Complex operator + (const Complex& a, const Complex& b);
inline Complex operator - (const Complex& a, const Complex& b);
inline Complex operator * (const Complex& a, const Complex& b);
inline Complex operator / (const Complex& a, const Complex& b);
inline Complex operator + (const Complex& a, Real b);
inline Complex operator - (const Complex& a, Real b);
inline Complex operator * (const Complex& a, Real b);
inline Complex operator / (const Complex& a, Real b);
inline Real Abs(const Complex& x);
inline Complex Sqr(const Complex& x);
inline Complex Sqrt(const Complex& x);
inline Complex Exp(const Complex& x);
inline Complex Log(const Complex& x);
inline Complex Pow(const Complex& x, Real y);
inline Complex Pow(const Complex& x, const Complex& y);
inline Complex Sin(const Complex& z);
inline Complex Cos(const Complex& z);
inline Complex Tan(const Complex& z);
inline Complex Sinh(const Complex& z);
inline Complex Cosh(const Complex& z);
inline Complex Tanh(const Complex& z);
inline Complex Inv(const Complex& x);
inline bool FuzzyEquals(const Complex& a, const Complex& b, Real eps=Epsilon);
inline bool FuzzyZero(const Complex& a, Real eps=Epsilon);
inline Complex PseudoInv(const Complex& x,Real eps=Epsilon);
inline bool FuzzyEquals(const Complex& a, const Complex& b, const Complex& eps);
inline bool FuzzyZero(const Complex& a, const Complex& eps);
inline Complex PseudoInv(const Complex& x,const Complex& eps);


/** @ingroup Math
 * @brief Complex quaternion class (w + ix + jy + kz).
 */
class Quaternion
{
 public:
  Quaternion();
  Quaternion(const Quaternion&);
  Quaternion(Real w);
  Quaternion(Real w, const Real* im);
  Quaternion(Real w, Real x, Real y, Real z);

  inline bool operator == (const Quaternion&) const;
  inline bool operator != (const Quaternion&) const;
  inline const Quaternion& operator =  (const Quaternion&);
  inline void operator += (const Quaternion&);
  inline void operator -= (const Quaternion&);
  inline void operator *= (const Quaternion&);
  inline void operator /= (const Quaternion&);
  inline const Quaternion& operator =  (Real);
  inline void operator += (Real);
  inline void operator -= (Real);
  inline void operator *= (Real);
  inline void operator /= (Real);

  inline void add(const Quaternion& a, const Quaternion& b);
  inline void sub(const Quaternion& a, const Quaternion& b);
  void mul(const Quaternion& a, const Quaternion& b);
  void div(const Quaternion& a, const Quaternion& b);
  void madd(const Quaternion& a, const Quaternion& b);
  inline void mul(const Quaternion& a, Real b);
  inline void div(const Quaternion& a, Real b);
  inline void madd(const Quaternion& a, Real b);

  inline void set(const Quaternion&);
  inline void set(Real w);
  inline void set(Real w, const Real* im);
  inline void set(Real w, Real x, Real y, Real z);
  inline void setZero();
  inline void setNegative(const Quaternion&);
  inline void setNormalized(const Quaternion&);
  inline void setConjugate (const Quaternion&);
  inline bool setInverse (const Quaternion&);
  void setExp (const Quaternion&);
  bool setLog (const Quaternion&, int n = 0);
  void setPow (const Quaternion&, Real);

  inline void get(Quaternion&) const;
  inline void get(Real& w, Real* im) const;
  inline void get(Real& w, Real& x, Real& y, Real& z) const;
  inline void getNegative(Quaternion&) const;
  inline void getNormalized(Quaternion&) const;
  inline void getConjugate (Quaternion&) const;
  inline bool getInverse (Quaternion&) const;

  inline void inplaceNegative();
  inline void inplaceScale(Real);
  inline void inplaceNormalize();
  inline void inplaceConjugate();
  inline bool inplaceInverse();

  inline bool isInvertible() const;
  inline Real norm() const;
  inline Real normSquared() const;
  inline Real imNorm() const;		//norm of imaginary part

  union {
    Real data [4];
    struct { Real w, x, y, z; };
  };
};

inline Real dot(const Quaternion& a, const Quaternion& b);
inline Quaternion operator - (const Quaternion& a);
inline Quaternion operator + (const Quaternion& a, const Quaternion& b);
inline Quaternion operator - (const Quaternion& a, const Quaternion& b);
inline Quaternion operator * (const Quaternion& a, const Quaternion& b);
inline Quaternion operator / (const Quaternion& a, const Quaternion& b);
inline Quaternion operator + (const Quaternion& a, Real b);
inline Quaternion operator - (const Quaternion& a, Real b);
inline Quaternion operator * (const Quaternion& a, Real b);
inline Quaternion operator / (const Quaternion& a, Real b);


inline bool Complex::operator == (const Complex& a) const { return a.x == x && a.y == y; }
inline bool Complex::operator != (const Complex& a) const { return a.x != x || a.y != y; }
inline const Complex& Complex::operator =  (const Complex& z) { set(z); return *this; }
inline void Complex::operator += (const Complex& z) { x += z.x; y += z.y; }
inline void Complex::operator -= (const Complex& z) { x -= z.x; y -= z.y; }
inline void Complex::operator *= (const Complex& z) { Real tmp = x*z.x - y*z.y; y = x*z.y + y*z.x; x = tmp; }
inline void Complex::operator /= (const Complex& z) { div(*this, z); }
inline const Complex& Complex::operator =  (Real a) { x = a; y = Zero;	return *this; }
inline void Complex::operator += (Real a) { x += a; }
inline void Complex::operator -= (Real a) { x -= a; }
inline void Complex::operator *= (Real a) { x *= a; y *= a; }
inline void Complex::operator /= (Real a) { Real ainv = Inv(a); operator *= (ainv); }
inline void Complex::add(const Complex& a, const Complex& b) { x = a.x + b.x; y = a.y + b.y; }
inline void Complex::sub(const Complex& a, const Complex& b) { x = a.x - b.x; y = a.y - b.y; }
inline void Complex::mul(const Complex& a, const Complex& b) { Real tmp = a.x*b.x - a.y*b.y; y = a.x*b.y + a.y*b.x; x = tmp; }
inline void Complex::madd(const Complex& a, const Complex& b) { Real tmp = a.x*b.x - a.y*b.y; y += a.x*b.y + a.y*b.x; x += tmp; }
inline void Complex::div(const Complex& a, const Complex& b) { Real l = Inv(b.normSquared()); Real tmp = a.x*b.x + a.y*b.y; y = a.y*b.x - a.x*b.y; x = tmp; x*=l; y*=l; }
inline void Complex::addConjugateB(const Complex& a, const Complex& b) { x = a.x + b.x; y = a.y - b.y; }
inline void Complex::subConjugateB(const Complex& a, const Complex& b) { x = a.x - b.x; y = a.y + b.y; }
inline void Complex::mulConjugateB(const Complex& a, const Complex& b) { Real tmp = a.x*b.x + a.y*b.y; y = -a.x*b.y + a.y*b.x; x = tmp; }
inline void Complex::maddConjugateB(const Complex& a, const Complex& b) { Real tmp = a.x*b.x + a.y*b.y; y += -a.x*b.y + a.y*b.x; x += tmp; }
inline void Complex::divConjugateB(const Complex& a, const Complex& b) { Real l = Inv(b.normSquared()); Real tmp = a.x*b.x - a.y*b.y; y = a.y*b.x + a.x*b.y; x = tmp; x*=l; y*=l; }
inline void Complex::add(const Complex& a, Real b) { x = a.x + b; }
inline void Complex::sub(const Complex& a, Real b) { x = a.x - b; }
inline void Complex::mul(const Complex& a, Real b) { x = a.x*b; y = a.y*b; }
inline void Complex::div(const Complex& a, Real b) { Real binv = Inv(b); mul(a,binv); }
inline void Complex::madd(const Complex& a, Real b) { x += a.x*b; y += a.y*b; }
inline void Complex::set(const Complex& z) { x = z.x; y = z.y; }
inline void Complex::set(Real _x) { x = _x; y = Zero; }
inline void Complex::set(Real _x, Real _y) { x = _x; y = _y; }
inline void Complex::setZero() { x = Zero; y = Zero; }
inline void Complex::setNegative(const Complex& z) { x = -z.x; y = -z.y; }
inline void Complex::setNormalized(const Complex& z) { div(z,z.norm()); }
inline void Complex::setConjugate(const Complex& z) { x = z.x; y = -z.y; }
inline void Complex::setPolar(Real r, Real theta) { x=Cos(theta)*r; y=Sin(theta)*r; }
inline bool Complex::setInverse(const Complex& z) { Real n2 = z.normSquared(); if(n2==Zero) return false; div(z,n2); inplaceConjugate(); return true;}
inline void Complex::get(Complex& z) const { z.x = x; z.y = y; }
inline void Complex::get(Real& _x, Real& _y) const  { _x = x; _y = y; }
inline void Complex::getNegative(Complex& z) const { z.setNegative(*this); }
inline void Complex::getNormalized(Complex& z) const { z.setNormalized(*this); }
inline void Complex::getConjugate(Complex& z) const { z.setConjugate(*this); }
inline void Complex::getPolar(Real& r, Real& theta) { r = norm(); theta=arg(); }
inline bool Complex::getInverse(Complex& z) const { return z.setInverse(*this); }
inline void Complex::inplaceNegative() { x = -x; y = -y; }
inline void Complex::inplaceScale(Real c) { x*=c; y*=c; }
inline void Complex::inplaceNormalize() { inplaceScale(Inv(norm())); }
inline void Complex::inplaceConjugate() { y = -y; }
inline bool Complex::inplaceInverse() { Real n2 = normSquared(); if(n2==Zero) return false; inplaceScale(Inv(n2)); inplaceConjugate(); return true; }
inline bool Complex::isInvertible() const { return !(x == Zero && y == Zero); }
inline Real Complex::normSquared() const { return Sqr(x)+Sqr(y); }
inline Real Complex::norm() const { return Sqrt(normSquared()); }
inline Real Complex::arg() const { return Atan2(y,x); }
inline const Real& Complex::Re() const { return x; }
inline const Real& Complex::Im() const { return y; }
inline Real& Complex::Re() { return x; }
inline Real& Complex::Im() { return y; }


inline bool Quaternion::operator == (const Quaternion& a) const { return a.x == x && a.y == y && a.z == z && a.w == w; }
inline bool Quaternion::operator != (const Quaternion& a) const { return a.x != x || a.y != y || a.z != z || a.w != w; }
inline const Quaternion& Quaternion::operator = (const Quaternion& q) { set(q); return *this; }
inline void Quaternion::operator += (const Quaternion& q) { w+=q.w; x+=q.x; y+=q.y; z+=q.z; }
inline void Quaternion::operator -= (const Quaternion& q) { w-=q.w; x-=q.x; y-=q.y; z-=q.z; }
inline void Quaternion::operator *= (const Quaternion& q) { mul(*this,q); }
inline void Quaternion::operator /= (const Quaternion& q) { div(*this,q); }
inline const Quaternion& Quaternion::operator = (Real s) { set(s); return *this; }
inline void Quaternion::operator += (Real s) { w += s; }
inline void Quaternion::operator -= (Real s) { w -= s; }
inline void Quaternion::operator *= (Real s) { w*=s; x*=s; y*=s; z*=s; }
inline void Quaternion::operator /= (Real s) { Real s_inv = Inv(s); operator *= (s_inv); }
inline void Quaternion::add(const Quaternion& a, const Quaternion& b) { w=a.w+b.w; x=a.x+b.x; y=a.y+b.y; z=a.z+b.z; }
inline void Quaternion::sub(const Quaternion& a, const Quaternion& b) { w=a.w-b.w; x=a.x-b.x; y=a.y-b.y; z=a.z-b.z; }
inline void Quaternion::mul(const Quaternion& a, Real b) { w=a.w*b; x=a.x*b; y=a.y*b; z=a.z*b; }
inline void Quaternion::div(const Quaternion& a, Real b) { Real binv = Inv(b); mul(a,binv); }
inline void Quaternion::madd(const Quaternion& a, Real b) { w+=a.w*b; x+=a.x*b; y+=a.y*b; z+=a.z*b; }
inline void Quaternion::set(const Quaternion& q) { w=q.w; x=q.x; y=q.y; z=q.z; }
inline void Quaternion::set(Real _w) { w=_w; x = y = z = Zero; }
inline void Quaternion::set(Real _w, const Real* im)  { w=_w; x=im[0]; y=im[1]; z=im[2]; }
inline void Quaternion::set(Real _w, Real _x, Real _y, Real _z) { w=_w; x=_x; y=_y; z=_z; }
inline void Quaternion::setZero() { w = x = y = z = Zero; }
inline void Quaternion::setNegative(const Quaternion& q) { w=-q.w; x=-q.x; y=-q.y; z=-q.z; }
inline void Quaternion::setNormalized(const Quaternion& q) { div(q,q.norm()); }
inline void Quaternion::setConjugate (const Quaternion& q) { w = q.w; x = -q.x; y = -q.y; z = -q.z; }
inline bool Quaternion::setInverse (const Quaternion& q) { Real n2=q.normSquared(); if(n2==Zero) return false; div(q,Inv(n2)); inplaceConjugate(); return true; }
inline void Quaternion::get(Quaternion& q) const { q.set(*this); }
inline void Quaternion::get(Real& _w, Real* im) const { _w=w; im[0]=x; im[1]=y; im[2]=z; }
inline void Quaternion::get(Real& _w, Real& _x, Real& _y, Real& _z) const { _w=w; _x=x; _y=y; _z=z; }
inline void Quaternion::getNegative(Quaternion& q) const { q.setNegative(*this); }
inline void Quaternion::getNormalized(Quaternion& q) const { q.setNormalized(*this); }
inline void Quaternion::getConjugate (Quaternion& q) const { q.setConjugate(*this); }
inline bool Quaternion::getInverse (Quaternion& q) const { return q.setInverse(*this); }
inline void Quaternion::inplaceNegative() { w=-w; x=-x; y=-y; z=-z; }
inline void Quaternion::inplaceScale(Real c) { w*=c; x*=c; y*=c; z*=c; }
inline void Quaternion::inplaceNormalize() { inplaceScale(Inv(norm())); }
inline void Quaternion::inplaceConjugate() { x=-x; y=-y; z=-z; }
inline bool Quaternion::inplaceInverse() { Real n2 = normSquared(); if(n2==Zero) return false; inplaceScale(Inv(n2)); inplaceConjugate(); return true; }
inline bool Quaternion::isInvertible() const { return !(w == Zero && x == Zero && y == Zero && z == Zero); }
inline Real Quaternion::norm() const { return Sqrt(normSquared()); }
inline Real Quaternion::normSquared() const { return Sqr(w)+Sqr(x)+Sqr(y)+Sqr(z); }
inline Real Quaternion::imNorm() const { return Sqr(x)+Sqr(y)+Sqr(z); }

//inlined standalone functions/operators (often less efficient than the member functions)



//Complex

inline Real dot(const Complex& a, const Complex& b)
{
  return a.x*b.x + a.y*b.y;
}

inline Complex operator - (const Complex& a)
{
  Complex temp;
  temp.setNegative(a);
  return temp;
}

inline Complex operator + (const Complex& a, const Complex& b)
{
  Complex temp;
  temp.add(a,b);
  return temp;
}

inline Complex operator - (const Complex& a, const Complex& b)
{
  Complex temp;
  temp.sub(a,b);
  return temp;
}

inline Complex operator * (const Complex& a, const Complex& b)
{
  Complex temp;
  temp.mul(a,b);
  return temp;
}

inline Complex operator / (const Complex& a, const Complex& b)
{
  Complex temp;
  temp.div(a,b);
  return temp;
}

inline Complex operator + (const Complex& a, Real b)
{
  Complex temp;
  temp.add(a,b);
  return temp;
}

inline Complex operator - (const Complex& a, Real b)
{
  Complex temp;
  temp.sub(a,b);
  return temp;
}

inline Complex operator * (const Complex& a, Real b)
{
  Complex temp;
  temp.mul(a,b);
  return temp;
}

inline Complex operator / (const Complex& a, Real b)
{
  Complex temp;
  temp.div(a,b);
  return temp;
}

inline Real Abs(const Complex& x) { return x.norm(); }
inline Complex Sqr(const Complex& x) { return x*x; }
inline Complex Sqrt(const Complex& x) { Complex z; z.setPow(x,Half); return z; }
inline Complex Exp(const Complex& x) { Complex z; z.setExp(x); return z; }
inline Complex Log(const Complex& x) { Complex z; z.setLog(x); return z; }
inline Complex Pow(const Complex& x, Real y) { Complex z; z.setPow(x,y); return z; }
inline Complex Pow(const Complex& x,const Complex& y) { Complex z; z.setPow(x,y); return z; }
inline Complex Sin(const Complex& z)
{
  Real ey = Exp(z.y), e_y = Inv(ey);
  Real sx = Sin(z.x), cx = Cos(z.x);
  Complex r; 
  r.x = Half*(ey+e_y)*sx;
  r.y = Half*(ey-e_y)*cx;
  return r;
}
inline Complex Cos(const Complex& z)
{
  Real ey = Exp(z.y), e_y = Inv(ey);
  Real sx = Sin(z.x), cx = Cos(z.x);
  Complex r; 
  r.x = Half*(ey+e_y)*cx;
  r.y = Half*(e_y-ey)*sx;
  return r;
}
inline Complex Tan(const Complex& z)
{
  Real ey = Exp(z.y), e_y = Inv(ey);
  Real sx = Sin(z.x), cx = Cos(z.x);
  Complex sz,cz; 
  sz.x = Half*(ey+e_y)*cx;
  sz.y = Half*(e_y-ey)*sx;
  cz.x = Half*(ey+e_y)*sx;
  cz.y = Half*(ey-e_y)*cx;
  return sz/cz;
}
inline Complex Sinh(const Complex& z) 
{
  Real ex = Exp(z.x), e_x = Inv(ex);
  Real sy = Sin(z.y), cy = Cos(z.y);
  Complex r;
  r.x = Half*(ex-e_x)*cy;
  r.y = Half*(ex+e_x)*sy;
  return r;
}
inline Complex Cosh(const Complex& z)
{
  Real ex = Exp(z.x), e_x = Inv(ex);
  Real sy = Sin(z.y), cy = Cos(z.y);
  Complex r;
  r.x = Half*(ex+e_x)*cy;
  r.y = Half*(ex-e_x)*sy;
  return r;
}

inline Complex Tanh(const Complex& z)
{
  Real ex = Exp(z.x), e_x = Inv(ex);
  Real sy = Sin(z.y), cy = Cos(z.y);
  Complex sz,cz; 
  sz.x = Half*(ex-e_x)*cy;
  sz.y = Half*(ex+e_x)*sy;
  cz.x = Half*(ex+e_x)*cy;
  cz.y = Half*(ex-e_x)*sy;
  return sz/cz;
}

inline Complex Inv(const Complex& x) { Complex z; z.setInverse(x); return z; }
inline bool FuzzyEquals(const Complex& a, const Complex& b, Real eps) { return Abs(a-b) <= eps; }
inline bool FuzzyZero(const Complex& a, Real eps) { return Abs(a) <= eps; }
inline Complex PseudoInv(const Complex& x,Real eps) { if(Abs(x)<eps) return Complex(Zero); return Inv(x); }
inline bool FuzzyEquals(const Complex& a, const Complex& b, const Complex& eps) { Assert(eps.y==Zero); return FuzzyEquals(a,b,eps.x); }
inline bool FuzzyZero(const Complex& a, const Complex& eps) { Assert(eps.y==Zero); return FuzzyZero(a,eps.x); }
inline Complex PseudoInv(const Complex& x,const Complex& eps) { Assert(eps.y==Zero); return PseudoInv(x,eps.x); }

//Quaternion

inline Real dot(const Quaternion& a, const Quaternion& b)
{
  return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

inline Quaternion operator - (const Quaternion& a)
{
  Quaternion temp;
  temp.setNegative(a);
  return temp;
}

inline Quaternion operator + (const Quaternion& a, const Quaternion& b)
{
  Quaternion temp;
  temp.add(a,b);
  return temp;
}

inline Quaternion operator - (const Quaternion& a, const Quaternion& b)
{
  Quaternion temp;
  temp.sub(a,b);
  return temp;
}

inline Quaternion operator * (const Quaternion& a, const Quaternion& b)
{
  Quaternion temp;
  temp.mul(a,b);
  return temp;
}

inline Quaternion operator / (const Quaternion& a, const Quaternion& b)
{
  Quaternion temp;
  temp.div(a,b);
  return temp;
}

inline Quaternion operator + (const Quaternion& a, Real b)
{
  Quaternion temp;
  temp.add(a,b);
  return temp;
}

inline Quaternion operator - (const Quaternion& a, Real b)
{
  Quaternion temp;
  temp.sub(a,b);
  return temp;
}

inline Quaternion operator * (const Quaternion& a, Real b)
{
  Quaternion temp;
  temp.mul(a,b);
  return temp;
}

inline Quaternion operator / (const Quaternion& a, Real b)
{
  Quaternion temp;
  temp.div(a,b);
  return temp;
}


//IO functions

std::ostream& operator << (std::ostream&, const Complex&);
std::istream& operator >> (std::istream&, Complex&);

std::ostream& operator << (std::ostream&, const Quaternion&);
std::istream& operator >> (std::istream&, Quaternion&);

}

#endif
