#ifndef MATH3D_PRIMITIVES_H
#define MATH3D_PRIMITIVES_H

#include <mintos/math/math.h>
#include <mintos/misc/utils.h>
#include <iostream>

/** @defgroup Math3D
 * @brief Definitions of 3D math classes and functions 
 */

/** @file math3d/primitives.h
 * @ingroup Math3D
 * @brief Class declarations for useful 3D math types.
 */

/** @ingroup Math3D 
 * @brief Contains all the definitions in the Math3D package.
 */
namespace Math3D {

/** @addtogroup Math3D */
/*@{*/

using namespace Math;

class Vector2;
class Vector3;
class Vector4;
class Matrix2;
class Matrix3;
class Matrix4;
class RigidTransform2D;
class RigidTransform;

/** @brief A 2D vector class
 *
 * The elements can be accessed as (x,y), or using square brackets [{0,1}].
 */
class Vector2
{
 public:
  Vector2();
  Vector2(const Vector2&);
  Vector2(Real x);
  Vector2(Real x, Real y);
  Vector2(const Real* data);

  inline bool operator == (const Vector2&) const;
  inline bool operator != (const Vector2&) const;
  inline const Vector2& operator = (const Vector2&);
  inline void operator += (const Vector2&);
  inline void operator -= (const Vector2&);
  inline void operator *= (Real);
  inline void operator /= (Real);
  inline Real& operator [] (int);
  inline const Real& operator [] (int) const;
  inline operator Real* ();
  inline operator const Real* () const;

  inline void add(const Vector2& a, const Vector2& b);
  inline void sub(const Vector2& a, const Vector2& b);
  inline void mul(const Vector2& a, Real);
  inline void div(const Vector2& a, Real);
  inline void madd(const Vector2& b, Real);
  inline Real dot(const Vector2& a) const;
  inline Real cross(const Vector2& a) const;   ///< 2d cross product
  inline Real distance(const Vector2& a) const;
  inline Real distanceSquared(const Vector2& a) const;
  inline Real norm() const;
  inline Real normSquared() const;
  inline Real length() const;		///< = norm
  inline Real minElement(int* index=NULL) const;
  inline Real maxElement(int* index=NULL) const;
  inline Real minAbsElement(int* index=NULL) const;
  inline Real maxAbsElement(int* index=NULL) const;

  inline void set(const Vector2&);
  inline void set(Real x);
  inline void set(Real x, Real y);
  inline void set(const Real* data);
  inline void setZero();
  inline void setRotation(Real rads);    ///< sets x=cos(rads), y=sin(rads) 
  inline void setPerpendicular(const Vector2&);  ///<sets this to the vector rotated 90 degrees ccw
  inline void setOrthogonal(const Vector2& v) { setPerpendicular(v); }
  inline void setNegative(const Vector2&);
  inline void setNormalized(const Vector2&);
  inline void setProjection(const Vector2&, const Vector2&);  ///< sets this to the projection of a on b
  inline void setMinimum(const Vector2&,const Vector2&);
  inline void setMinimum(const Vector2&);
  inline void setMaximum(const Vector2&,const Vector2&);
  inline void setMaximum(const Vector2&);

  inline void get(Vector2&) const;
  inline void get(Real& x, Real& y) const;
  inline void get(Real data[2]) const;
  inline void getNegative(Vector2&) const;
  inline void getNormalized(Vector2&) const;
  inline void getOrthogonal(Vector2&) const;  ///< calculates an orthogonal vector

  inline void inplaceNegative();
  inline void inplaceMul(Real);
  inline void inplaceDiv(Real);
  inline void inplaceNormalize();

  inline bool isZero(Real eps=Zero) const;
  inline bool isEqual(const Vector2&,Real eps=Zero) const;

  union {
    Real data[2];
    struct { Real x,y; };
  };
};

inline Real dot(const Vector2& a, const Vector2& b);
inline Real cross(const Vector2& a, const Vector2& b);
inline void normalize(Vector2& a);
inline Vector2 project(const Vector2& a, const Vector2& b);
inline Vector2 operator - (const Vector2& a);
inline Vector2 operator + (const Vector2& a, const Vector2& b);
inline Vector2 operator - (const Vector2& a, const Vector2& b);
inline Vector2 operator * (const Vector2& a, Real b);
inline Vector2 operator * (Real a, const Vector2& b);
inline Vector2 operator / (const Vector2& a, Real b);


/** @brief A 3D vector class
 *
 * The elements can be accessed as (x,y,z), or using square brackets 
 * [{0,1,2}].
 */
class Vector3
{
 public:
  Vector3();
  Vector3(const Vector3&);
  Vector3(Real x);
  Vector3(Real x, Real y, Real z);
  Vector3(const Real* data);

  inline bool operator == (const Vector3&) const;
  inline bool operator != (const Vector3&) const;
  inline const Vector3& operator = (const Vector3&);
  inline void operator += (const Vector3&);
  inline void operator -= (const Vector3&);
  inline void operator *= (Real);
  inline void operator /= (Real);
  inline Real& operator [] (int);
  inline const Real& operator [] (int) const;
  inline operator Real* ();
  inline operator const Real* () const;

  inline void add(const Vector3& a, const Vector3& b);
  inline void sub(const Vector3& a, const Vector3& b);
  inline void mul(const Vector3& a, Real);
  inline void div(const Vector3& a, Real);
  inline void madd(const Vector3& b, Real);
  inline Real dot(const Vector3& a) const;
  inline Real distance(const Vector3& a) const;
  inline Real distanceSquared(const Vector3& a) const;
  inline Real norm() const;
  inline Real normSquared() const;
  inline Real length() const;		///< = norm
  inline Real minElement(int* index=NULL) const;
  inline Real maxElement(int* index=NULL) const;
  inline Real minAbsElement(int* index=NULL) const;
  inline Real maxAbsElement(int* index=NULL) const;

  inline void set(const Vector3&);
  inline void set(Real x);
  inline void set(Real x, Real y, Real z);
  inline void set(const Real* data);
  inline void setZero();
  inline void setNegative(const Vector3&);
  inline void setCross(const Vector3&, const Vector3&);
  inline void setNormalized(const Vector3&);
  inline void setProjection(const Vector3&, const Vector3&);
  inline void setMinimum(const Vector3&,const Vector3&);
  inline void setMinimum(const Vector3&);
  inline void setMaximum(const Vector3&,const Vector3&);
  inline void setMaximum(const Vector3&);

  inline void get(Vector3&) const;
  inline void get(Real& x, Real& y, Real& z) const;
  inline void get(Real data[3]) const;
  inline void getNegative(Vector3&) const;
  inline void getNormalized(Vector3&) const;
  inline void getOrthogonalBasis(Vector3& xb,Vector3& yb) const;  ///< calculates two unit vectors orthogonal to this vector (and positively oriented)

  inline void inplaceNegative();
  inline void inplaceMul(Real);
  inline void inplaceDiv(Real);
  inline void inplaceNormalize();

  inline bool isZero(Real eps=Zero) const;
  inline bool isEqual(const Vector3&,Real eps=Zero) const;

  union {
    Real data[3];
    struct { Real x,y,z; };
  };
};

inline Real dot(const Vector3& a, const Vector3& b);
inline Vector3 cross(const Vector3& a, const Vector3& b);
inline void normalize(Vector3& a);
inline Vector3 project(const Vector3& a, const Vector3& b);
inline Vector3 operator - (const Vector3& a);
inline Vector3 operator + (const Vector3& a, const Vector3& b);
inline Vector3 operator - (const Vector3& a, const Vector3& b);
inline Vector3 operator * (const Vector3& a, Real b);
inline Vector3 operator * (Real a, const Vector3& b);
inline Vector3 operator / (const Vector3& a, Real b);


/** @brief A 4D vector class
 *
 * The elements can be accessed as (x,y,z,w), or using square brackets 
 * [{0,1,2,3}].
 */
class Vector4
{
 public:
  Vector4();
  Vector4(const Vector4&);
  Vector4(Real x);
  Vector4(Real x, Real y, Real z, Real w = One);
  Vector4(const Real* data);
  Vector4(const Vector3&);		//point in homogeneous coords

  inline bool operator == (const Vector4&) const;
  inline bool operator != (const Vector4&) const;
  inline const Vector4& operator = (const Vector4&);
  inline void operator += (const Vector4&);
  inline void operator -= (const Vector4&);
  inline void operator *= (Real);
  inline void operator /= (Real);
  inline void operator += (const Vector3&);
  inline void operator -= (const Vector3&);
  inline Real& operator [] (int);
  inline const Real& operator [] (int) const;
  inline operator Vector3() const;
  inline operator Real* ();
  inline operator const Real* () const;

  inline void add(const Vector4& a, const Vector4& b);
  inline void sub(const Vector4& a, const Vector4& b);
  inline void mul(const Vector4& a, Real);
  inline void div(const Vector4& a, Real);
  inline void madd(const Vector4& a, Real);
  inline void add(const Vector4& a, const Vector3& b);
  inline void sub(const Vector4& a, const Vector3& b);
  inline Real dot(const Vector4& a) const;
  inline Real dot3(const Vector4& a) const;
  inline Real dot3(const Vector3& a) const;
  inline Real distance(const Vector4& a) const;
  inline Real distanceSquared(const Vector4& a) const;
  inline Real norm() const;
  inline Real normSquared() const;
  inline Real length() const;		// = norm
  inline Real minElement(int* index=NULL) const;
  inline Real maxElement(int* index=NULL) const;
  inline Real minAbsElement(int* index=NULL) const;
  inline Real maxAbsElement(int* index=NULL) const;

  inline void set(const Vector4&);
  inline void set(Real x);
  inline void set(Real x, Real y, Real z, Real w = One);
  inline void set(const Real* data);
  inline void setZero();
  inline void setVector(const Vector3&);	       ///<sets w = 0
  inline void setHomogeneous(const Vector3&);	       ///<sets w = 1
  inline void setNegative(const Vector4&);
  inline void setNormalized(const Vector4&);
  inline void setProjection(const Vector4&, const Vector4&);
  inline void setMinimum(const Vector4&,const Vector4&);
  inline void setMinimum(const Vector4&);
  inline void setMaximum(const Vector4&,const Vector4&);
  inline void setMaximum(const Vector4&);

  inline void get(Vector4&) const;
  inline void get(Real& x, Real& y, Real& z, Real& w) const;
  inline void get(Real data[4]) const;
  inline void get(Vector3&) const;
  inline void getNegative(Vector4&) const;
  inline void getNormalized(Vector4&) const;

  inline void inplaceNegative();
  inline void inplaceMul(Real);
  inline void inplaceDiv(Real);
  inline void inplaceNormalize();

  inline bool isZero(Real eps=Zero) const;
  inline bool isEqual(const Vector4&,Real eps=Zero) const;

  union {
    Real data[4];
    struct { Real x,y,z,w; };
  };
};

inline Real dot(const Vector4& a, const Vector4& b);
inline Real dot3(const Vector4& a, const Vector4& b);
inline Real dot3(const Vector4& a, const Vector3& b);
inline Real dot3(const Vector3& a, const Vector4& b);
inline void normalize(Vector4& a);
inline Vector4 project(const Vector4& a, const Vector4& b);
inline Vector4 operator - (const Vector4& a);
inline Vector4 operator + (const Vector4& a, const Vector4& b);
inline Vector4 operator - (const Vector4& a, const Vector4& b);
inline Vector4 operator * (const Vector4& a, Real b);
inline Vector4 operator * (Real a, const Vector4& b);
inline Vector4 operator / (const Vector4& a, Real b);


/** @brief A 2x2 matrix class
 *
 * The elements can be accessed as (i,j) for i,j in [0,1].
 *
 * WARNING: For historical reasons, the data array data[2][2] is in 
 * column major format, e.g. data[i][j] = element (j,i).
 */
class Matrix2
{
 public:
  Matrix2();
  Matrix2(const Matrix2&);
  Matrix2(Real);
  Matrix2(const Real [2][2]);
  Matrix2(const Real *);
  Matrix2(const Vector2& xb, const Vector2& yb);

  bool operator == (const Matrix2&) const;
  bool operator != (const Matrix2&) const;
  inline const Matrix2& operator  = (const Matrix2&);
  inline void operator += (const Matrix2&);
  inline void operator -= (const Matrix2&);
  inline void operator *= (const Matrix2&);
  inline void operator *= (Real scale);
  inline void operator /= (Real scale);
  operator const Real*() const { return &data[0][0]; }
  inline Real& operator () (int i, int j) { return data[j][i]; }
  inline const Real& operator () (int i, int j) const { return data[j][i]; }

  ///@name matrix ops (this = a op b)
  ///@{
  inline void add(const Matrix2& a, const Matrix2& b);
  inline void sub(const Matrix2& a, const Matrix2& b);
  inline void mul(const Matrix2& a, const Matrix2& b);
  inline void mulTransposeA(const Matrix2& a, const Matrix2& b);
  inline void mulTransposeB(const Matrix2& a, const Matrix2& b);
  ///@}

  ///@name scalar ops (this = a op b)
  ///@{
  inline void mul(const Matrix2& a, Real b);
  inline void div(const Matrix2& a, Real b);
  ///@}

  ///@name vector ops (out = this op a)
  ///@{
  inline void mul(const Vector2& a, Vector2& out) const;			//result is incorrect if out = a
  inline void mulTranspose(const Vector2& a, Vector2& out) const;	//result is incorrect if out = a
  ///@}

  inline void set(const Matrix2&);
  inline void set(Real);
  inline void set(const Real [2][2]);
  inline void set(const Real*);
  inline void set(const Vector2& xb, const Vector2& yb);
  inline void setZero();
  inline void setIdentity();
  inline void setDiagonal(const Vector2&);
  inline void setTranspose(const Matrix2&);
  inline void setNegative(const Matrix2&);
  inline bool setInverse(const Matrix2&);   ///< returns true when successful
  inline void setScale(Real s);
  inline void setScale(Real sx, Real sy);
  inline void setRotate(Real rads);  ///< sets the matrix that rotates ccw by rads
  inline void setOuterProduct(const Vector2& a,const Vector2& b); ///< this = a*b^t

  inline void get(Matrix2&) const;
  inline void get(Real [2][2]) const;
  inline void get(Real [4]) const;
  inline void get(Vector2& xb, Vector2& yb) const;
  inline void getTranspose(Matrix2&) const;
  inline void getNegative(Matrix2&) const;
  inline bool getInverse(Matrix2&) const;

  inline void inplaceTranspose();
  inline void inplaceNegative();
  inline bool inplaceInverse();
  inline void inplaceMul(Real s);
  inline void inplaceDiv(Real s);
  inline void inplaceRowScale(Real sx, Real sy);
  inline void inplaceColScale(Real sx, Real sy);

  inline bool isZero(Real eps=Zero) const;
  inline bool isIdentity(Real eps=Zero) const;
  inline bool isEqual(const Matrix2&,Real eps=Zero) const;
  inline bool isInvertible(Real eps=Zero) const;
  inline Real determinant() const;
  inline Real trace() const;
  inline Real minElement(int* i=NULL,int* j=NULL) const;
  inline Real maxElement(int* i=NULL,int* j=NULL) const;
  inline Real minAbsElement(int* i=NULL,int* j=NULL) const;
  inline Real maxAbsElement(int* i=NULL,int* j=NULL) const;

  inline Real* col(int j) { return data[j]; }
  inline Real* col1() { return data[0]; }
  inline Real* col2() { return data[1]; }
  inline const Real* col(int j) const { return data[j]; }
  inline const Real* col1() const { return data[0]; }
  inline const Real* col2() const { return data[1]; }

  inline void setCol(int j, const Vector2& v) { v.get(data[j]); }
  inline void setCol1(const Vector2& v) { v.get(data[0]); }
  inline void setCol2(const Vector2& v) { v.get(data[1]); }

  inline void setRow(int i, const Vector2& v) { v.get(data[0][i], data[1][i]); }
  inline void setRow1(const Vector2& v) { v.get(data[0][0], data[1][0]); }
  inline void setRow2(const Vector2& v) { v.get(data[0][1], data[1][1]); }

  inline void getCol(int j, Vector2& v) const { v.set(data[j]); }
  inline void getCol1(Vector2& v) const { v.set(data[0]); }
  inline void getCol2(Vector2& v) const { v.set(data[1]); }

  inline void getRow(int i, Vector2& v) const { v.set(data[0][i], data[1][i]); }
  inline void getRow1(Vector2& v) const { v.set(data[0][0], data[1][0]); }
  inline void getRow2(Vector2& v) const { v.set(data[0][1], data[1][1]); }

  inline Vector2 getXBasis() const { return Vector2(col1()); }
  inline Vector2 getYBasis() const { return Vector2(col2()); }

  ///column major format
  Real data[2][2];
};

inline Real determinant(const Matrix2&);
inline Real trace(const Matrix2&);
inline Matrix2 operator + (const Matrix2& a, const Matrix2& b);
inline Matrix2 operator - (const Matrix2& a, const Matrix2& b);
inline Matrix2 operator * (const Matrix2& a, const Matrix2& b);
inline Vector2 operator * (const Matrix2& a, const Vector2& b);
inline Vector2 operator * (const Vector2& a, const Matrix2& b);



/** @brief A 3x3 matrix class
 *
 * The elements can be accessed as (i,j) for i,j in [0,2].
 *
 * WARNING: For historical reasons, the data array data[2][2] is in 
 * column major format, e.g. data[i][j] = element (j,i).
 */
class Matrix3
{
 public:
  Matrix3();
  Matrix3(const Matrix3&);
  Matrix3(Real);
  Matrix3(const Real [3][3]);
  Matrix3(const Real *);
  Matrix3(const Vector3& xb, const Vector3& yb, const Vector3& zb);

  bool operator == (const Matrix3&) const;
  bool operator != (const Matrix3&) const;
  inline const Matrix3& operator  = (const Matrix3&);
  inline void operator += (const Matrix3&);
  inline void operator -= (const Matrix3&);
  inline void operator *= (const Matrix3&);
  inline void operator *= (Real scale);
  inline void operator /= (Real scale);
  operator const Real*() const { return &data[0][0]; }
  inline Real& operator () (int i, int j) { return data[j][i]; }
  inline const Real& operator () (int i, int j) const { return data[j][i]; }

  ///@name matrix ops (this = a op b)
  ///@{
  inline void add(const Matrix3& a, const Matrix3& b);
  inline void sub(const Matrix3& a, const Matrix3& b);
  void mul(const Matrix3& a, const Matrix3& b);
  void mulTransposeA(const Matrix3& a, const Matrix3& b);
  void mulTransposeB(const Matrix3& a, const Matrix3& b);
  ///@}

  ///@name scalar ops (this = a op b)
  ///@}
  inline void mul(const Matrix3& a, Real b);
  inline void div(const Matrix3& a, Real b);

  ///@name vector ops (out = this op a)
  ///@{
  inline void mul(const Vector3& a, Vector3& out) const;          ///<Note: result is incorrect if out = a
  inline void mulTranspose(const Vector3& a, Vector3& out) const; ///<Note: result is incorrect if out = a
  ///@}

  ///@name homogeneous vector ops (3rd coord is homogeneous coord)
  ///@{
  inline void mulPoint(const Vector2& a, Vector2& out) const; ///<assumes w = 1
  inline void mulVector(const Vector2& a, Vector2& out) const;///<assumes w = 0
  inline void mulVectorTranspose(const Vector2& a, Vector2& out) const;	///<assumes w = 0
  ///@}

  inline void set(const Matrix3&);
  inline void set(Real);
  inline void set(const Real [3][3]);
  inline void set(const Real [9]);
  inline void set(const Vector3& xb, const Vector3& yb, const Vector3& zb);
  inline void setZero();
  inline void setIdentity();
  inline void setDiagonal(const Vector3&);
  inline void setTranspose(const Matrix3&);
  inline void setNegative(const Matrix3&);
  bool setInverse(const Matrix3&);
  inline void setScale(Real s);
  inline void setScale(Real sx, Real sy, Real sz);
  inline void setCrossProduct(const Vector3&);  ///<sets the matrix that performs the vector cross product 
  inline void setRotateX(Real rads);  ///<sets the matrix that rotates ccw by rads around the x axis
  inline void setRotateY(Real rads);  ///<sets the matrix that rotates ccw by rads around the x axis
  inline void setRotateZ(Real rads);  ///<sets the matrix that rotates ccw by rads around the x axis
  inline void setOuterProduct(const Vector3& a,const Vector3& b); ///< this = a*b^t

  inline void get(Matrix3&) const;
  inline void get(Real [3][3]) const;
  inline void get(Real [9]) const;
  inline void get(Vector3& xb, Vector3& yb, Vector3& zb) const;
  inline void getTranspose(Matrix3&) const;
  inline void getNegative(Matrix3&) const;
  inline bool getInverse(Matrix3&) const;
  inline void getCrossProduct(Vector3&) const;  ///<if this is a cross-product matrix, returns the vector that performs the cross product

  inline void inplaceTranspose();
  inline void inplaceNegative();
  inline bool inplaceInverse();
  inline void inplaceMul(Real s);
  inline void inplaceDiv(Real s);
  inline void inplaceRowScale(Real sx, Real sy, Real sz);
  inline void inplaceColScale(Real sx, Real sy, Real sz);

  inline bool isZero(Real eps=Zero) const;
  inline bool isIdentity(Real eps=Zero) const;
  inline bool isEqual(const Matrix3&,Real eps=Zero) const;
  inline bool isInvertible(Real eps=Zero) const;
  Real cofactor(int i, int j) const;
  Real determinant() const;
  inline Real trace() const;
  inline Real minElement(int* i=NULL,int* j=NULL) const;
  inline Real maxElement(int* i=NULL,int* j=NULL) const;
  inline Real minAbsElement(int* i=NULL,int* j=NULL) const;
  inline Real maxAbsElement(int* i=NULL,int* j=NULL) const;

  inline Real* col(int j) { return data[j]; }
  inline Real* col1() { return data[0]; }
  inline Real* col2() { return data[1]; }
  inline Real* col3() { return data[2]; }
  inline const Real* col(int j) const { return data[j]; }
  inline const Real* col1() const { return data[0]; }
  inline const Real* col2() const { return data[1]; }
  inline const Real* col3() const { return data[2]; }

  inline void setCol(int j, const Vector3& v) { v.get(data[j]); }
  inline void setCol1(const Vector3& v) { v.get(data[0]); }
  inline void setCol2(const Vector3& v) { v.get(data[1]); }
  inline void setCol3(const Vector3& v) { v.get(data[2]); }

  inline void setRow(int i, const Vector3& v) { v.get(data[0][i], data[1][i], data[2][i]); }
  inline void setRow1(const Vector3& v) { v.get(data[0][0], data[1][0], data[2][0]); }
  inline void setRow2(const Vector3& v) { v.get(data[0][1], data[1][1], data[2][1]); }
  inline void setRow3(const Vector3& v) { v.get(data[0][2], data[1][2], data[2][2]); }

  inline void getCol(int j, Vector3& v) const { v.set(data[j]); }
  inline void getCol1(Vector3& v) const { v.set(data[0]); }
  inline void getCol2(Vector3& v) const { v.set(data[1]); }
  inline void getCol3(Vector3& v) const { v.set(data[2]); }

  inline void getRow(int i, Vector3& v) const { v.set(data[0][i], data[1][i], data[2][i]); }
  inline void getRow1(Vector3& v) const { v.set(data[0][0], data[1][0], data[2][0]); }
  inline void getRow2(Vector3& v) const { v.set(data[0][1], data[1][1], data[2][1]); }
  inline void getRow3(Vector3& v) const { v.set(data[0][2], data[1][2], data[2][2]); }

  inline Vector3 getXBasis() const { return Vector3(col1()); }
  inline Vector3 getYBasis() const { return Vector3(col2()); }
  inline Vector3 getZBasis() const { return Vector3(col3()); }

  inline Vector2 getXBasis2D() const { return Vector2(col1()); }
  inline Vector2 getYBasis2D() const { return Vector2(col2()); }
  inline Vector2 getTranslation2D() const { return Vector2(col3()); }

  ///column major format
  Real data[3][3];
};

inline Real determinant(const Matrix3&);
inline Real trace(const Matrix3&);
inline Matrix3 operator + (const Matrix3& a, const Matrix3& b);
inline Matrix3 operator - (const Matrix3& a, const Matrix3& b);
inline Matrix3 operator * (const Matrix3& a, const Matrix3& b);
inline Vector3 operator * (const Matrix3& a, const Vector3& b);
inline Vector3 operator * (const Vector3& a, const Matrix3& b);


/** @brief A 4x4 matrix class
 *
 * The elements can be accessed as (i,j) for i,j in [0,3].
 *
 * WARNING: For historical reasons, the data array data[2][2] is in 
 * column major format, e.g. data[i][j] = element (j,i).
 */
class Matrix4
{
 public:
  Matrix4();
  Matrix4(const Matrix4&);
  Matrix4(Real);
  Matrix4(const Real [4][4]);
  Matrix4(const Real *);
  Matrix4(const Vector3& xb, const Vector3& yb, const Vector3& zb, const Vector3& trans);
  Matrix4(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w);
  Matrix4(const Matrix3&);
  Matrix4(const Matrix3&, const Vector3& trans);
  Matrix4(const Vector3& trans);

  bool operator == (const Matrix4&) const;
  bool operator != (const Matrix4&) const;
  inline const Matrix4& operator  = (const Matrix4&);
  inline void operator += (const Matrix4&);
  inline void operator -= (const Matrix4&);
  inline void operator *= (const Matrix4&);
  inline void operator *= (Real scale);
  inline void operator /= (Real scale);
  operator Matrix3() const;
  inline operator const Real*() const { return &data[0][0]; }
  inline Real& operator () (int i, int j) { return data[j][i]; }
  inline const Real& operator () (int i, int j) const { return data[j][i]; }

  ///@name matrix ops (this = a op b)
  ///@{
  inline void add(const Matrix4& a, const Matrix4& b);
  inline void sub(const Matrix4& a, const Matrix4& b);
  void mul(const Matrix4& a, const Matrix4& b);
  void mulTransposeA(const Matrix4& a, const Matrix4& b);
  void mulTransposeB(const Matrix4& a, const Matrix4& b);
  ///@}

  ///@name scalar ops (this = a op b)
  ///@{
  inline void mul(const Matrix4& a, Real b);
  inline void div(const Matrix4& a, Real b);
  ///@}

  //vector ops (out = this op a)
  ///@{
  inline void mul(const Vector4& a, Vector4& out) const;
  inline void mulTranspose(const Vector4& a, Vector4& out) const;
  ///@}

  ///@name homogeneous vector ops (4th coord is homogeneous coord)
  ///@{
  inline void mulPoint(const Vector3& a, Vector3& out) const; ///<assumes w = 1
  inline void mulVector(const Vector3& a, Vector3& out) const;///<assumes w = 0
  inline void mulVectorTranspose(const Vector3& a, Vector3& out) const;	///<assumes w = 0
  ///@}

  inline void set(const Matrix4&);
  inline void set(Real);
  inline void set(const Real [4][4]);
  inline void set(const Real *);
  inline void set(const Vector3& xb, const Vector3& yb, const Vector3& zb, const Vector3& trans);
  inline void set(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w);
  inline void set(const Matrix3&);  ///<sets the upper 3x3 matrix
  inline void set(const Matrix3&, const Vector3& trans);  ///<sets the upper 3x3 matrix and the upper 3x1 vector (making an affine 3D transformation)
  inline void setZero();
  inline void setIdentity();
  inline void setDiagonal(const Vector4&);
  inline void setTranslate(const Vector3& trans);
  inline void setTranspose(const Matrix4&);
  inline void setNegative(const Matrix4&);
  bool setInverse(const Matrix4&);
  inline void setOuterProduct(const Vector4& a,const Vector4& b); ///< this = a*b^t

  inline void get(Matrix4&) const;
  inline void get(Real [4][4]) const;
  inline void get(Real [16]) const;
  inline void get(Vector3& xb, Vector3& yb, Vector3& zb, Vector3& trans) const;
  inline void get(Vector4& x, Vector4& y, Vector4& z, Vector4& w) const;
  inline void get(Matrix3&) const;
  inline void get(Matrix3&, Vector3&) const;
  inline void getTranspose(Matrix4&) const;
  inline void getNegative(Matrix4&) const;
  inline bool getInverse(Matrix4&) const;

  inline void inplaceTranspose();
  inline void inplaceNegative();
  inline bool inplaceInverse();
  inline void inplaceMul(Real s);
  inline void inplaceDiv(Real s);

  inline bool isZero(Real eps=Zero) const;
  inline bool isIdentity(Real eps=Zero) const;
  inline bool isEqual(const Matrix4&,Real eps=Zero) const;
  inline bool isInvertible(Real eps=Zero) const;
  Real cofactor(int i, int j) const;
  Real determinant() const;
  inline Real trace() const;
  inline Real minElement(int* i=NULL,int* j=NULL) const;
  inline Real maxElement(int* i=NULL,int* j=NULL) const;
  inline Real minAbsElement(int* i=NULL,int* j=NULL) const;
  inline Real maxAbsElement(int* i=NULL,int* j=NULL) const;

  inline Real* col(int j) { return data[j]; }
  inline Real* col1() { return data[0]; }
  inline Real* col2() { return data[1]; }
  inline Real* col3() { return data[2]; }
  inline Real* col4() { return data[3]; }
  inline const Real* col(int j) const { return data[j]; }
  inline const Real* col1() const { return data[0]; }
  inline const Real* col2() const { return data[1]; }
  inline const Real* col3() const { return data[2]; }
  inline const Real* col4() const { return data[3]; }

  inline void setCol(int j, const Vector4& v) { v.get(data[j]); }
  inline void setCol1(const Vector4& v) { v.get(data[0]); }
  inline void setCol2(const Vector4& v) { v.get(data[1]); }
  inline void setCol3(const Vector4& v) { v.get(data[2]); }
  inline void setCol4(const Vector4& v) { v.get(data[3]); }
  inline void setCol(int j, const Vector3& v) { v.get(data[j]); }
  inline void setCol1(const Vector3& v) { v.get(data[0]); }
  inline void setCol2(const Vector3& v) { v.get(data[1]); }
  inline void setCol3(const Vector3& v) { v.get(data[2]); }
  inline void setCol4(const Vector3& v) { v.get(data[3]); }

  inline void setRow(int i, const Vector4& v) { v.get(data[0][i], data[1][i], data[2][i], data[3][i]); }
  inline void setRow1(const Vector4& v) { v.get(data[0][0], data[1][0], data[2][0], data[3][0]); }
  inline void setRow2(const Vector4& v) { v.get(data[0][1], data[1][1], data[2][1], data[3][1]); }
  inline void setRow3(const Vector4& v) { v.get(data[0][2], data[1][2], data[2][2], data[3][2]); }
  inline void setRow4(const Vector4& v) { v.get(data[0][3], data[1][3], data[2][3], data[3][3]); }
  inline void setRow(int i, const Vector3& v) { v.get(data[0][i], data[1][i], data[2][i]); }
  inline void setRow1(const Vector3& v) { v.get(data[0][0], data[1][0], data[2][0]); }
  inline void setRow2(const Vector3& v) { v.get(data[0][1], data[1][1], data[2][1]); }
  inline void setRow3(const Vector3& v) { v.get(data[0][2], data[1][2], data[2][2]); }
  inline void setRow4(const Vector3& v) { v.get(data[0][3], data[1][3], data[2][3]); }

  inline void getCol(int j, Vector4& v) const { v.set(data[j]); }
  inline void getCol1(Vector4& v) const { v.set(data[0]); }
  inline void getCol2(Vector4& v) const { v.set(data[1]); }
  inline void getCol3(Vector4& v) const { v.set(data[2]); }
  inline void getCol4(Vector4& v) const { v.set(data[3]); }
  inline void getCol(int j, Vector3& v) const { v.set(data[j]); }
  inline void getCol1(Vector3& v) const { v.set(data[0]); }
  inline void getCol2(Vector3& v) const { v.set(data[1]); }
  inline void getCol3(Vector3& v) const { v.set(data[2]); }
  inline void getCol4(Vector3& v) const { v.set(data[3]); }

  inline void getRow(int i, Vector4& v) const { v.set(data[0][i], data[1][i], data[2][i], data[3][i]); }
  inline void getRow1(Vector4& v) const { v.set(data[0][0], data[1][0], data[2][0], data[3][0]); }
  inline void getRow2(Vector4& v) const { v.set(data[0][1], data[1][1], data[2][1], data[3][1]); }
  inline void getRow3(Vector4& v) const { v.set(data[0][2], data[1][2], data[2][2], data[3][2]); }
  inline void getRow4(Vector4& v) const { v.set(data[0][3], data[1][3], data[2][3], data[3][3]); }
  inline void getRow(int i, Vector3& v) const { v.set(data[0][i], data[1][i], data[2][i]); }
  inline void getRow1(Vector3& v) const { v.set(data[0][0], data[1][0], data[2][0]); }
  inline void getRow2(Vector3& v) const { v.set(data[0][1], data[1][1], data[2][1]); }
  inline void getRow3(Vector3& v) const { v.set(data[0][2], data[1][2], data[2][2]); }
  inline void getRow4(Vector3& v) const { v.set(data[0][3], data[1][3], data[2][3]); }

  ///the following are for 4x4 homogeneous transforms
  inline Vector3 getXBasis() const { return Vector3(col1()); }
  inline Vector3 getYBasis() const { return Vector3(col2()); }
  inline Vector3 getZBasis() const { return Vector3(col3()); }
  inline Vector3 getTranslation() const { return Vector3(col4()); }

  ///column major format
  Real data[4][4];
};

inline Real determinant(const Matrix4&);
inline Real trace(const Matrix4&);
inline Matrix4 operator + (const Matrix4& a, const Matrix4& b);
inline Matrix4 operator - (const Matrix4& a, const Matrix4& b);
inline Matrix4 operator * (const Matrix4& a, const Matrix4& b);
inline Vector4 operator * (const Matrix4& a, const Vector4& b);
inline Vector4 operator * (const Vector4& a, const Matrix4& b);
inline Vector3 operator * (const Matrix4& a, const Vector3& b);		///<WARNING: vector multiply
inline Vector3 operator * (const Vector3& a, const Matrix4& b);		///<WARNING: vector multiply (transpose)


/** @brief A rigid-body transformation.
 *
 * A linear transformation of the form T(t)*R where R is an orthogonal
 * (rotation) matrix, and T(t) is a translation by the vector t.
 *
 * The property that R is orthogonal is assumed to be user-maintained.
 *
 * This representation has the following properties:<br>
 * 1) \f$ R^{-1} = R^t  \f$ <br>
 * 2) \f$ R*T(t) = T(R*t)*R \f$.
 *
 * Therefore, many operations are more efficient than representing the
 * transformation as a 4x4 homogeneous matrix.
 */
class RigidTransform
{
 public:
  RigidTransform();
  RigidTransform(const RigidTransform&);
  RigidTransform(const Matrix3&, const Vector3&);
  RigidTransform(const Vector3&, const Vector3&, const Vector3&, const Vector3&);
  RigidTransform(const Matrix4&);

  inline bool operator == (const RigidTransform&) const;
  inline bool operator != (const RigidTransform&) const;
  inline const RigidTransform& operator = (const RigidTransform&);
  inline void operator *= (const RigidTransform&);									//this(v) = this(a(v))
  inline void operator *= (const Matrix3&);
  inline void operator += (const Vector3&);
  inline void operator -= (const Vector3&);
  inline operator Matrix4 () const;

  inline void compose(const RigidTransform& a, const RigidTransform& b);		///<this(v) = a(b(v))
  inline void composeInverseA(const RigidTransform& a, const RigidTransform& b);	///<this(v) = a^-1(b(v))
  inline void composeInverseB(const RigidTransform& a, const RigidTransform& b);	///<this(v) = a(b^-1(v))
  ///mul = compose
  inline void mul(const RigidTransform& a, const RigidTransform& b);
  inline void mulInverseA(const RigidTransform& a, const RigidTransform& b);
  inline void mulInverseB(const RigidTransform& a, const RigidTransform& b);

  ///@name vector operators
  ///@{
  inline void mul(const Vector3& a, Vector3& out) const;						///< = mulPoint
  inline void mul(const Vector4& a, Vector3& out) const;
  inline void mulPoint(const Vector3& a, Vector3& out) const;
  inline void mulVector(const Vector3& a, Vector3& out) const;
  inline void mulInverse(const Vector3& a, Vector3& out) const;                               ///< = mulPointInverse
  inline void mulInverse(const Vector4& a, Vector3& out) const;
  inline void mulPointInverse(const Vector3& a, Vector3& out) const;
  inline void mulVectorInverse(const Vector3& a, Vector3& out) const;
  ///@}

  inline void setIdentity();
  inline void set(const RigidTransform&);
  inline void set(const Matrix3&, const Vector3&);
  inline void set(const Vector3& x, const Vector3& y, const Vector3& z, const Vector3& trans);
  inline void set(const Matrix4&);
  inline void setRotate(const Matrix3&);   ///<sets the xform to just rotate
  inline void setTranslate(const Vector3&);///<sets the xform to just translate
  inline void setRotation(const Matrix3&);
  inline void setTranslation(const Vector3&);
  inline void setInverse(const RigidTransform&);
  inline void setRotated(const RigidTransform& a, const Matrix3& r);
  inline void setShifted(const RigidTransform& a, const Vector3& v);

  inline void get(RigidTransform&) const;
  inline void get(Matrix3&, Vector3&) const;
  inline void get(Vector3& x, Vector3& y, Vector3& z, Vector3& trans) const;
  inline void get(Matrix4&) const;
  inline void getRotation(Matrix3&) const;
  inline void getTranslation(Vector3&) const;
  inline void getInverse(RigidTransform&);

  inline void inplaceInverse();
  inline void inplaceRotate(const Matrix3&);
  inline void inplaceShift(const Vector3&);

  inline bool isIdentity(Real eps=Zero) const;
  bool isValid(Real eps=Epsilon) const;   ///<true if the matrix is orthogonal

  Matrix3 R;
  Vector3 t;
};

inline RigidTransform operator * (const RigidTransform&, const RigidTransform&);
inline RigidTransform operator * (const RigidTransform&, const Matrix3&);
inline RigidTransform operator * (const Matrix3& a, const RigidTransform& b);
inline Vector3 operator * (const RigidTransform&, const Vector3&);
inline RigidTransform operator + (const RigidTransform&, const Vector3&);
inline RigidTransform operator - (const RigidTransform&, const Vector3&);

/** @brief Same as above, but in 2D
 */
class RigidTransform2D
{
public:
  RigidTransform2D();
  RigidTransform2D(const RigidTransform2D& rhs);
  RigidTransform2D(const Matrix3& m);
  RigidTransform2D(const Matrix2& R,const Vector2& t);
  RigidTransform2D(Real theta,const Vector2& t);
  
  inline const RigidTransform2D& operator = (const RigidTransform2D&);
  inline void operator *= (const RigidTransform2D&);  //this(v) = this(a(v))
  
  inline void compose(const RigidTransform2D& a, const RigidTransform2D& b);
  inline void composeInverseA(const RigidTransform2D& a, const RigidTransform2D& b);
  inline void composeInverseB(const RigidTransform2D& a, const RigidTransform2D& b);
  inline void mul(const RigidTransform2D& a, const RigidTransform2D& b);
  inline void mulInverseA(const RigidTransform2D& a, const RigidTransform2D& b);
  inline void mulInverseB(const RigidTransform2D& a, const RigidTransform2D& b);
  
  inline void mul(const Vector3& a, Vector2& out) const;
  inline void mul(const Vector2& a, Vector2& out) const;
  inline void mulPoint(const Vector2& a, Vector2& out) const;
  inline void mulVector(const Vector2& a, Vector2& out) const;
  inline void mulInverse(const Vector2& a, Vector2& out) const;
  inline void mulInverse(const Vector3& a, Vector2& out) const;
  inline void mulPointInverse(const Vector2& a, Vector2& out) const;
  inline void mulVectorInverse(const Vector2& a, Vector2& out) const;

  inline void setIdentity();
  inline void set(const RigidTransform2D& rhs);
  inline void set(const Matrix3& mat);
  inline void set(Real theta,const Vector2& t);
  inline void setInverse(const RigidTransform2D&);
  inline void get(RigidTransform2D& rhs) const;
  inline void get(Matrix3& mat) const;
  inline void get(Real& theta,Vector2& t) const;

  inline bool isIdentity(Real eps=Zero) const;
  bool isValid(Real eps=Epsilon) const;  ///<true if the matrix is orthogonal  

  Matrix2 R;
  Vector2 t;
};

inline RigidTransform2D operator * (const RigidTransform2D&, const RigidTransform2D&);
inline Vector2 operator * (const RigidTransform2D&, const Vector2&);



//inlined member functions
inline bool Vector2::operator == (const Vector2& a) const { return a.x == x && a.y == y; }
inline bool Vector2::operator != (const Vector2& a) const { return a.x != x || a.y != y; }
inline const Vector2& Vector2::operator = (const Vector2& v) { set(v); return *this; }
inline void Vector2::operator += (const Vector2& v) { x += v.x; y += v.y; }
inline void Vector2::operator -= (const Vector2& v) { x -= v.x; y -= v.y; }
inline void Vector2::operator *= (Real c) { inplaceMul(c); }
inline void Vector2::operator /= (Real c) { inplaceDiv(c); }
inline Real& Vector2::operator [] (int i) { return data[i]; }
inline const Real& Vector2::operator [] (int i) const  { return data[i]; }
inline Vector2::operator Real* () { return data; }
inline Vector2::operator const Real* () const { return data; }
inline void Vector2::add(const Vector2& a, const Vector2& b) { x=a.x+b.x; y=a.y+b.y; }
inline void Vector2::sub(const Vector2& a, const Vector2& b) { x=a.x-b.x; y=a.y-b.y; }
inline void Vector2::mul(const Vector2& a, Real b) { x=a.x*b; y=a.y*b; }
inline void Vector2::div(const Vector2& a, Real b) { x=a.x/b; y=a.y/b; }
inline void Vector2::madd(const Vector2& a, Real b) { x+=a.x*b; y+=a.y*b; }
inline Real Vector2::dot(const Vector2& a) const { return x*a.x + y*a.y; }
inline Real Vector2::cross(const Vector2& a) const { return x*a.y - y*a.x; }
inline Real Vector2::distance(const Vector2& a) const { return Sqrt(distanceSquared(a)); }
inline Real Vector2::distanceSquared(const Vector2& a) const { return Sqr(x-a.x)+Sqr(y-a.y); }
inline Real Vector2::minElement(int* index) const {
  if(x <= y) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline Real Vector2::maxElement(int* index) const {
  if(x >= y) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline Real Vector2::minAbsElement(int* index) const {
  if(Abs(x) <= Abs(y)) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline Real Vector2::maxAbsElement(int* index) const {
  if(Abs(x) >= Abs(y)) { if(index) *index=0; return x; }
  else { if(index) *index=1; return y; }
}
inline Real Vector2::norm() const { return Sqrt(normSquared()); }
inline Real Vector2::normSquared() const { return Sqr(x)+Sqr(y); }
inline Real Vector2::length() const { return norm(); }
inline void Vector2::set(const Vector2& a) { set(a.x,a.y); }
inline void Vector2::set(Real _x) { x = y = _x; }
inline void Vector2::set(Real _x, Real _y) { x=_x; y=_y; }
inline void Vector2::set(const Real* _data) { if(_data) set(_data[0],_data[1]); else setZero(); }
inline void Vector2::setZero() { x = y = Zero; }
inline void Vector2::setRotation(Real rads) { set(Cos(rads), Sin(rads)); }
inline void Vector2::setPerpendicular(const Vector2& v) { set(-v.y, v.x); }
inline void Vector2::setNegative(const Vector2& a) { x=-a.x; y=-a.y; }
inline void Vector2::setNormalized(const Vector2& a) { mul(a,PseudoInv(a.norm())); }
inline void Vector2::setProjection(const Vector2& a, const Vector2& b) { mul(b, a.dot(b)/b.dot(b)); }
inline void Vector2::setMinimum(const Vector2& v) { if(v.x<x) x=v.x; if(v.y<y) y=v.y; }
inline void Vector2::setMinimum(const Vector2& a,const Vector2& b) { set(Min(a.x,b.x),Min(a.y,b.y)); }
inline void Vector2::setMaximum(const Vector2& v) { if(v.x>x) x=v.x; if(v.y>y) y=v.y; }
inline void Vector2::setMaximum(const Vector2& a,const Vector2& b) { set(Max(a.x,b.x),Max(a.y,b.y)); }
inline void Vector2::get(Vector2& v) const { get(v.x,v.y); }
inline void Vector2::get(Real& _x, Real& _y) const { _x=x; _y=y; }
inline void Vector2::get(Real _data[2]) const { get(_data[0],_data[1]); }
inline void Vector2::getNegative(Vector2& v) const { v.setNegative(*this); }
inline void Vector2::getNormalized(Vector2& v) const  { v.setNormalized(*this); }
inline void Vector2::getOrthogonal(Vector2& v) const { v.setOrthogonal(*this); }
inline void Vector2::inplaceNegative() { x=-x; y=-y; }
inline void Vector2::inplaceMul(Real c) { x*=c; y*=c; }
inline void Vector2::inplaceDiv(Real c) { x/=c; y/=c; }
inline void Vector2::inplaceNormalize() { inplaceMul(PseudoInv(norm())); }
inline bool Vector2::isZero(Real eps) const { return FuzzyZero(x,eps)&&FuzzyZero(y,eps); }
inline bool Vector2::isEqual(const Vector2&a,Real eps) const { return FuzzyEquals(x,a.x,eps) && FuzzyEquals(y,a.y,eps); }

inline bool Vector3::operator == (const Vector3& a) const { return a.x == x && a.y == y && a.z == z; }
inline bool Vector3::operator != (const Vector3& a) const { return a.x != x || a.y != y || a.z != z; }
inline const Vector3& Vector3::operator = (const Vector3& v) { set(v); return *this; }
inline void Vector3::operator += (const Vector3& v) { x += v.x; y += v.y; z += v.z; }
inline void Vector3::operator -= (const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; }
inline void Vector3::operator *= (Real c) { inplaceMul(c); }
inline void Vector3::operator /= (Real c) { inplaceDiv(c); }
inline Real& Vector3::operator [] (int i) { return data[i]; }
inline const Real& Vector3::operator [] (int i) const  { return data[i]; }
inline Vector3::operator Real* () { return data; }
inline Vector3::operator const Real* () const { return data; }
inline void Vector3::add(const Vector3& a, const Vector3& b) { x=a.x+b.x; y=a.y+b.y; z=a.z+b.z; }
inline void Vector3::sub(const Vector3& a, const Vector3& b) { x=a.x-b.x; y=a.y-b.y; z=a.z-b.z; }
inline void Vector3::mul(const Vector3& a, Real b) { x=a.x*b; y=a.y*b; z=a.z*b; }
inline void Vector3::div(const Vector3& a, Real b) { x=a.x/b; y=a.y/b; z=a.z/b; }
inline void Vector3::madd(const Vector3& a, Real b) { x+=a.x*b; y+=a.y*b; z+=a.z*b; }
inline Real Vector3::dot(const Vector3& a) const { return x*a.x + y*a.y + z*a.z; }
inline Real Vector3::distance(const Vector3& a) const { return Sqrt(distanceSquared(a)); }
inline Real Vector3::distanceSquared(const Vector3& a) const { return Sqr(x-a.x)+Sqr(y-a.y)+Sqr(z-a.z); }
inline Real Vector3::norm() const { return Sqrt(normSquared()); }
inline Real Vector3::normSquared() const { return Sqr(x)+Sqr(y)+Sqr(z); }
inline Real Vector3::length() const { return norm(); }
inline Real Vector3::minElement(int* index) const
{
  Real vmin=x;
  int imin=0;
  if(y < vmin) { vmin=y; imin=1; }
  if(z < vmin) { vmin=z; imin=2; }
  if(index) *index=imin;
  return vmin;
}
inline Real Vector3::maxElement(int* index) const
{
  Real vmax=x;
  int imin=0;
  if(y > vmax) { vmax=y; imin=1; }
  if(z > vmax) { vmax=z; imin=2; }
  if(index) *index=imin;
  return vmax;
}
inline Real Vector3::minAbsElement(int* index) const
{
  Real vmin=Abs(x);
  int imin=0;
  if(Abs(y) < vmin) { vmin=Abs(y); imin=1; }
  if(Abs(z) < vmin) { vmin=Abs(z); imin=2; }
  if(index) *index=imin;
  return vmin;
}
inline Real Vector3::maxAbsElement(int* index) const
{
  Real vmax=Abs(x);
  int imin=0;
  if(Abs(y) > vmax) { vmax=Abs(y); imin=1; }
  if(Abs(z) > vmax) { vmax=Abs(z); imin=2; }
  if(index) *index=imin;
  return vmax;
}
inline void Vector3::set(const Vector3& a) { set(a.x,a.y,a.z); }
inline void Vector3::set(Real _x) { x = y = z = _x; }
inline void Vector3::set(Real _x, Real _y, Real _z) { x=_x; y=_y; z=_z; }
inline void Vector3::set(const Real *_data) { if(_data) set(_data[0],_data[1],_data[2]); else setZero(); }
inline void Vector3::setZero() { x = y = z = Zero; }
inline void Vector3::setNegative(const Vector3& a) { x=-a.x; y=-a.y; z=-a.z; }
inline void Vector3::setCross(const Vector3& a, const Vector3& b) { x=a.y*b.z-a.z*b.y; y=a.z*b.x-a.x*b.z; z=a.x*b.y-a.y*b.x; }
inline void Vector3::setNormalized(const Vector3& a) { mul(a,PseudoInv(a.norm())); }
inline void Vector3::setProjection(const Vector3& a, const Vector3& b) { mul(b, a.dot(b)/b.dot(b)); }
inline void Vector3::setMinimum(const Vector3& v) { if(v.x<x) x=v.x; if(v.y<y) y=v.y; if(v.z<z) z=v.z; }
inline void Vector3::setMinimum(const Vector3& a,const Vector3& b) { set(Min(a.x,b.x),Min(a.y,b.y),Min(a.z,b.z)); }
inline void Vector3::setMaximum(const Vector3& v) { if(v.x>x) x=v.x; if(v.y>y) y=v.y; if(v.z>z) z=v.z; }
inline void Vector3::setMaximum(const Vector3& a,const Vector3& b) { set(Max(a.x,b.x),Max(a.y,b.y),Max(a.z,b.z)); }
inline void Vector3::get(Vector3& v) const { get(v.x,v.y,v.z); }
inline void Vector3::get(Real& _x, Real& _y, Real& _z) const { _x=x; _y=y; _z=z; }
inline void Vector3::get(Real _data[3]) const { get(_data[0],_data[1],_data[2]); }
inline void Vector3::getNegative(Vector3& v) const { v.setNegative(*this); }
inline void Vector3::getNormalized(Vector3& v) const  { v.setNormalized(*this); }
inline void Vector3::getOrthogonalBasis(Vector3& yb,Vector3& zb) const 
{
    Real scale;
    Real n = normSquared(),invn = 1.0;
    if(FuzzyZero(n)) {
      yb.set(0.0,1.0,0.0);
      zb.set(0.0,0.0,1.0);
      return;
    }
    if(!FuzzyEquals(n,1.0)) {
      n = Sqrt(n);
      invn  = 1.0/n;
    }
    if(FuzzyEquals(x,n)) scale = 0;
    else if(FuzzyEquals(x,-n)) {  //A complete flip of the basis
      yb.set(0.0,-1.0,0.0);
      zb.set(0.0,0.0,1.0);
      return;
    }
    else scale = n*(n-x)/(Sqr(n)-Sqr(x));
    yb.x = -y;
    yb.y = x + scale*Sqr(z);
    yb.z = -scale*y*z;
    zb.x = -z;
    zb.y = -scale*y*z;
    zb.z = x + scale*Sqr(y);
    if(invn != 1.0) {
      yb *= invn;
      zb *= invn;
    }
}
inline void Vector3::inplaceNegative() { x=-x; y=-y; z=-z; }
inline void Vector3::inplaceMul(Real c) { x*=c; y*=c; z*=c; }
inline void Vector3::inplaceDiv(Real c) { x/=c; y/=c; z/=c; }
inline void Vector3::inplaceNormalize() { inplaceMul(PseudoInv(norm())); }
inline bool Vector3::isZero(Real eps) const { return FuzzyZero(x,eps)&&FuzzyZero(y,eps)&&FuzzyZero(z,eps); }
inline bool Vector3::isEqual(const Vector3&a,Real eps) const { return FuzzyEquals(x,a.x,eps) && FuzzyEquals(y,a.y,eps) && FuzzyEquals(z,a.z,eps); }

inline bool Vector4::operator == (const Vector4& a) const { return a.x == x && a.y == y && a.z == z && a.w == w; }
inline bool Vector4::operator != (const Vector4& a) const { return a.x != x || a.y != y || a.z != z || a.w != w; }
inline const Vector4& Vector4::operator = (const Vector4& v) { set(v); return *this; }
inline void Vector4::operator += (const Vector4& v) { x += v.x; y += v.y; z += v.z; w += v.w; }
inline void Vector4::operator -= (const Vector4& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; }
inline void Vector4::operator *= (Real c) { inplaceMul(c); }
inline void Vector4::operator /= (Real c) { inplaceDiv(c); }
inline void Vector4::operator += (const Vector3& v) { x += v.x; y += v.y; z += v.z; }
inline void Vector4::operator -= (const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; }
inline Real& Vector4::operator [] (int i) { return data[i]; }
inline const Real& Vector4::operator [] (int i) const  { return data[i]; }
inline Vector4::operator Vector3() const { return Vector3(x,y,z); }
inline Vector4::operator Real* () { return data; }
inline Vector4::operator const Real* () const { return data; }
inline void Vector4::add(const Vector4& a, const Vector4& b) { x=a.x+b.x; y=a.y+b.y; z=a.z+b.z; w=a.w+b.w; }
inline void Vector4::sub(const Vector4& a, const Vector4& b) { x=a.x-b.x; y=a.y-b.y; z=a.z-b.z; w=a.w-b.w; }
inline void Vector4::mul(const Vector4& a, Real b) { x=a.x*b; y=a.y*b; z=a.z*b; w=a.w*b; }
inline void Vector4::div(const Vector4& a, Real b) { x=a.x/b; y=a.y/b; z=a.z/b; w=a.w/b; }
inline void Vector4::madd(const Vector4& a, Real b) { x+=a.x*b; y+=a.y*b; z+=a.z*b; w+=a.w*b; }
inline Real Vector4::dot(const Vector4& a) const { return x*a.x + y*a.y + z*a.z + w*a.w; }
inline Real Vector4::dot3(const Vector4& a) const { return x*a.x + y*a.y + z*a.z; }
inline Real Vector4::dot3(const Vector3& a) const { return x*a.x + y*a.y + z*a.z; }
inline Real Vector4::distance(const Vector4& a) const { return Sqrt(distanceSquared(a)); }
inline Real Vector4::distanceSquared(const Vector4& a) const { return Sqr(x-a.x)+Sqr(y-a.y)+Sqr(z-a.z)+Sqr(w-a.w); }
inline Real Vector4::norm() const { return Sqrt(normSquared()); }
inline Real Vector4::normSquared() const { return Sqr(x)+Sqr(y)+Sqr(z)+Sqr(w); }
inline Real Vector4::length() const { return norm(); }
inline Real Vector4::minElement(int* index) const
{
  Real vmin=x;
  int imin=0;
  if(y < vmin) { vmin=y; imin=1; }
  if(z < vmin) { vmin=z; imin=2; }
  if(w < vmin) { vmin=w; imin=3; }
  if(index) *index=imin;
  return vmin;
}
inline Real Vector4::maxElement(int* index) const
{
  Real vmax=x;
  int imin=0;
  if(y > vmax) { vmax=y; imin=1; }
  if(z > vmax) { vmax=z; imin=2; }
  if(w > vmax) { vmax=w; imin=3; }
  if(index) *index=imin;
  return vmax;
}
inline Real Vector4::minAbsElement(int* index) const
{
  Real vmin=Abs(x);
  int imin=0;
  if(Abs(y) < vmin) { vmin=Abs(y); imin=1; }
  if(Abs(z) < vmin) { vmin=Abs(z); imin=2; }
  if(Abs(w) < vmin) { vmin=Abs(w); imin=3; }
  if(index) *index=imin;
  return vmin;
}
inline Real Vector4::maxAbsElement(int* index) const
{
  Real vmax=Abs(x);
  int imin=0;
  if(Abs(y) > vmax) { vmax=Abs(y); imin=1; }
  if(Abs(z) > vmax) { vmax=Abs(z); imin=2; }
  if(Abs(w) > vmax) { vmax=Abs(w); imin=3; }
  if(index) *index=imin;
  return vmax;
}
inline void Vector4::set(const Vector4& a) { x = a.x; y = a.y; z = a.z; w = a.w; }
inline void Vector4::set(Real _x) { x = y = z = w = _x; }
inline void Vector4::set(Real _x, Real _y, Real _z, Real _w) { x=_x; y=_y; z=_z; w=_w; }
inline void Vector4::set(const Real *_data)  { if(_data) set(_data[0],_data[1],_data[2],_data[3]); else setZero(); }
inline void Vector4::setZero() { x = y = z = w = Zero; }
inline void Vector4::setVector(const Vector3& v) { set(v.x,v.y,v.z,Zero); }
inline void Vector4::setHomogeneous(const Vector3& v) { set(v.x,v.y,v.z,One); }
inline void Vector4::setNegative(const Vector4& v) { set(-v.x, -v.y, -v.z, -v.w); }
inline void Vector4::setNormalized(const Vector4& v) { mul(v, PseudoInv(v.norm())); }
inline void Vector4::setProjection(const Vector4& a, const Vector4& b) { mul(b, a.dot(b)/b.dot(b)); }
inline void Vector4::setMinimum(const Vector4& v) { if(v.x<x) x=v.x; if(v.y<y) y=v.y; if(v.z<z) z=v.z; if(v.w<w) w=v.w; }
inline void Vector4::setMinimum(const Vector4& a,const Vector4& b) { set(Min(a.x,b.x),Min(a.y,b.y),Min(a.z,b.z),Min(a.w,b.w)); }
inline void Vector4::setMaximum(const Vector4& v) { if(v.x>x) x=v.x; if(v.y>y) y=v.y; if(v.z>z) z=v.z; if(v.w>w) w=v.w; }
inline void Vector4::setMaximum(const Vector4& a,const Vector4& b) { set(Max(a.x,b.x),Max(a.y,b.y),Max(a.z,b.z),Max(a.w,b.w)); }
inline void Vector4::get(Vector4& v) const { get(v.x,v.y,v.z,v.w); }
inline void Vector4::get(Real& _x, Real& _y, Real& _z, Real& _w) const { _x=x; _y=y; _z=z; _w=w; }
inline void Vector4::get(Real _data[4]) const { get(_data[0],_data[1],_data[2],_data[3]); }
inline void Vector4::get(Vector3& v) const { v.x=x; v.y=y; v.z=z; }
inline void Vector4::getNegative(Vector4& v) const { v.setNegative(*this); }
inline void Vector4::getNormalized(Vector4& v) const { v.setNormalized(*this); }
inline void Vector4::inplaceNegative() { x=-x; y=-y; z=-z; w=-w; }
inline void Vector4::inplaceMul(Real c) { x*=c; y*=c; z*=c; w*=c; }
inline void Vector4::inplaceDiv(Real c) { x/=c; y/=c; z/=c; w/=c; }
inline void Vector4::inplaceNormalize() { inplaceMul(PseudoInv(norm())); }
inline bool Vector4::isZero(Real eps) const { return FuzzyZero(x,eps)&&FuzzyZero(y,eps)&&FuzzyZero(z,eps)&&FuzzyZero(w,eps); }
inline bool Vector4::isEqual(const Vector4&a,Real eps) const { return FuzzyEquals(x,a.x,eps) && FuzzyEquals(y,a.y,eps) && FuzzyEquals(z,a.z,eps) && FuzzyEquals(w,a.w,eps); }



inline const Matrix2& Matrix2::operator  = (const Matrix2& m)
{
  set(m);
  return *this;
}

inline void Matrix2::operator += (const Matrix2& m)
{
  int i,j;
  for(i=0; i<2; i++)
  {
      for(j=0; j<2; j++)
		data[i][j] += m.data[i][j];
  }
}

inline void Matrix2::operator -= (const Matrix2& m)
{
  int i,j;
  for(i=0; i<2; i++)
  {
      for(j=0; j<2; j++)
		data[i][j] -= m.data[i][j];
  }
}

inline void Matrix2::operator *= (const Matrix2& m)
{
  mul(*this, m);
}

inline void Matrix2::operator *= (Real s)
{
  inplaceMul(s);
}

inline void Matrix2::operator /= (Real s)
{
  inplaceDiv(s);
}

inline void Matrix2::add(const Matrix2& a, const Matrix2& b)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = a.data[i][j] + b.data[i][j];
}

inline void Matrix2::sub(const Matrix2& a, const Matrix2& b)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = a.data[i][j] - b.data[i][j];
}

inline void Matrix2::mul(const Matrix2& a, const Matrix2& b)
{
  int i,j;
  Real dat [2][2];
  //NOTE: remember column major
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      dat[i][j] = b.data[i][0]*a.data[0][j] + b.data[i][1]*a.data[1][j];
  set(dat);
}

inline void Matrix2::mulTransposeA(const Matrix2& a, const Matrix2& b)
{
  int i,j;
  Real dat [2][2];
  //NOTE: remember column major
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      dat[i][j] = b.data[i][0]*a.data[j][0] + b.data[i][1]*a.data[j][1];
  set(dat);
}

inline void Matrix2::mulTransposeB(const Matrix2& a, const Matrix2& b)
{
  int i,j;
  Real dat [2][2];
  //NOTE: remember column major
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      dat[i][j] = b.data[0][i]*a.data[0][j] + b.data[1][i]*a.data[1][j];
  set(dat);
}

inline void Matrix2::mul(const Matrix2& a, Real b)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = a.data[i][j]*b;
}

inline void Matrix2::div(const Matrix2& a, Real b)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = a.data[i][j]/b;
}

inline void Matrix2::mul(const Vector2& a, Vector2& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y;
  out.y = data[0][1]*a.x + data[1][1]*a.y;
}

inline void Matrix2::mulTranspose(const Vector2& a, Vector2& out) const
{
  out.x = data[0][0]*a.x + data[0][1]*a.y;
  out.y = data[1][0]*a.x + data[1][1]*a.y;
}

inline void Matrix2::set(const Matrix2& m)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = m.data[i][j];
}

inline void Matrix2::set(Real x)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = x;
}

inline void Matrix2::set(const Real m[2][2])
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = m[i][j];
}

inline void Matrix2::set(const Real* m)
{
  if(!m) {
    setZero();
    return;
  }
  int i,j,k=0;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++, k++)
      data[i][j] = m[k];
}

inline void Matrix2::set(const Vector2& xb, const Vector2& yb)
{
  setCol1(xb);
  setCol2(yb);
}

inline void Matrix2::setZero()
{
  set(Zero);
}

inline void Matrix2::setIdentity()
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = Delta(i,j);
}

inline void Matrix2::setDiagonal(const Vector2& d)
{
  setZero();
  d.get(data[0][0],data[1][1]);
}

inline void Matrix2::setTranspose(const Matrix2& m)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = m.data[j][i];
}

inline void Matrix2::setNegative(const Matrix2& m)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = -m.data[i][j];
}

inline bool Matrix2::setInverse(const Matrix2& m)
{
  Real det = m.determinant();
  if(det == Zero) return false;
  Real detinv = Inv(det);
  data[0][0] = m.data[1][1];
  data[1][1] = m.data[0][0];
  data[0][1] = -m.data[0][1];
  data[1][0] = -m.data[1][0];
  inplaceMul(detinv);
  return true;
}

inline void Matrix2::setScale(Real s)
{
  setZero();
  data[0][0] = data[1][1] = s;
}

inline void Matrix2::setScale(Real sx, Real sy)
{
  setZero();
  data[0][0] = sx;
  data[1][1] = sy;
}

inline void Matrix2::setRotate(Real rads)
{
  Real cr = Cos(rads);
  Real sr = Sin(rads);

  data[0][0] = cr;  data[1][0] = -sr;
  data[0][1] = sr;  data[1][1] = cr;
}

inline void Matrix2::setOuterProduct(const Vector2& a,const Vector2& b)
{
  data[0][0] = a.x*b.x; data[1][0] = a.x*b.y;
  data[0][1] = a.y*b.x; data[1][1] = a.y*b.y;
}


inline void Matrix2::get(Matrix2& m) const
{
  m.set(*this);
}

inline void Matrix2::get(Real m [2][2]) const
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      m[i][j] = data[i][j];
}

inline void Matrix2::get(Real m [4]) const
{
  int i,j,k=0;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++, k++)
      m[k] = data[i][j];
}

inline void Matrix2::get(Vector2& xb, Vector2& yb) const
{
  getCol1(xb);
  getCol2(yb);
}

inline void Matrix2::getTranspose(Matrix2& m) const
{
  m.setTranspose(*this);
}

inline void Matrix2::getNegative(Matrix2& m) const
{
  m.setNegative(*this);
}

inline bool Matrix2::getInverse(Matrix2& m) const
{
  return m.setInverse(*this);
}

inline void Matrix2::inplaceTranspose()
{
  Real temp;
  int i,j;
  for(i=0; i<2; i++) {
    for(j=0; j<i; j++) {
      temp = data[i][j];
      data[i][j] = data[j][i];
      data[j][i] = temp;
    }
  }
}

inline void Matrix2::inplaceNegative()
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] = -data[i][j];
}

inline void Matrix2::inplaceMul(Real s)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] *= s;	
}

inline void Matrix2::inplaceDiv(Real s)
{
  int i,j;
  for(i=0; i<2; i++)
    for(j=0; j<2; j++)
      data[i][j] /= s;	
}

inline void Matrix2::inplaceRowScale(Real sx, Real sy)
{
  int j;
  for(j=0; j<2; j++) {
    data[0][j] *= sx;
    data[1][j] *= sy;
  }
}

inline void Matrix2::inplaceColScale(Real sx, Real sy)
{
  int j;
  for(j=0; j<2; j++) {
    data[j][0] *= sx;
    data[j][1] *= sy;
  }
}

inline bool Matrix2::inplaceInverse()
{
  Matrix2 tmp = *this;
  return setInverse(tmp);
}

inline bool Matrix2::isZero(Real eps) const
{
  return FuzzyZero(data[0][0],eps)
    && FuzzyZero(data[0][1],eps)
    && FuzzyZero(data[1][0],eps)
    && FuzzyZero(data[1][1],eps);
}

inline bool Matrix2::isEqual(const Matrix2&a,Real eps) const
{
  return FuzzyEquals(data[0][0],a.data[0][0],eps) 
    && FuzzyEquals(data[0][1],a.data[0][1],eps)
    && FuzzyEquals(data[1][0],a.data[1][0],eps)
    && FuzzyEquals(data[1][1],a.data[1][1],eps);
}

inline bool Matrix2::isIdentity(Real eps) const
{
  return FuzzyEquals(data[0][0],One,eps)
    && FuzzyZero(data[0][1],eps)
    && FuzzyZero(data[1][0],eps)
    && FuzzyEquals(data[1][1],One,eps);
}

inline bool Matrix2::isInvertible(Real eps) const
{
  return !FuzzyZero(determinant(),eps);
}

inline Real Matrix2::trace() const { return data[0][0] + data[1][1]; }

inline Real Matrix2::determinant() const
{
	return data[0][0]*data[1][1]-data[0][1]*data[1][0];
}

inline Real Matrix2::minElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(data[i][j] < vmin) {  //recall data is stored column-major
	vmin=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix2::maxElement(int* _i,int* _j) const
{
  Real vmax=-Inf;
  int imin=0,jmin=0;
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(data[i][j] > vmax) {  //recall data is stored column-major
	vmax=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}

Real Matrix2::minAbsElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(Abs(data[i][j]) < vmin) {  //recall data is stored column-major
	vmin=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix2::maxAbsElement(int* _i,int* _j) const
{
  Real vmax=0;
  int imin=0,jmin=0;
  for(int i=0;i<2;i++)
    for(int j=0;j<2;j++) 
      if(Abs(data[i][j]) > vmax) {  //recall data is stored column-major
	vmax=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}




inline const Matrix3& Matrix3::operator  = (const Matrix3& m)
{
  set(m);
  return *this;
}

inline void Matrix3::operator += (const Matrix3& m)
{
  int i,j;
  for(i=0; i<3; i++)
    {
      for(j=0; j<3; j++)
	data[i][j] += m.data[i][j];
    }
}

inline void Matrix3::operator -= (const Matrix3& m)
{
  int i,j;
  for(i=0; i<3; i++)
    {
      for(j=0; j<3; j++)
	data[i][j] -= m.data[i][j];
    }
}

inline void Matrix3::operator *= (const Matrix3& m)
{
  mul(*this, m);
}

inline void Matrix3::operator *= (Real s)
{
  inplaceMul(s);
}

inline void Matrix3::operator /= (Real s)
{
  inplaceDiv(s);
}

inline void Matrix3::add(const Matrix3& a, const Matrix3& b)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = a.data[i][j] + b.data[i][j];
}

inline void Matrix3::sub(const Matrix3& a, const Matrix3& b)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = a.data[i][j] - b.data[i][j];
}

inline void Matrix3::mul(const Matrix3& a, Real b)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = a.data[i][j]*b;
}

inline void Matrix3::div(const Matrix3& a, Real b)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = a.data[i][j]/b;
}

inline void Matrix3::mul(const Vector3& a, Vector3& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y + data[2][0]*a.z;
  out.y = data[0][1]*a.x + data[1][1]*a.y + data[2][1]*a.z;
  out.z = data[0][2]*a.x + data[1][2]*a.y + data[2][2]*a.z;
}

inline void Matrix3::mulTranspose(const Vector3& a, Vector3& out) const
{
  out.x = data[0][0]*a.x + data[0][1]*a.y + data[0][2]*a.z;
  out.y = data[1][0]*a.x + data[1][1]*a.y + data[1][2]*a.z;
  out.z = data[2][0]*a.x + data[2][1]*a.y + data[2][2]*a.z;
}

inline void Matrix3::mulPoint(const Vector2& a, Vector2& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y + data[2][0];
  out.y = data[0][1]*a.x + data[1][1]*a.y + data[2][1];
}

inline void Matrix3::mulVector(const Vector2& a, Vector2& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y;
  out.y = data[0][1]*a.x + data[1][1]*a.y;
}

inline void Matrix3::mulVectorTranspose(const Vector2& a, Vector2& out) const
{
  out.x = data[0][0]*a.x + data[0][1]*a.y;
  out.y = data[1][0]*a.x + data[1][1]*a.y;
}

inline void Matrix3::set(const Matrix3& m)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = m.data[i][j];
}

inline void Matrix3::set(Real x)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = x;
}

inline void Matrix3::set(const Real m[3][3])
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = m[i][j];
}

inline void Matrix3::set(const Real* m)
{
  if(!m) {
    setZero();
    return;
  }
  int i,j,k=0;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++, k++)
      data[i][j] = m[k];
}

inline void Matrix3::set(const Vector3& xb, const Vector3& yb, const Vector3& zb)
{
  setCol1(xb);
  setCol2(yb);
  setCol3(zb);
}

inline void Matrix3::setZero()
{
  set(Zero);
}

inline void Matrix3::setIdentity()
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = Delta(i,j);
}

inline void Matrix3::setDiagonal(const Vector3& d)
{
  setZero();
  d.get(data[0][0],data[1][1],data[2][2]);
}

inline void Matrix3::setTranspose(const Matrix3& m)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = m.data[j][i];
}

inline void Matrix3::setNegative(const Matrix3& m)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = -m.data[i][j];
}

inline void Matrix3::setScale(Real s)
{
  setZero();
  data[0][0] = data[1][1] = data[2][2] = s;
}

inline void Matrix3::setScale(Real sx, Real sy, Real sz)
{
  setZero();
  data[0][0] = sx;
  data[1][1] = sy;
  data[2][2] = sz;
}

inline void Matrix3::setRotateX(Real rads)
{
  Real cr = Cos(rads);
  Real sr = Sin(rads);
  setIdentity();
  data[1][1] = cr;  data[2][1] = -sr;
  data[1][2] = sr;  data[2][2] = cr;
}

inline void Matrix3::setRotateY(Real rads)
{
  Real cr = Cos(rads);
  Real sr = Sin(rads);
  setIdentity();
  data[0][0] = cr;  data[2][0] = sr;
  data[0][2] = -sr;  data[2][2] = cr;
}

inline void Matrix3::setRotateZ(Real rads)
{
  Real cr = Cos(rads);
  Real sr = Sin(rads);
  setIdentity();
  data[0][0] = cr;  data[1][0] = -sr;
  data[0][1] = sr;  data[1][1] = cr;
}

inline void Matrix3::setCrossProduct(const Vector3& v)
{
  data[1][0] = -v[2];
  data[2][1] = -v[0];
  data[2][0] = v[1];

  data[0][1] = v[2];
  data[1][2] = v[0];
  data[0][2] = -v[1];

  data[0][0] = data[1][1] = data[2][2] = Zero;
}

inline void Matrix3::setOuterProduct(const Vector3& a,const Vector3& b)
{
  data[0][0] = a.x*b.x; data[1][0] = a.x*b.y; data[2][0] = a.x*b.z;
  data[0][1] = a.y*b.x; data[1][1] = a.y*b.y; data[2][1] = a.y*b.z;
  data[0][2] = a.z*b.x; data[1][2] = a.z*b.y; data[2][2] = a.z*b.z;
}

inline void Matrix3::get(Matrix3& m) const
{
  m.set(*this);
}

inline void Matrix3::get(Real m [3][3]) const
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      m[i][j] = data[i][j];
}

inline void Matrix3::get(Real m [9]) const
{
  int i,j,k=0;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++, k++)
      m[k] = data[i][j];
}

inline void Matrix3::get(Vector3& xb, Vector3& yb, Vector3& zb) const
{
  getCol1(xb);
  getCol2(yb);
  getCol3(zb);
}

inline void Matrix3::getTranspose(Matrix3& m) const
{
  m.setTranspose(*this);
}

inline void Matrix3::getNegative(Matrix3& m) const
{
  m.setNegative(*this);
}

inline bool Matrix3::getInverse(Matrix3& m) const
{
  return m.setInverse(*this);
}

inline void Matrix3::getCrossProduct(Vector3& v) const
{
  v[0] = Half*(data[1][2]-data[2][1]);
  v[1] = Half*(data[2][0]-data[0][2]);
  v[2] = Half*(data[0][1]-data[1][0]);
}

inline void Matrix3::inplaceTranspose()
{
  Real temp;
  int i,j;
  for(i=0; i<3; i++) {
    for(j=0; j<i; j++) {
      temp = data[i][j];
      data[i][j] = data[j][i];
      data[j][i] = temp;
    }
  }
}

inline void Matrix3::inplaceNegative()
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = -data[i][j];
}

inline void Matrix3::inplaceMul(Real s)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] *= s;	
}

inline void Matrix3::inplaceDiv(Real s)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] /= s;	
}

inline void Matrix3::inplaceRowScale(Real sx, Real sy, Real sz)
{
  int j;
  for(j=0; j<3; j++) {
    data[0][j] *= sx;
    data[1][j] *= sy;
    data[2][j] *= sz;
  }
}

inline void Matrix3::inplaceColScale(Real sx, Real sy, Real sz)
{
  int j;
  for(j=0; j<3; j++) {
    data[j][0] *= sx;
    data[j][1] *= sy;
    data[j][2] *= sz;
  }
}

inline bool Matrix3::inplaceInverse()
{
  Matrix3 tmp = *this;
  return setInverse(tmp);
}

inline bool Matrix3::isZero(Real eps) const
{
  int i,j;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      if(!FuzzyZero(data[i][j],eps)) return false;
  return true;
}

inline bool Matrix3::isEqual(const Matrix3& a,Real eps) const
{
  int i,j;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      if(!FuzzyEquals(data[i][j],a.data[i][j],eps)) return false;
  return true;
}

inline bool Matrix3::isIdentity(Real eps) const
{
  int i,j;
  for(i=0;i<3;i++)
    for(j=0;j<3;j++)
      if(!FuzzyEquals(data[i][j],Delta(i,j),eps)) return false;
  return true;
}

inline bool Matrix3::isInvertible(Real eps) const
{
  return !FuzzyEquals(determinant(),eps);
}

inline Real Matrix3::trace() const { return data[0][0] + data[1][1] + data[2][2]; }

inline Real Matrix3::minElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(data[i][j] < vmin) {  //recall data is stored column-major
	vmin=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix3::maxElement(int* _i,int* _j) const
{
  Real vmax=-Inf;
  int imin=0,jmin=0;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(data[i][j] > vmax) {  //recall data is stored column-major
	vmax=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}

Real Matrix3::minAbsElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(Abs(data[i][j]) < vmin) {  //recall data is stored column-major
	vmin=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix3::maxAbsElement(int* _i,int* _j) const
{
  Real vmax=0;
  int imin=0,jmin=0;
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++) 
      if(Abs(data[i][j]) > vmax) {  //recall data is stored column-major
	vmax=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}




inline const Matrix4& Matrix4::operator  = (const Matrix4& m)
{
  set(m);
  return *this;
}

inline void Matrix4::operator += (const Matrix4& m)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] += m.data[i][j];
  }
}

inline void Matrix4::operator -= (const Matrix4& m)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] -= m.data[i][j];
  }
}

inline void Matrix4::operator *= (const Matrix4& m)
{
  Matrix4 tmp(*this);
  mul(tmp, m);
}

inline void Matrix4::operator *= (Real s)
{
  inplaceMul(s);
}

inline void Matrix4::operator /= (Real s)
{
  inplaceDiv(s);
}


inline void Matrix4::add(const Matrix4& a, const Matrix4& b)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] = a.data[i][j] + b.data[i][j];
  }
}

inline void Matrix4::sub(const Matrix4& a, const Matrix4& b)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] = a.data[i][j] - b.data[i][j];
  }
}

inline void Matrix4::mul(const Matrix4& a, Real b)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] = a.data[i][j]*b;
  }
}

inline void Matrix4::div(const Matrix4& a, Real b)
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++)
      data[i][j] = a.data[i][j]/b;
  }
}

inline void Matrix4::mul(const Vector4& a, Vector4& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y + data[2][0]*a.z + data[3][0]*a.w;
  out.y = data[0][1]*a.x + data[1][1]*a.y + data[2][1]*a.z + data[3][1]*a.w;
  out.z = data[0][2]*a.x + data[1][2]*a.y + data[2][2]*a.z + data[3][2]*a.w;
  out.w = data[0][3]*a.x + data[1][3]*a.y + data[2][3]*a.z + data[3][3]*a.w;
}

inline void Matrix4::mulTranspose(const Vector4& a, Vector4& out) const
{
  out.x = data[0][0]*a.x + data[0][1]*a.y + data[0][2]*a.z + data[0][3]*a.w;
  out.y = data[1][0]*a.x + data[1][1]*a.y + data[1][2]*a.z + data[1][3]*a.w;
  out.z = data[2][0]*a.x + data[2][1]*a.y + data[2][2]*a.z + data[2][3]*a.w;
  out.w = data[3][0]*a.x + data[3][1]*a.y + data[3][2]*a.z + data[3][3]*a.w;
}

inline void Matrix4::mulPoint(const Vector3& a, Vector3& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y + data[2][0]*a.z + data[3][0];
  out.y = data[0][1]*a.x + data[1][1]*a.y + data[2][1]*a.z + data[3][1];
  out.z = data[0][2]*a.x + data[1][2]*a.y + data[2][2]*a.z + data[3][2];
}

inline void Matrix4::mulVector(const Vector3& a, Vector3& out) const
{
  out.x = data[0][0]*a.x + data[1][0]*a.y + data[2][0]*a.z;
  out.y = data[0][1]*a.x + data[1][1]*a.y + data[2][1]*a.z;
  out.z = data[0][2]*a.x + data[1][2]*a.y + data[2][2]*a.z;
}

inline void Matrix4::mulVectorTranspose(const Vector3& a, Vector3& out) const
{
  out.x = data[0][0]*a.x + data[0][1]*a.y + data[0][2]*a.z;
  out.y = data[1][0]*a.x + data[1][1]*a.y + data[1][2]*a.z;
  out.z = data[2][0]*a.x + data[2][1]*a.y + data[2][2]*a.z;
}

inline void Matrix4::set(const Matrix4& m)
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = m.data[i][j];
}

inline void Matrix4::set(Real x)
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = x;
}

inline void Matrix4::set(const Real m [4][4])
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = m[i][j];
}

inline void Matrix4::set(const Real* m)
{
  if(!m) {
    setZero();
    return;
  }
  int i,j,k=0;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++, k++)
      data[i][j] = m[k];

}

inline void Matrix4::set(const Vector3& xb, const Vector3& yb, const Vector3& zb, const Vector3& trans)
{
  data[0][3] = data[1][3] = data[2][3] = Zero;
  data[3][3] = One;
  setCol1(xb);
  setCol2(yb);
  setCol3(zb);
  setCol4(trans);
}

inline void Matrix4::set(const Vector4& x, const Vector4& y, const Vector4& z, const Vector4& w)
{
  setCol1(x);
  setCol2(y);
  setCol3(z);
  setCol4(w);
}

inline void Matrix4::set(const Matrix3& m)
{
  data[0][3] = data[1][3] = data[2][3] = Zero;
  data[3][3] = One;
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = m.data[i][j];
  data[3][0] = data[3][1] = data[3][2] = Zero;
}

inline void Matrix4::set(const Matrix3& m, const Vector3& trans)
{
  data[0][3] = data[1][3] = data[2][3] = Zero;
  data[3][3] = One;
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      data[i][j] = m.data[i][j];
  setCol4(trans);
}

inline void Matrix4::setZero()
{
  set(Zero);
}

inline void Matrix4::setIdentity()
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = Delta(i,j);
}

inline void Matrix4::setDiagonal(const Vector4& d)
{
  setZero();
  d.get(data[0][0],data[1][1],data[2][2],data[3][3]);
}

inline void Matrix4::setTranslate(const Vector3& trans)
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<4; j++)
      data[i][j] = Delta(i,j);

  trans.get(col4());
  data[3][3] = One;
}

inline void Matrix4::setTranspose(const Matrix4& m)
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = m.data[j][i];
}

inline void Matrix4::setNegative(const Matrix4& m)
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      data[i][j] = -m.data[i][j];

}

inline void Matrix4::setOuterProduct(const Vector4& a,const Vector4& b)
{
  data[0][0] = a.x*b.x; data[1][0] = a.x*b.y; data[2][0] = a.x*b.z; data[3][0] = a.x*b.z;
  data[0][1] = a.y*b.x; data[1][1] = a.y*b.y; data[2][1] = a.y*b.z; data[3][1] = a.y*b.z;
  data[0][2] = a.z*b.x; data[1][2] = a.z*b.y; data[2][2] = a.z*b.z; data[3][2] = a.z*b.z;
  data[0][3] = a.w*b.x; data[1][3] = a.w*b.y; data[2][3] = a.w*b.z; data[3][3] = a.w*b.w;
}

inline void Matrix4::get(Matrix4& m) const { m.set(*this); }

inline void Matrix4::get(Real m[4][4]) const
{
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++)
      m[i][j] = data[i][j];
}

inline void Matrix4::get(Real m[16]) const
{
  int i,j,k=0;
  for(i=0; i<4; i++)
    for(j=0; j<4; j++, k++)
      m[k] = data[i][j];
}

inline void Matrix4::get(Vector3& xb, Vector3& yb, Vector3& zb, Vector3& trans) const
{
  getCol1(xb);
  getCol2(yb);
  getCol3(zb);
  getCol4(trans);
}

inline void Matrix4::get(Vector4& x, Vector4& y, Vector4& z, Vector4& w) const
{
  getCol1(x);
  getCol2(y);
  getCol3(z);
  getCol4(w);
}

inline void Matrix4::get(Matrix3& m) const
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      m.data[i][j] = data[i][j];
}

inline void Matrix4::get(Matrix3& m, Vector3& trans) const
{
  int i,j;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++)
      m.data[i][j] = data[i][j];
  getCol4(trans);
}

inline void Matrix4::getTranspose(Matrix4& m) const { m.setTranspose(*this); }
inline void Matrix4::getNegative(Matrix4& m) const { m.setNegative(*this); }
inline bool Matrix4::getInverse(Matrix4& m) const { return m.setInverse(*this); }

inline void Matrix4::inplaceTranspose()
{
  Real temp;
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<i; j++) {
      temp = data[i][j];
      data[i][j] = data[j][i];
      data[j][i] = temp;
    }
  }
}

inline void Matrix4::inplaceNegative()
{
  int i,j;
  for(i=0; i<4; i++) {
    for(j=0; j<4; j++) {
      data[i][j] = -data[i][j];
    }
  }
}

inline bool Matrix4::inplaceInverse()
{
  Matrix4 tmp = *this;
  return setInverse(tmp);
}

inline void Matrix4::inplaceMul(Real s)
{
  int i,j;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++)
      data[i][j] *= s;
}

inline void Matrix4::inplaceDiv(Real s)
{
  int i,j;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++)
      data[i][j] /= s;
}

inline bool Matrix4::isZero(Real eps) const
{
  int i,j;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++)
      if(!FuzzyZero(data[i][j],eps)) return false;
  return true;
}

inline bool Matrix4::isEqual(const Matrix4& a,Real eps) const
{
  int i,j;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++)
      if(!FuzzyEquals(data[i][j],a.data[i][j],eps)) return false;
  return true;
}

inline bool Matrix4::isIdentity(Real eps) const
{
  int i,j;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++)
      if(!FuzzyEquals(data[i][j],Delta(i,j),eps)) return false;
  return true;
}

inline bool Matrix4::isInvertible(Real eps) const
{
  return !FuzzyEquals(determinant(),eps);
}

inline Real Matrix4::trace() const { return data[0][0] + data[1][1] + data[2][2] + data[3][3]; }

inline Real Matrix4::minElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(data[i][j] < vmin) {  //recall data is stored column-major
	vmin=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix4::maxElement(int* _i,int* _j) const
{
  Real vmax=-Inf;
  int imin=0,jmin=0;
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(data[i][j] > vmax) {  //recall data is stored column-major
	vmax=data[i][j];
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}

Real Matrix4::minAbsElement(int* _i,int* _j) const
{
  Real vmin=Inf;
  int imin=0,jmin=0;
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(Abs(data[i][j]) < vmin) {  //recall data is stored column-major
	vmin=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmin;
}

Real Matrix4::maxAbsElement(int* _i,int* _j) const
{
  Real vmax=0;
  int imin=0,jmin=0;
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++) 
      if(Abs(data[i][j]) > vmax) {  //recall data is stored column-major
	vmax=Abs(data[i][j]);
	imin=j;
	jmin=i;
      }
  if(_i) *_i=imin;
  if(_j) *_j=jmin;
  return vmax;
}




inline bool RigidTransform::operator == (const RigidTransform& a) const { return a.R == R && a.t == t; }
inline bool RigidTransform::operator != (const RigidTransform& a) const { return a.R != R || a.t != t; }
inline const RigidTransform& RigidTransform::operator = (const RigidTransform& x) { set(x); return *this; }
inline void RigidTransform::operator *= (const RigidTransform& x) { t += R*x.t; R *= x.R; }
inline void RigidTransform::operator *= (const Matrix3& r) { inplaceRotate(r); }
inline void RigidTransform::operator += (const Vector3& v) { t += v; }
inline void RigidTransform::operator -= (const Vector3& v) { t -= v; }
inline RigidTransform::operator Matrix4 () const { Matrix4 tmp; tmp.set(R, t); return tmp; }
inline void RigidTransform::compose(const RigidTransform& a, const RigidTransform& b)
{
  //t = ta + Ra*tb
  //R = Ra*Rb
  a.R.mul(b.t, t);
  t += a.t;
  R.mul(a.R,b.R);
}
inline void RigidTransform::composeInverseA(const RigidTransform& a, const RigidTransform& b)
{
  //Ra' = Ra^t
  //ta' = -Ra'*ta
  //t = ta' + Ra'*tb
  //R = Ra'*Rb
  a.R.mulTranspose(b.t-a.t, t);
  R.mulTransposeA(a.R,b.R);
}
inline void RigidTransform::composeInverseB(const RigidTransform& a, const RigidTransform& b)
{
  //Rb' = Rb^t
  //tb' = -Rb'*tb
  //t = ta + Ra*tb'
  //R = Ra*Rb'
  R.mulTransposeB(a.R,b.R); 
  R.mul(b.t, t);
  t.inplaceNegative();
  t += a.t;
}
inline void RigidTransform::mul(const RigidTransform& a, const RigidTransform& b) { compose(a,b); }
inline void RigidTransform::mulInverseA(const RigidTransform& a, const RigidTransform& b) { composeInverseA(a,b); }
inline void RigidTransform::mulInverseB(const RigidTransform& a, const RigidTransform& b) { composeInverseB(a,b); }
//vector operators
inline void RigidTransform::mul(const Vector3& a, Vector3& out) const { mulPoint(a,out); }
inline void RigidTransform::mul(const Vector4& a, Vector3& out) const { R.mul(Vector3(a.x,a.y,a.z), out); out.madd(t, a.w); }
inline void RigidTransform::mulPoint(const Vector3& a, Vector3& out) const { R.mul(a,out); out += t; }
inline void RigidTransform::mulVector(const Vector3& a, Vector3& out) const { R.mul(a,out); }
inline void RigidTransform::mulInverse(const Vector3& a, Vector3& out) const { mulPointInverse(a,out); }
inline void RigidTransform::mulInverse(const Vector4& a, Vector3& out) const { Vector3 tmp(a.x,a.y,a.z); tmp.madd(t,-a.w); R.mulTranspose(tmp,out); }
inline void RigidTransform::mulPointInverse(const Vector3& a, Vector3& out) const { R.mulTranspose(a-t,out); }
inline void RigidTransform::mulVectorInverse(const Vector3& a, Vector3& out) const { R.mulTranspose(a,out); }
inline void RigidTransform::setIdentity() { R.setIdentity(); t.setZero(); }
inline void RigidTransform::set(const RigidTransform& x) { set(x.R, x.t); }
inline void RigidTransform::set(const Matrix3& m, const Vector3& v) { R.set(m); t.set(v); }
inline void RigidTransform::set(const Vector3& x, const Vector3& y, const Vector3& z, const Vector3& trans) { R.set(x,y,z); t.set(trans); }
inline void RigidTransform::set(const Matrix4& m) { m.get(R,t); }
inline void RigidTransform::setRotate(const Matrix3& m) { R.set(m); t.setZero(); }
inline void RigidTransform::setTranslate(const Vector3& v) { R.setIdentity(); t.set(v); }
inline void RigidTransform::setRotation(const Matrix3& m) { R.set(m); }
inline void RigidTransform::setTranslation(const Vector3& v) { t.set(v); }
inline void RigidTransform::setInverse(const RigidTransform& x) { R.setTranspose(x.R); R.mul(x.t,t); t.inplaceNegative(); }
inline void RigidTransform::setRotated(const RigidTransform& a, const Matrix3& r) { R.mul(a.R,r); t.set(a.t); }
inline void RigidTransform::setShifted(const RigidTransform& a, const Vector3& v) { R.set(a.R); t.add(a.t,v); }
inline void RigidTransform::get(RigidTransform& x) const { x.set(*this); }
inline void RigidTransform::get(Matrix3& m, Vector3& v) const { m.set(R); v.set(t); }
inline void RigidTransform::get(Vector3& x, Vector3& y, Vector3& z, Vector3& trans) const { R.get(x,y,z); t.get(trans); }
inline void RigidTransform::get(Matrix4& m) const { m.set(R,t); }
inline void RigidTransform::getRotation(Matrix3& m) const { m.set(R); }
inline void RigidTransform::getTranslation(Vector3& v) const { v.set(t); }
inline void RigidTransform::getInverse(RigidTransform& x) { x.setInverse(*this); }
inline void RigidTransform::inplaceInverse() { R.inplaceTranspose(); Vector3 tmp; tmp.setNegative(t); R.mul(tmp,t); }
inline void RigidTransform::inplaceRotate(const Matrix3& m) { R *= m; }
inline void RigidTransform::inplaceShift(const Vector3& v) { t += v; }
inline bool RigidTransform::isIdentity(Real eps) const { return R.isIdentity(eps) && t.isZero(eps); }

inline const RigidTransform2D& RigidTransform2D::operator = (const RigidTransform2D& x) { set(x); return *this; }
inline void RigidTransform2D::operator *= (const RigidTransform2D& x) { t += R*x.t; R *= x.R; }
inline void RigidTransform2D::compose(const RigidTransform2D& a, const RigidTransform2D& b)
{
	a.R.mul(b.t, t);
	t += a.t;
	R.mul(a.R,b.R);
}
inline void RigidTransform2D::composeInverseA(const RigidTransform2D& a, const RigidTransform2D& b)
{
  a.R.mulTranspose(b.t-a.t, t);
  R.mulTransposeA(a.R,b.R);
}
inline void RigidTransform2D::composeInverseB(const RigidTransform2D& a, const RigidTransform2D& b)
{
  R.mulTransposeB(a.R,b.R); 
  R.mulTranspose(b.t, t);
  t.inplaceNegative();
  t += a.t;
}
inline void RigidTransform2D::mul(const RigidTransform2D& a, const RigidTransform2D& b) { compose(a,b); }
inline void RigidTransform2D::mulInverseA(const RigidTransform2D& a, const RigidTransform2D& b) { composeInverseA(a,b); }
inline void RigidTransform2D::mulInverseB(const RigidTransform2D& a, const RigidTransform2D& b) { composeInverseB(a,b); }
inline void RigidTransform2D::mul(const Vector3& a, Vector2& out) const { R.mul(Vector2(a.x,a.y),out); out.madd(t,a.z); }
inline void RigidTransform2D::mul(const Vector2& a, Vector2& out) const { mulPoint(a,out); }
inline void RigidTransform2D::mulPoint(const Vector2& a, Vector2& out) const { R.mul(a,out); out+=t; }
inline void RigidTransform2D::mulVector(const Vector2& a, Vector2& out) const { R.mul(a,out); }
inline void RigidTransform2D::mulInverse(const Vector2& a, Vector2& out) const { mulPointInverse(a,out); }
inline void RigidTransform2D::mulInverse(const Vector3& a, Vector2& out) const { Vector2 tmp(a.x,a.y); tmp.madd(t,-a.z); R.mulTranspose(tmp,out); }
inline void RigidTransform2D::mulPointInverse(const Vector2& a, Vector2& out) const { R.mulTranspose(a-t,out); }
inline void RigidTransform2D::mulVectorInverse(const Vector2& a, Vector2& out) const { R.mulTranspose(a,out); }
inline void RigidTransform2D::setIdentity() { R.setIdentity(); t.setZero(); }
inline void RigidTransform2D::set(const RigidTransform2D& rhs) { R=rhs.R; t=rhs.t; }
inline void RigidTransform2D::set(const Matrix3& mat) { R.set(Vector2(mat(0,0),mat(0,1)),Vector2(mat(1,0),mat(1,1))); t.set(mat(2,0),mat(2,1)); }
inline void RigidTransform2D::set(Real theta,const Vector2& _t) { R.setRotate(theta); t=_t; }
inline void RigidTransform2D::setInverse(const RigidTransform2D& x) { R.setTranspose(x.R); R.mul(x.t,t); t.inplaceNegative(); }
inline void RigidTransform2D::get(RigidTransform2D& rhs) const { rhs.R=R; rhs.t=t; }
inline void RigidTransform2D::get(Matrix3& m) const { m.set(Vector3(R(0,0),R(1,0),Zero),Vector3(R(0,1),R(1,1),Zero),Vector3(t.x,t.y,One)); }
inline void RigidTransform2D::get(Real& theta,Vector2& _t) const { theta=Acos(R(0,0)); if(R(1,0)<0) theta=-theta; _t=t; }
inline bool RigidTransform2D::isIdentity(Real eps) const { return R.isIdentity(eps) && t.isZero(eps); }


//inlined standalone functions/operators (often less efficient than the member functions)

//Vector2

inline Real dot(const Vector2& a, const Vector2& b)
{
  return a.dot(b);
}

inline void normalize(Vector2& a)
{
  a.inplaceNormalize();
}

inline Real cross(const Vector2& a, const Vector2& b)
{
  return a.cross(b);
}

inline Vector2 project(const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.setProjection(a,b);
  return temp;
}

inline Vector2 operator - (const Vector2& a)
{
  Vector2 temp;
  temp.setNegative(a);
  return temp;
}

inline Vector2 operator + (const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.add(a,b);
  return temp;
}

inline Vector2 operator - (const Vector2& a, const Vector2& b)
{
  Vector2 temp;
  temp.sub(a,b);
  return temp;
}

inline Vector2 operator * (const Vector2& a, Real b)
{
  Vector2 temp;
  temp.mul(a,b);
  return temp;
}

inline Vector2 operator * (Real a, const Vector2& b)
{
  Vector2 temp;
  temp.mul(b,a);
  return temp;
}

inline Vector2 operator / (const Vector2& a, Real b)
{
  Vector2 temp;
  temp.div(a,b);
  return temp;
}

//Vector3

inline Real dot(const Vector3& a, const Vector3& b)
{
  return a.dot(b);
}

inline void normalize(Vector3& a)
{
  a.inplaceNormalize();
}

inline Vector3 cross(const Vector3& a, const Vector3& b)
{
  Vector3 temp;
  temp.setCross(a,b);
  return temp;
}

inline Vector3 project(const Vector3& a, const Vector3& b)
{
  Vector3 temp;
  temp.setProjection(a,b);
  return temp;
}

inline Vector3 operator - (const Vector3& a)
{
  Vector3 temp;
  temp.setNegative(a);
  return temp;
}

inline Vector3 operator + (const Vector3& a, const Vector3& b)
{
  Vector3 temp;
  temp.add(a,b);
  return temp;
}

inline Vector3 operator - (const Vector3& a, const Vector3& b)
{
  Vector3 temp;
  temp.sub(a,b);
  return temp;
}

inline Vector3 operator * (const Vector3& a, Real b)
{
  Vector3 temp;
  temp.mul(a,b);
  return temp;
}

inline Vector3 operator * (Real a, const Vector3& b)
{
  Vector3 temp;
  temp.mul(b,a);
  return temp;
}

inline Vector3 operator / (const Vector3& a, Real b)
{
  Vector3 temp;
  temp.div(a,b);
  return temp;
}

//Vector4

inline Real dot(const Vector4& a, const Vector4& b)
{
  return a.dot(b);
}

inline Real dot3(const Vector4& a, const Vector4& b)
{
  return a.dot3(b);
}

inline Real dot3(const Vector4& a, const Vector3& b)
{
  return a.dot3(b);
}

inline Real dot3(const Vector3& a, const Vector4& b)
{
  return b.dot3(a);
}

inline void normalize(Vector4& a)
{
  a.inplaceNormalize();
}

inline Vector4 operator - (const Vector4& a)
{
  Vector4 temp;
  temp.setNegative(a);
  return temp;
}

inline Vector4 operator + (const Vector4& a, const Vector4& b)
{
  Vector4 temp;
  temp.add(a,b);
  return temp;
}

inline Vector4 operator - (const Vector4& a, const Vector4& b)
{
  Vector4 temp;
  temp.sub(a,b);
  return temp;
}

inline Vector4 operator * (const Vector4& a, Real b)
{
  Vector4 temp;
  temp.mul(a,b);
  return temp;
}

inline Vector4 operator * (Real a, const Vector4& b)
{
  Vector4 temp;
  temp.mul(b,a);
  return temp;
}

inline Vector4 operator / (const Vector4& a, Real b)
{
  Vector4 temp;
  temp.div(a,b);
  return temp;
}

//Matrix2

inline Real determinant(const Matrix2& m) { return m.determinant(); }
inline Real trace(const Matrix2& m) { return m.trace(); }

inline Matrix2 transpose(const Matrix2& m)
{
  Matrix2 temp;
  temp.setTranspose(m);
  return temp;
}

inline Matrix2 inverse(const Matrix2& m)
{
  Matrix2 temp;
  if(!temp.setInverse(m)) { temp.setZero(); temp.inplaceDiv(0.0); }
  return temp;
}

inline Matrix2 outerProduct(const Vector2& a,const Vector2& b) { Matrix2 m; m.setOuterProduct(a,b);  return m; }

inline Matrix2 operator + (const Matrix2& a, const Matrix2& b)
{
	Matrix2 m;
	m.add(a,b);
	return m;
}

inline Matrix2 operator - (const Matrix2& a, const Matrix2& b)
{
	Matrix2 m;
	m.sub(a,b);
	return m;
}

inline Matrix2 operator * (const Matrix2& a, const Matrix2& b)
{
	Matrix2 m;
	m.mul(a,b);
	return m;
}


inline Matrix2 operator * (const Matrix2& a, Real b)
{
	Matrix2 m;
	m.mul(a,b);
	return m;
}

inline Matrix2 operator / (const Matrix2& a, Real b)
{
	Matrix2 m;
	m.div(a,b);
	return m;
}

inline Vector2 operator * (const Matrix2& a, const Vector2& b)
{
	Vector2 v;
	a.mul(b,v);
	return v;
}

inline Vector2 operator * (const Vector2& a, const Matrix2& b)
{
	Vector2 v;
	b.mulTranspose(a,v);
	return v;
}

//Matrix3

inline Real determinant(const Matrix3& m) { return m.determinant(); }
inline Real trace(const Matrix3& m) { return m.trace(); }

inline Matrix3 transpose(const Matrix3& m)
{
  Matrix3 temp;
  temp.setTranspose(m);
  return temp;
}

inline Matrix3 inverse(const Matrix3& m)
{
  Matrix3 temp;
  if(!temp.setInverse(m)) { temp.setZero(); temp.inplaceDiv(0.0); }
  return temp;
}

inline Matrix3 outerProduct(const Vector3& a,const Vector3& b) { Matrix3 m; m.setOuterProduct(a,b);  return m; }


inline Matrix3 operator + (const Matrix3& a, const Matrix3& b)
{
	Matrix3 m;
	m.add(a,b);
	return m;
}

inline Matrix3 operator - (const Matrix3& a, const Matrix3& b)
{
	Matrix3 m;
	m.sub(a,b);
	return m;
}

inline Matrix3 operator * (const Matrix3& a, const Matrix3& b)
{
	Matrix3 m;
	m.mul(a,b);
	return m;
}

inline Matrix3 operator * (const Matrix3& a, Real b)
{
	Matrix3 m;
	m.mul(a,b);
	return m;
}

inline Matrix3 operator / (const Matrix3& a, Real b)
{
	Matrix3 m;
	m.div(a,b);
	return m;
}

inline Vector3 operator * (const Matrix3& a, const Vector3& b)
{
	Vector3 v;
	a.mul(b,v);
	return v;
}

inline Vector3 operator * (const Vector3& a, const Matrix3& b)
{
	Vector3 v;
	b.mulTranspose(a,v);
	return v;
}


//Matrix4

inline Real determinant(const Matrix4& m) { return m.determinant(); }
inline Real trace(const Matrix4& m) { return m.determinant(); }

inline Matrix4 transpose(const Matrix4& m)
{
  Matrix4 temp;
  temp.setTranspose(m);
  return temp;
}

inline Matrix4 inverse(const Matrix4& m)
{
  Matrix4 temp;
  if(!temp.setInverse(m)) { temp.setZero(); temp.inplaceDiv(0.0); }
  return temp;
}

inline Matrix4 outerProduct(const Vector4& a,const Vector4& b) { Matrix4 m; m.setOuterProduct(a,b);  return m; }

inline Matrix4 operator + (const Matrix4& a, const Matrix4& b)
{
	Matrix4 m;
	m.add(a,b);
	return m;
}

inline Matrix4 operator - (const Matrix4& a, const Matrix4& b)
{
	Matrix4 m;
	m.sub(a,b);
	return m;
}

inline Matrix4 operator * (const Matrix4& a, const Matrix4& b)
{
	Matrix4 m;
	m.mul(a,b);
	return m;
}

inline Matrix4 operator * (const Matrix4& a, Real b)
{
	Matrix4 m;
	m.mul(a,b);
	return m;
}

inline Matrix4 operator / (const Matrix4& a, Real b)
{
	Matrix4 m;
	m.div(a,b);
	return m;
}

inline Vector4 operator * (const Matrix4& a, const Vector4& b)
{
	Vector4 v;
	a.mul(b,v);
	return v;
}

inline Vector4 operator * (const Vector4& a, const Matrix4& b)
{
	Vector4 v;
	b.mulTranspose(a,v);
	return v;
}

inline Vector3 operator * (const Matrix4& a, const Vector3& b)
{
	Vector3 v;
	a.mulVector(b,v);
	return v;
}

inline Vector3 operator * (const Vector3& a, const Matrix4& b)
{
	Vector3 v;
	b.mulVectorTranspose(a,v);
	return v;
}


//RigidTransform

inline RigidTransform operator * (const RigidTransform& a, const RigidTransform& b)
{
	RigidTransform x;
	x.compose(a,b);
	return x;
}

inline RigidTransform operator * (const RigidTransform& a, const Matrix3& b)
{
	RigidTransform x;
	x.R.mul(a.R,b);
	x.t = a.t;
	return x;
}

inline RigidTransform operator * (const Matrix3& a, const RigidTransform& b)
{
	RigidTransform x;
	x.R.mul(a,b.R);
	a.mul(b.t,x.t);
	return x;
}

inline Vector3 operator * (const RigidTransform& a, const Vector3& b)
{
	Vector3 x;
	a.mul(b,x);
	return x;
}


inline RigidTransform operator + (const RigidTransform& a, const Vector3& b)
{
	return RigidTransform (a.R,a.t+b);
}

inline RigidTransform operator - (const RigidTransform& a, const Vector3& b)
{
	return RigidTransform (a.R,a.t-b);
}



//RigidTransform2D

inline RigidTransform2D operator * (const RigidTransform2D& a, const RigidTransform2D& b)
{
	RigidTransform2D x;
	x.compose(a,b);
	return x;
}

inline Vector2 operator * (const RigidTransform2D& a, const Vector2& b)
{
	Vector2 x;
	a.mul(b,x);
	return x;
}




//IO functions

std::ostream& operator << (std::ostream&, const Vector2&);
std::istream& operator >> (std::istream&, Vector2&);

std::ostream& operator << (std::ostream&, const Vector3&);
std::istream& operator >> (std::istream&, Vector3&);

std::ostream& operator << (std::ostream&, const Vector4&);
std::istream& operator >> (std::istream&, Vector4&);

std::ostream& operator << (std::ostream&, const Matrix2&);
std::istream& operator >> (std::istream&, Matrix2&);

std::ostream& operator << (std::ostream&, const Matrix3&);
std::istream& operator >> (std::istream&, Matrix3&);

std::ostream& operator << (std::ostream&, const Matrix4&);
std::istream& operator >> (std::istream&, Matrix4&);

std::ostream& operator << (std::ostream&, const RigidTransform&);
std::istream& operator >> (std::istream&, RigidTransform&);

std::ostream& operator << (std::ostream&, const RigidTransform2D&);
std::istream& operator >> (std::istream&, RigidTransform2D&);

/*@}*/
} 

#endif
