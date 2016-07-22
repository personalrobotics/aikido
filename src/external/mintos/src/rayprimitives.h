#ifndef GEOMETRY_RAYPRIMITIVES_H
#define GEOMETRY_RAYPRIMITIVES_H 

#include "primitives.h"

/** @file geometry/rayprimitives.h
 * @ingroup Geometry
 * @brief Contains primitives that allow rays to be represented as
 * points at infinity.
 */

namespace Geometry {

  using namespace Math;
  
  /** @addtogroup Geometry */
  /*@{*/

/// A 2D point or infinite ray.  Note: rays should be normalized unit vectors.
struct PointRay2D : public Math3D::Vector2
{
	bool isRay;
};

/// A 3D point or infinite ray.  Note: rays should be normalized unit vectors.
struct PointRay3D : public Math3D::Vector3
{
	bool isRay;
};

/** @brief Comparison of 1D points in homogeneous coordinates.
 *
 * The 1D points are given in homogeneous coordinates a/aw, b/bw
 * @returns
 *  < 0 if a < b
 *  = 0 if a = b
 *  > 0 if a > b
 */
inline Real HomogeneousCmp(Real a, bool awZero, Real b, bool bwZero)
{
  if(awZero == bwZero) return a-b;
  else if(awZero) {
    if(a == 0) return -b;
    else return a;
  }
  else if(bwZero) {
    if(b == 0) return -a;
    else return b;
  }
}

/// Subtraction of 1D points in homogeneous coordinates.
inline Real HomogeneousSub(Real a, bool awZero, Real b, bool bwZero)
{
  if(awZero == bwZero) return a-b;
  else if(awZero) return a;
  else return -b;
}


/// Lexical < order on 2D point/rays
/// @sa Lexical2DOrder
inline bool LexicalRay2DOrder (const PointRay2D& p1,const PointRay2D& p2)
{
  if(p1.isRay != p2.isRay) {
    if(p1.isRay) {
      if(p1.x < 0) return true;
      else if(p1.x > 0) return false;
      if(p2.x > 0) return true;
      else if(p2.x < 0) return false;

      return (p1.y < 0);
    }
    else return !LexicalRay2DOrder(p2,p1);
	}
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  return (p1.y < p2.y);
}

/// Lexical < order on 3D point/rays
/// @sa Lexical3DOrder
inline bool LexicalRay3DOrder (const PointRay3D& p1,const PointRay3D& p2)
{
  if(p1.isRay != p2.isRay) {
    if(p1.isRay) {
      if(p1.x < 0) return true;
      else if(p1.x > 0) return false;
      if(p2.x > 0) return true;
      else if(p2.x < 0) return false;

      if(p1.y < 0) return true;
      else if(p1.y > 0) return false;
      if(p2.y > 0) return true;
      else if(p2.y < 0) return false;

      return (p1.z < 0);
    }
    else return !LexicalRay3DOrder(p2,p1);
	}
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  if(p1.y < p2.y) return true;
  else if(p1.y > p2.y) return false;
  return (p1.z < p2.z);
}

/// Orientation of p2 relative to p1, relative to p0.
/// @sa Orient2D
inline Real OrientRay2D(const PointRay2D& p0, const PointRay2D& p1, const PointRay2D& p2)
{
  //shift it so p0 isn't a ray (if possible)
  if(p0.isRay) {
    if(p1.isRay) {
      if(p2.isRay)
        return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
      else
        return OrientRay2D(p2,p0,p1);
    }
    else 
      return OrientRay2D(p1,p2,p0);
  }
  Real bx = p1.isRay ? p1.x : p1.x - p0.x;
  Real by = p1.isRay ? p1.y : p1.y - p0.y;
  Real cx = p2.isRay ? p2.x : p2.x - p0.x;
  Real cy = p2.isRay ? p2.y : p2.y - p0.y;
  return bx*cy-cx*by;
}

/*@{*/

} //namespace Geometry

namespace std {

inline ostream& operator << (ostream& out, const Geometry::PointRay2D& pt)
{
  out<<pt.x<<" "<<pt.y;
  if(pt.isRay) out<<" (ray)";
  return out;
}


}

#endif

