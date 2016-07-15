#ifndef GEOMETRY_CONVEXHULL2D_H
#define GEOMETRY_CONVEXHULL2D_H

#include "primitives.h"
#include "rayprimitives.h"
#include "Plane2D.h"

/** @file geometry/ConvexHull2D.h
 * @ingroup Geometry
 * @brief 2-D convex hull routines
 */

namespace Geometry {

  using namespace Math3D;
  typedef Vector2 Point2D;
  typedef Vector3 Point3D;

  /** @addtogroup Geometry */
  /*@{*/

/** @brief Compute the convex hull of a set of sorted points (or point-rays)
 *
 * Uses Andrew's monotone chain algorithm which is O(n).
 * @param P an array of 2D points presorted by increasing x- and y-coordinates
 * @param n the number of points in P[]
 * @param H an array of the convex hull vertices (max is n+1)
 *            (NOTE THAT THE SIZE OF H SHOULD BE n+1, NOT n)
 * @param HIndex (optional) for each convex hull vertex, the
 *             index of the original vertex in P
 * @return the number of points in H
 */
int ConvexHull2D_Chain(const Point2D P[], int n, Point2D H[]);
int ConvexHull2D_Chain(const PointRay2D P[], int n, PointRay2D H[]);
int ConvexHull2D_Chain(const Point2D P[], int n, Point2D H[], int Hindex[]);
int ConvexHull2D_Chain(const PointRay2D P[], int n, PointRay2D H[], int Hindex[]);

/** @brief Same as above, but unsorted input.
 * Upon exit, the contents of P are rearranged in lexicographical order
 */
int ConvexHull2D_Chain_Unsorted( Point2D P[], int n, Point2D H[]);
int ConvexHull2D_Chain_Unsorted( PointRay2D P[], int n, PointRay2D H[]);
int ConvexHull2D_Chain_Unsorted( Point2D P[], int n, Point2D H[],int Hindex[]);
int ConvexHull2D_Chain_Unsorted( PointRay2D P[], int n, PointRay2D H[],int Hindex[]);

/** @brief Given a list of points on a convex hull, return a list
 *   of planes along edges with normals pointing outward.
 *
 * H must have storage for n planes.
 * For the PointRay version, the number of returned planes may be less than n.
 * @return The number of planes.
 */
void Point2DListToPlanes(const Point2D P[], int n, Plane2D H[]);
int Point2DListToPlanes(const PointRay2D P[], int n, Plane2D H[]);

  /*@}*/

} // namespace Geometry

#endif
