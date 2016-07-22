#ifndef GEOMETRY_UNBOUNDED_POLYTOPE_H
#define GEOMETRY_UNBOUNDED_POLYTOPE_H

#include <mintos/math3d/primitives.h>
#include "Plane2D.h"
#include "rayprimitives.h"
#include <vector>

namespace Geometry { 

  using namespace Math3D;

/** @ingroup Contact
 * @brief A representation of a possibly unbounded polytope in the 2d plane.
 *
 * Contains both a vertex/ray representation and a halfplane representation.
 */
struct UnboundedPolytope2D
{
  /// Converts the vertex representation in vertices to planes
  void CalcPlanes();
  /// Converts the plane representation to vertices
  void CalcVertices();
  /// Returns true if the point is within the polytope
  bool Contains(const Vector2& x) const;
  /// Returns the orthogonal distance to the nearest plane (<0 means outside)
  Real Margin(const Vector2& x) const;
  /// Returns the closest point inside the polytope if one exists (<0 means
  /// the polytope was found to be empty)
  Real ClosestPoint(const Vector2& x,Vector2& cp) const;

  /// Vertex representation of the polygon
  std::vector<PointRay2D> vertices;

  /// Halfplane representation of the polygon.
  /// Normals point outward.
  std::vector<Plane2D> planes;
};

} //namespace Geometry

#endif
