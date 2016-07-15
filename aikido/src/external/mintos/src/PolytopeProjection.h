#ifndef POLYTOPE_PROJECTION_H
#define POLYTOPE_PROJECTION_H

#include <mintos/math3d/primitives.h>
#include "UnboundedPolytope2D.h"
#include "LinearProgram.h"
#include "GLPKInterface.h"
#include <list>

namespace Geometry {
  using namespace Math3D;

/** @brief Helper callback used in PolytopeProjection2D.
 *
 * Given a linear program whose first two variables are the x,y directions,
 * EvalExtremum returns the furthest feasible point in that direction.
 */
class LPSolvePointCallback
{
 public:
  LPSolvePointCallback(Optimization::LinearProgram& lp,int varx,int vary);
  LPSolvePointCallback(Optimization::LinearProgram_Sparse& lps,int varx,int vary);
  bool EvalExtremum(const Vector2& dir, Geometry::PointRay2D& x);

  int varx,vary;
  Optimization::LinearProgram* lp;
  Optimization::LinearProgram_Sparse* lps;
  Optimization::GLPKInterface solver;
  bool unbounded_initialized;
  Optimization::LinearProgram lp_unbounded;
  Optimization::LinearProgram_Sparse lps_unbounded;
  Optimization::GLPKInterface unboundedSolver;
  //stats 
  int numEvals;
};



/** @brief A method for projecting a polytope defined by the feasible set
 * of a linear program lp onto a 2D subspace.  Handles unbounded and
 * infeasible problems.
 *
 * Typical calling convention is
 * UnboundedPolytope2D poly;
 * PolytopeProjection2D proj(lp);
 * proj.Create(poly);
 */
class PolytopeProjection2D 
{
 public:
  PolytopeProjection2D(Optimization::LinearProgram& lp,int varx=0,int vary=1);
  PolytopeProjection2D(Optimization::LinearProgram_Sparse& lps,int varx=0,int vary=1);

  ///Solves for the polytope.  Equivalent to Expand(), Create().
  void Solve(UnboundedPolytope2D& poly);

  ///Expand the entire polytope
  void Expand();

  ///Creates the polytope structures
  void Create(UnboundedPolytope2D& poly) const;

  typedef std::list<Geometry::PointRay2D>::iterator PointIterator;

  ///Helper: expand the edge coming out of i (whose source is i)
  void ExpandEdge(PointIterator i,int depth=0);

  LPSolvePointCallback f;
  std::list<Geometry::PointRay2D> points;
  int maxDepth;
};


} //namespace Geometry

#endif
