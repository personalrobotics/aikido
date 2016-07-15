#include "PolytopeProjection.h"
#include <iostream>
#include "ConvexHull2D.h"
using namespace Optimization;
using namespace Geometry;
using namespace std;



PolytopeProjection2D::PolytopeProjection2D(Optimization::LinearProgram& lp,int varx,int vary)
  :f(lp,varx,vary),maxDepth(6)
{}

PolytopeProjection2D::PolytopeProjection2D(Optimization::LinearProgram_Sparse& lps,int varx,int vary)
  :f(lps,varx,vary),maxDepth(6)
{}


void PolytopeProjection2D::Solve(UnboundedPolytope2D& poly)
{
  Expand();
  Create(poly);
}

void PolytopeProjection2D::Create(UnboundedPolytope2D& poly) const
{
  if(points.empty()) {
    //it's infeasible!
    poly.vertices.clear();
    poly.planes.resize(1);
    poly.planes[0].normal.set(1,0);
    poly.planes[0].offset = -Inf;
    return;
  }

  //find the convex hull
  vector<PointRay2D> pts(points.size());
  std::copy(points.begin(),points.end(),pts.begin());
  poly.vertices.resize(pts.size()+1);
  int k = ConvexHull2D_Chain_Unsorted(&pts[0],pts.size(),&poly.vertices[0]);
  poly.vertices.resize(k);

  //convert the vertex list to planes
  poly.CalcPlanes();
}

void PolytopeProjection2D::Expand()
{
  vector<PointRay2D> initPts(2);
  Real angle = 0;
  Vector2 d(Cos(angle),Sin(angle));
  if(!f.EvalExtremum(d,initPts[0])) {
    fprintf(stderr,"PolytopeProjection2D::Expand(): looks like polytope is empty!\n");
    points.clear();
    return;
  }
  if(!f.EvalExtremum(-d,initPts[1])) {
    fprintf(stderr,"PolytopeProjection2D::Expand(): Strange! could eval one extremum of polytope, but not the second!\n");
    points.clear();
    return;
  }
  //cout<<"Initial points "<<initPts[0]<<", "<<initPts[1]<<endl;
  points.push_back(initPts[0]);
  points.push_back(initPts[1]);
  PointIterator i1=points.begin();
  PointIterator i2=++points.begin();
  ExpandEdge(i1);
  ExpandEdge(i2);
}

//expand the edge coming out of i (whose source is i)
void PolytopeProjection2D::ExpandEdge(PointIterator i,int depth)
{
  if(depth > maxDepth) {
    //cout<<"ExpandEdge iters exceeding "<<maxDepth<<", quitting iteration"<<endl;
    //getchar();
    return;
  }
  //get the next point in the list
  PointIterator j=i; j++;
  if(j == points.end()) j = points.begin();
  
  //get a representation for the ray from i to j - d is direction, o is origin
  Vector2 n,d,o;
  if(i->isRay) {
    if(j->isRay) {
      //only expand if they're not ccw, and then do so with the angle bisector
      if(i->cross(*j) > Zero) {
	fprintf(stderr,"CCW turn of angle %g between rays!\n",Asin(i->cross(*j)));
	return;
      }
      else {
	cout<<"Expanding between rays "<<*i<<" and "<<*j<<endl;
	n.set(-*i-*j);
	n.inplaceNormalize();
	PointRay2D x;
	if(!f.EvalExtremum(n,x)) return;
	
	if(x.isRay) {
	  cout<<"It's a ray... "<<x<<endl;
	  PointIterator k=points.insert(j,x);
	  ExpandEdge(i,depth+1);
	  ExpandEdge(k,depth+1);
	}
	else {
	  cout<<"It's a point... "<<x<<endl;
	  PointIterator k=points.insert(j,x);
	  ExpandEdge(i,depth+1);
	  ExpandEdge(k,depth+1);
	}
	return;
      }
    }
    else {
      d = *i;
      d.inplaceNegative();
      o = (*j);
    }
  }
  else {
    if(j->isRay) {
      d = *j;
    }
    else d = (*j-*i);
    o = (*i);
  }
  n.set(d.y,-d.x);
  n.inplaceNormalize();
  PointRay2D x;
  if(!f.EvalExtremum(n,x)) return;
  
  //if x doesn't lie on the edge, insert a point between i and j
  if(x.isRay) {
    if(dot(x,n) > Epsilon) {
      PointIterator k=points.insert(j,x);
      if(!i->isRay) {
	ExpandEdge(i,depth+1);
      }
      if(!j->isRay) {
	ExpandEdge(k,depth+1);
      }
    }
    else {
      //cout<<"Extremum "<<x<<" in direction "<<n<<" doesn't expand edge"<<endl;
    }
  }
  else {
    if(dot(x,n) - dot(o,n) > Epsilon) {
      PointIterator k=points.insert(j,x);
      ExpandEdge(i,depth+1);
      ExpandEdge(k,depth+1);
    }
  }
}


LPSolvePointCallback::LPSolvePointCallback(LinearProgram& _lp,int _varx,int _vary)
  :varx(_varx),vary(_vary),lp(&_lp),lps(NULL),
   unbounded_initialized(false),numEvals(0)
{
  Assert(lp->c.n > varx && varx >= 0);
  Assert(lp->c.n > vary && vary >= 0);
}

LPSolvePointCallback::LPSolvePointCallback(LinearProgram_Sparse& _lps,int _varx,int _vary)
  :varx(_varx),vary(_vary),lp(NULL),lps(&_lps),
   unbounded_initialized(false),numEvals(0)
{
  Assert(lps->c.n > varx && varx >= 0);
  Assert(lps->c.n > vary && vary >= 0);
}

bool LPSolvePointCallback::EvalExtremum(const Vector2& dir, PointRay2D& x)
{
  Vector xopt;
  numEvals++;
  LinearProgram::Result res;
  if(lp) {
    lp->minimize = false;
    lp->c.setZero();
    lp->c(varx) = dir.x; lp->c(vary) = dir.y;
    solver.Set(*lp);
    res = solver.Solve(xopt);
  }
  else {
    Assert(lps != NULL);
    lps->minimize = false;
    lps->c.setZero();
    lps->c(varx) = dir.x;
    lps->c(vary) = dir.y;
    solver.Set(*lps);
    res = solver.Solve(xopt);
  }
  switch(res) {
  case LinearProgram::Unbounded:
    {
      //get the unbounded lp
      if(lp) 
	lp_unbounded.c = lp->c;
      else
	lps_unbounded.c = lps->c;
      LinearProgram::Result res2;
      if(!unbounded_initialized) {
	if(lp) {
	  lp_unbounded = *lp;
	  //convert lower bounds, upper bounds to 0, unbounded to -1,1 
	  for(int i=0;i<lp_unbounded.l.n;i++) {
	    if(IsInf(lp_unbounded.l(i)) != -1) lp_unbounded.l(i) = 0;
	    else lp_unbounded.l(i) = -1;
	    if(IsInf(lp_unbounded.u(i)) != 1) lp_unbounded.u(i) = 0;
	    else lp_unbounded.u(i) = 1;
	  }
	  //convert lower bounds, upper bounds to 0
	  for(int i=0;i<lp_unbounded.p.n;i++) {
	    if(IsInf(lp_unbounded.q(i)) != -1) lp_unbounded.q(i) = 0;
	    else lp_unbounded.q(i) = -1;
	    if(IsInf(lp_unbounded.p(i)) != 1) lp_unbounded.p(i) = 0;
	    else lp_unbounded.p(i) = 1;
	  }
	}
	else {
	  lps_unbounded = *lps;
	  //convert lower bounds, upper bounds to 0, unbounded to -1,1 
	  for(int i=0;i<lps_unbounded.l.n;i++) {
	    if(IsInf(lps_unbounded.l(i)) != -1) lps_unbounded.l(i) = 0;
	    else lps_unbounded.l(i) = -1;
	    if(IsInf(lps_unbounded.u(i)) != 1) lps_unbounded.u(i) = 0;
	    else lps_unbounded.u(i) = 1;
	  }
	  //convert lower bounds, upper bounds to 0
	  for(int i=0;i<lps_unbounded.p.n;i++) {
	    if(IsInf(lps_unbounded.q(i)) != -1) lps_unbounded.q(i) = 0;
	    else lps_unbounded.q(i) = -1;
	    if(IsInf(lps_unbounded.p(i)) != 1) lps_unbounded.p(i) = 0;
	    else lps_unbounded.p(i) = 1;
	  }
	}

	unbounded_initialized = true;
	if(lp) 
	  unboundedSolver.Set(lp_unbounded);
	else
	  unboundedSolver.Set(lps_unbounded);
      }
      else {
	if(lp)
	  unboundedSolver.SetObjective(lp_unbounded.c,lp_unbounded.minimize);
	else
	  unboundedSolver.SetObjective(lps_unbounded.c,lps_unbounded.minimize);
      }
      res2 = unboundedSolver.Solve(xopt);
      Assert(res2 != LinearProgram::Unbounded);
      if(res2 == LinearProgram::Feasible) {
	x.x = xopt(varx);
	x.y = xopt(vary);
	x.isRay = true;
	x.inplaceNormalize();
	//cout<<"Polytope projection unbounded in direction "<<dir<<", ray "<<x<<endl;
      }
      else {
	cout<<"Couldn't solve for unbounded direction! "<<dir<<endl;
	return false;
      }
    }
    break;
  case LinearProgram::Feasible:
    x.x = xopt(varx);
    x.y = xopt(vary);
    x.isRay = false;
    break;
  default:
    return false;
  }
  return true;
}
  

