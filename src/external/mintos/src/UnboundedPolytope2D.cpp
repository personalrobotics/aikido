#include "UnboundedPolytope2D.h"
#include "ConvexHull2D.h"
#include <mintos/misc/errors.h>
#include <iostream>
using namespace std;
using namespace Geometry;

void UnboundedPolytope2D::CalcPlanes()
{
  //find the planes
  size_t k=vertices.size();
  if(k == 0) {
    planes.resize(1);
    planes[0].normal.set(1,0);
    planes[0].offset=-Inf;
  }
  else if(k == 1) {
    planes.resize(4);
    planes[0].normal.set(1,0);
    planes[0].offset = vertices[0].x;
    planes[1].normal.set(0,1);
    planes[1].offset = vertices[0].y;
    planes[2].normal.set(-1,0);
    planes[2].offset = -vertices[0].x;
    planes[3].normal.set(0,-1);
    planes[3].offset = -vertices[0].y;
  }
  else if(k == 2) {
    planes.resize(4);
    planes.resize(4);
    Vector2 v = vertices[1] - vertices[0];
    v.inplaceNormalize();
    Vector2 n;
    n.setPerpendicular(v);
    planes[0].setPointNormal(vertices[1],v);
    planes[1].setPointNormal(vertices[0],-v);
    planes[2].setPointNormal(vertices[0],n);
    planes[3].setPointNormal(vertices[0],-n);
  }
  else {
    planes.resize(k);
    int np = Point2DListToPlanes(&vertices[0],k,&planes[0]);
    planes.resize(np);
  }
}

void UnboundedPolytope2D::CalcVertices()
{
  FatalError("UnboundedPolytope2D::CalcVertices(): Not done yet");
}

bool UnboundedPolytope2D::Contains(const Vector2& x) const
{
  for(size_t i=0;i<planes.size();i++)
    if(!(planes[i].distance(x) <= Zero)) return false;
  return true;
}

Real UnboundedPolytope2D::Margin(const Vector2& x) const
{
  Real margin=-Inf;
  for(size_t i=0;i<planes.size();i++)
    margin = Max(margin,planes[i].distance(x));
  return -margin;
}

Real UnboundedPolytope2D::ClosestPoint(const Vector2& x,Vector2& cp) const
{
  cp = x;
  vector<bool> isActive(planes.size(),false);
  vector<int> activeSet;  //active set
  vector<int> freeSet(planes.size());
  for(size_t i=0;i<planes.size();i++) freeSet[i]=(int)i;
  bool done=false;
  while(!done) {
    int enteringIndex=-1;
    //max dist heuristic (remember planes point outward)
    Real maxDist = -Inf;
    for(size_t i=0;i<freeSet.size();i++) {
      int p=freeSet[i];
      if(planes[p].distance(cp) > maxDist) {
	//entering
	enteringIndex = int(i);
	maxDist = planes[p].distance(cp);
      }
    }
    if(enteringIndex == -1 || maxDist < Epsilon) {
      //all done!
      done=true;
      break;
    }
    int entering = freeSet[enteringIndex];
    
    //which means some other planes may be leaving
    if(activeSet.size()>=2) {
      //find which pair of planes gives the furthest point from x
      int furthestIndex = -1;
      Real furthestDistance = x.distanceSquared(cp);
      Vector2 pt;
      for(size_t i=0;i<activeSet.size();i++) {
	int p = activeSet[i];
	int res=planes[p].allIntersections(planes[entering],pt);
	if(res == 1) { //one intersection at a plane
	  if(x.distanceSquared(pt) > furthestDistance) {
	    furthestIndex = (int)i;
	    furthestDistance = x.distanceSquared(pt);
	  }
	}
	else if(res == 0) {  //uh.. no intersections
	  fprintf(stderr,"No intersections between entering plane and active plane\n");
	  return -Inf;
	}
	else {  //planes are identical -- no change
	}
      }
      if(furthestIndex == -1) {
	fprintf(stderr,"Uh... adding plane with equivalent distance to x\n");
	Abort();
      }
      //calculate the new cp
      int remaining = activeSet[furthestIndex];
      int res=planes[remaining].allIntersections(planes[entering],cp);
      Assert(res==1);

      //delete all items in the active set except for the remaining plane
      for(size_t i=0;i<activeSet.size();i++)
	isActive[activeSet[i]] = false;
      activeSet.resize(0);
      isActive[remaining]=true;
      activeSet.push_back(remaining);
    }
    else if(activeSet.size()==1) {
      int res=planes[activeSet[0]].allIntersections(planes[entering],cp);
      Assert(res==1);
    }
    else {
      Vector2 pt;
      planes[entering].project(cp,pt);
      cp = pt;
    }

    freeSet.erase(freeSet.begin()+enteringIndex);
    isActive[entering]=true;
    activeSet.push_back(entering);
  }
  return x.distance(cp);
}
