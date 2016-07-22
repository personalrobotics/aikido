#include "ConvexHull2D.h"
#include "arrayutils.h"
#include <iostream>
using namespace std;

namespace Geometry {

/// Lexical < order on 2D points
inline bool Lexical2DOrder (const Point2D& p1,const Point2D& p2)
{
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  return (p1.y < p2.y);
}

/// Lexical < order on 3D points
inline bool Lexical3DOrder (const Point3D& p1,const Point3D& p2)
{
  if(p1.x < p2.x) return true;
  else if(p1.x > p2.x) return false;
  if(p1.y < p2.y) return true;
  else if(p1.y > p2.y) return false;
  return (p1.z < p2.z);
}

/** @brief Orientation of p2 relative to p1, relative to p0.
 *
 * @return
 * >0 for p2 left of the line through p0 and p1
 * =0 for p2 on the line
 * <0 for p2 right of the line
 */
inline Real Orient2D(const Point2D& p0, const Point2D& p1, const Point2D& p2)
{
  return (p1.x - p0.x)*(p2.y - p0.y) - (p2.x - p0.x)*(p1.y - p0.y);
}


struct Point2DWithID : public Point2D
{
  void set(const Point2D& pt,int _id) {
    Point2D::operator =(pt);
    id = _id;
  }
  int id;
};

struct PointRay2DWithID : public PointRay2D
{
  void set(const PointRay2D& pt,int _id) {
    PointRay2D::operator =(pt);
    id = _id;
  }
  int id;
};

template <typename Point,typename OrientFunc>
int ConvexHull2D_ChainTemplate(const Point P[],int n, Point H[],OrientFunc Orient)
{
    // the output array H[] will be used as the stack
    int    bot=0, top=(-1);  // indices for bottom and top of the stack
    int    i;                // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    Real xmin = P[0].x;
    for (i=1; i<n; i++)
        if (P[i].x != xmin) break;
    minmax = i-1;
    if (minmax == n-1) {       // degenerate case: all x-coords == xmin
        H[++top] = P[minmin];
        if (P[minmax].y != P[minmin].y) // a nontrivial segment
            H[++top] = P[minmax];
        H[++top] = P[minmin];           // add polygon endpoint
        return top;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = n-1;
    Real xmax = P[n-1].x;
    for (i=n-2; i>=0; i--)
        if (P[i].x != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    H[++top] = P[minmin];      // push minmin point onto stack
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin] with P[maxmin]
        if (Orient( P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;          // ignore P[i] above or on the lower line

        while (top > 0)        // there are at least 2 points on the stack
        {
            // test if P[i] is left of the line at the stack top
            if (Orient( H[top-1], H[top], P[i]) > 0)
                break;         // P[i] is a new hull vertex
            else
                top--;         // pop top point off stack
        }
        H[++top] = P[i];       // push P[i] onto stack
    }

    // Next, compute the upper hull on the stack H above the bottom hull
    if (maxmax != maxmin)      // if distinct xmax points
        H[++top] = P[maxmax];  // push maxmax point onto stack
    bot = top;                 // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax] with P[minmax]
        if (Orient( P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
            continue;          // ignore P[i] below or on the upper line

        while (top > bot)    // at least 2 points on the upper stack
        {
            // test if P[i] is left of the line at the stack top
            if (Orient( H[top-1], H[top], P[i]) > 0)
                break;         // P[i] is a new hull vertex
            else
                top--;         // pop top point off stack
        }
        H[++top] = P[i];       // push P[i] onto stack
    }
    if (minmax != minmin)
        H[++top] = P[minmin];  // push joining endpoint onto stack

    assert(top <= n);
    return top;
}

int ConvexHull2D_Chain(const Point2D P[], int n, Point2D H[]) {
  return ConvexHull2D_ChainTemplate(P,n,H,Orient2D);
}

int ConvexHull2D_Chain(const PointRay2D P[], int n, PointRay2D H[]) {
  return ConvexHull2D_ChainTemplate(P,n,H,OrientRay2D);
}

int ConvexHull2D_Chain(const Point2D P[], int n, Point2D H[],int Hindex[]) {
  Point2DWithID* Pid = new Point2DWithID[n];
  Point2DWithID* Hid = new Point2DWithID[n+1];
  for(int i=0;i<n;i++)
    Pid[i].set(P[i],i);
  n = ConvexHull2D_ChainTemplate(Pid,n,Hid,Orient2D);
  for(int i=0;i<n;i++) {
    H[i] = Hid[i];
    Hindex[i] = Hid[i].id;
  }
  delete [] Pid;
  delete [] Hid;
  return n;
}

int ConvexHull2D_Chain(const PointRay2D P[], int n, PointRay2D H[],int Hindex[]) {
  PointRay2DWithID* Pid = new PointRay2DWithID[n];
  PointRay2DWithID* Hid = new PointRay2DWithID[n+1];
  for(int i=0;i<n;i++)
    Pid[i].set(P[i],i);
  n = ConvexHull2D_ChainTemplate(Pid,n,Hid,OrientRay2D);
  for(int i=0;i<n;i++) {
    H[i] = Hid[i];
    Hindex[i] = Hid[i].id;
  }
  delete [] Pid;
  delete [] Hid;
  return n;
}

int ConvexHull2D_Chain_Unsorted(Point2D P[], int n, Point2D H[])
{
  ArrayUtils::sort(P,n,Lexical2DOrder);
  assert(ArrayUtils::is_sorted(P,n,Lexical2DOrder));
  return ConvexHull2D_Chain(P,n,H);
}

int ConvexHull2D_Chain_Unsorted(PointRay2D P[], int n, PointRay2D H[])
{
  ArrayUtils::sort(P,n,LexicalRay2DOrder);
  assert(ArrayUtils::is_sorted(P,n,LexicalRay2DOrder));
  return ConvexHull2D_Chain(P,n,H);
}

int ConvexHull2D_Chain_Unsorted(Point2D P[], int n, Point2D H[],int Hindex[]) {
  Point2DWithID* Pid = new Point2DWithID[n];
  Point2DWithID* Hid = new Point2DWithID[n+1];
  for(int i=0;i<n;i++)
    Pid[i].set(P[i],i);
  ArrayUtils::sort(Pid,n,Lexical2DOrder);
  assert(ArrayUtils::is_sorted(Pid,n,Lexical2DOrder));
  //n = ConvexHull2D_ChainTemplate(Pid,n,Hid,Orient2D);
  for(int i=0;i<n;i++) {
    //H[i] = Hid[i];
    //Hindex[i] = Hid[i].id;
    H[i] = P[i];
    Hindex[i] = i;
  }
  delete [] Pid;
  delete [] Hid;
  return n;
}

int ConvexHull2D_Chain_Unsorted(PointRay2D P[], int n, PointRay2D H[],int Hindex[]) {
  PointRay2DWithID* Pid = new PointRay2DWithID[n];
  PointRay2DWithID* Hid = new PointRay2DWithID[n+1];
  for(int i=0;i<n;i++)
    Pid[i].set(P[i],i);
  ArrayUtils::sort(Pid,n,LexicalRay2DOrder);
  assert(ArrayUtils::is_sorted(Pid,n,LexicalRay2DOrder));
  n = ConvexHull2D_ChainTemplate(Pid,n,Hid,OrientRay2D);
  for(int i=0;i<n;i++) {
    H[i] = Hid[i];
    Hindex[i] = Hid[i].id;
  }
  delete [] Pid;
  delete [] Hid;
  return n;
}

void Point2DListToPlanes(const Point2D P[], int n, Plane2D H[])
{
  for(int i=0;i<n;i++) {
    int j=(i+1)%n;
    H[i].setPoints(P[i],P[j]);
  }
}

int Point2DListToPlanes(const PointRay2D P[], int n, Plane2D H[])
{
  int k=0;
  for(int i=0;i<n;i++) {
    int j=(i+1)%n;
    if(P[i].isRay && P[j].isRay) 
      continue;
    Real x = HomogeneousSub(P[j].x,P[j].isRay,P[i].x,P[i].isRay);
    Real y = HomogeneousSub(P[j].y,P[j].isRay,P[i].y,P[i].isRay);
    H[k].normal.set(y,-x);
    H[k].normal.inplaceNormalize();
    if(P[i].isRay)
      H[k].offset = H[k].normal.dot(P[j]);
    else
      H[k].offset = H[k].normal.dot(P[i]);
    k++;
  }
  return k;
}

} //namespace Geometry
