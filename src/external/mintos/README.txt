*******************************************************
* Manifold Interpolation and Time Optimized Smoothing *
*                 (MInTOS) library                    *
*                      v 0.2                          *
*                                                     *
*          Kris Hauser, Indiana University            *
*                hauserk@indiana.edu                  *
*******************************************************

Contents
I. Package Contents
II. Compiling
III. Usage
  A. Defining manifold constraints
  B. Example code
  C. Non-Cartesian interpolation
  D. Performance issues
  E. timeopt utility
IV. Contact Information
V. Version History
VI. License

**** I. Package Contents ****

Mintos is a package for 1) generating smooth interpolating paths on
manifolds and 2) optimizing a time-scaling of such a path under
actuation constraints.  It supports velocity and acceleration bounds strictly across the entire trajectory, and can also handle torque and frictional force bounds at collocation points. 

In basic usage, Mintos takes as input:
* start/end configurations qs and qe,
* a vector-valued constraint C(x)=0
* velocity bounds (vmin,vmax) and acceleration bounds (amin,amax).

It can handle torque bounds given:
* hooks to compute elements of the robot's dynamics equation.

Additionally, it can handle force constraints at fixed contact points given:
* a list of contact points, their links, and their Coulomb friction coefficients,
* hooks to compute Jacobians.

The interpolation step outputs a path p(s):[0,1]->R^n such that
* p(0) = qs, p(1) = qe
* C(p(s)) ~= 0 up to some specified tolerance
* p(s) is twice differentiable.

The time scaling step computes a time scaling s(t) and final time T
such that p(s(t)):[0,T]->R^n satisfies:
* vmin <= dp/dt <= vmax 
* amin <= ddp/dt^2 <= amax.
and (if specified) torque and frictional force constraints.

Convenience functions are also given to support smooth interpolation
of N configurations q[0],...,q[N-1].  The library also supports 
arbitrary geodesic ambient spaces


The interpolation step is not guaranteed to always find a solution
but it works quite reliably in practice.  The time scaling is
guaranteed to converge to an optimum up to the
specified resolution.  More details can be found in the paper:

  K. Hauser. "Fast Interpolation and Time-Optimization on Implicit
  Contact Submanifolds". Robotics: Science and Systems Conference, 2013.

Please acknowledge (e.g., cite) the paper if you find Mintos useful
in your projects.



include/mintos/
  - Include files for the Mintos library
lib/
  - Will contain the shared library file libMintos.a after building
src/
  - Source files.
examples/
  - Example files for the timeopt program.
test.cpp
  - Code for a test program.
timeopt.cpp
  - Code for time optimization of a bezier spline.
Timer.h/cpp
  - Cross platform timer for use in the test program.
Makefile
  - The project makefile.
doxygen.conf
  - Configuration file for the automatic documentation tool doxygen.


**** II. Compiling ****

Dependencies:
* The GNU Linear Programming Kit (GLPK) must first be installed on your
  system.  The library was tested with version 4.39 but more recent versions
  should work as well.
The 'GLPK' and 'GLPK_LIB' variables in the Makefile should be edited to
reflect the locations of GLPK include and lib files on your system.

Building:
* 'make' will make the library and the test program
* 'make lib' will make the library only
* 'make test' will make the test program
* 'make timeopt' will make the time-optimization program
* 'make docs' will build the documentation in the docs folder

Running:

* './test' will perform several optimizations on toy
  problems and print the results.

* './timeopt spline_file limits_file grid_res dt' will optimize
  a Bezier spline and print the results to stdout.  A 1D example is
  ./timeopt examples/simple.spline examples/unit_limits.txt 100 0.01 


**** III. Usage ****

Interpolation and optimization occur in separate steps:
1) Path interpolation on a constraint manifold via recursive
   Bezier curve projection.
2) Time scaling under bounded velocities and accelerations via
   convex optimization (sequential linear program).

** III.A. Defining manifold constraints **

A hook to a vector-field constraint C(x)=0, is defined by subclassing
the VectorFieldFunction class (mintos/math/functions.h).  For example, 
the unit sphere constraint inputs a 3D vector x and the difference
between ||x||^2 and 1:

class UnitSphereFunction : public VectorFieldFunction
{
 public:
  virtual void Eval(const Vector& x,Vector& out) {
    out.resize(1);
    out[0] = dot(x,x)-1;
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    J.setRow(0,x);
    J *= 2;
  }
};

Note that a Jacobian function must also be defined.


** III.B. Example Code **

With the above constraint, desired endpoints (1,0,0) -> 
(0,1,0), and tolerance 0.001, we can compute an interpolator as 
follows:

#include <mintos/Mintos.h>
using namespace Mintos;

Config qs(3),qe(3);
qs(0) = 1; qs(1) = 0; qs(2) = 0;  
qe(0) = 0; qe(1) = 1; qe(2) = 0;  
double desiredTolerance = 0.001;
double outputTimestep = 0.01;

//set up the list of configuration control points, in this case just
//the start and end milestones
vector<Config> configs(2);
configs[0] = qs;
configs[1] = qe;

UnitSphereFunction C;
TimeScaledBezierCurve curve;
bool res=InterpolateAndTimeOptimize(configs,NULL,&C,desiredTolerance,
				     vmin,vmax,amin,amax,curve);
if(!res) { printf("Error during time scaling optimization\n"); exit(0); }
printf("Optimized path takes time %g\n",curve.EndTime());

If you wish to split the TimeScaledBezierCurve into points, use the
following code:

//retrieve finely discretized waypoints at 100Hz
vector<Config> waypoints;
curve.GetDiscretizedPath(0.01,waypoints);


** III.C. Non-Cartesian interpolation **

This library also supports interpolation in non-Cartesian spaces.
For example, 3D rotations should be interpolated in SO(3), and
can be represented via quaternions, exponential maps, or rotation
matrix entries.

To do so, define a subclass of GeodesicManifold with the proper
Distance(), Interpolate(), etc. methods.

class MyGeodesicManifold : public GeodesicManifold
{
public:
  //Implement Distance(), Interpolate(), etc.
};

And replace the NULL pointer to the InterpolateAndTimeOptimize
call with a pointer to the manifold:

MyGeodesicManifold manifold;
bool res=InterpolateAndTimeOptimize(configs,&manifold,&C,desiredTolerance,
				   vmin,vmax,amin,amax,curve);


** III.D. Torque and force constraints **

To handle time-scaling with torque and/or frictional force constraints, use the classes in mintos/ContactTimeScaling.h.  You must provide hooks to your robot's dynamics equation and information about the robot's contact points.  [Note: before this step, the input keyframes should be interpolated manually to the desired resolution using the functions in mintos/ConstrainedInterpolation.h]

Hooks are provided by subclassing the DynamicsEquationBase class.  The dynamics equation is:
   B(q)q'' + C(q,q') + G(q) = t + J1(q)^T f1 + ... + Jk(q)^T fk
where t is the torque, Ji(q) is the Jacobian of the i'th contact point, and fi is the force at the i'th contact point.
The user must provide hooks to compute:
* B(q)*v for any configuration q and vector v.
* C(q,q') for any state (q,q').
* G(q) for any configuration q.
* J[p,l,t](q) the jacobian of a point p on the link l at configuration q, given in the coordinates of target t.  t=-1 indicates the world frame.

To perform a torque-constrained time scaling, the following framework code can be used:

GeneralizedCubicBezierSpline path;
Vector tmin,tmax;
//TODO: interpolate the path from keyframes and set up torque limits as desired

//TODO: set up your dynamics equation as necessary
MyDynamicsEquation myDynamicsEquation;

//TODO: set up colocation points as desired -- this uses 100 points evenly spaced in the path domain
Real pathDomainMax = path.TotalTime();
vector<Real> colocationPoints(100);
for(size_t i=0;i<colocationPoints.size();i++)
	colocationPoints[i] = pathDomainMax*Real(i)/Real(colocationPoints.size());

//create the time scaling
TorqueTimeScaling scaling(&myDynamicsEquation);
//initialize the path and torque constraints
scaling.Init(path,colocationPoints,tmin,tmax);
//constrains the start and end velocity
scaling.SetStartStop();
//optionally constrains the joint velocity and acceleration
//scaling.SetVelocityBounds(vmin,vmax);
//scaling.SetAccelerationBounds(amin,amax);
if(scaling.Optimize()) {
	//solution is stored in scaling.traj
	printf("Solution found: duration %g\n",scaling.traj.EndTime());
}
else {
	printf("Time scaling failed.\n");
}

To perform a contact-constrained time scaling, a ContactFormation structure must be set up to specify the set of contact points that are allowed to provide the robot with support.  A ContactFormation has a list of body-body contacts, specified by
* links[i]: a link index,
* contacts[i]: a set of contact points,
* targets[i]: for self-contacts, a target link index (-1 indicates robot-world contact).
The targets list can also be empty to indicate that all contacts are robot-world contacts.

Each contact point consists of a position, normal, and Coulomb friction coefficient given in the frame of the target link.  The normal is pointing into the link (toward the robot).  Mintos does not support sliding or rolling contacts.

Mintos supports multiple changing contact regimes by splitting the path into multiple sections, each with a separate ContactFormation.  In this example we will just use a single section.  Code is as follows:

ContactFormation myContactFormation;
//TODO: set up contact formation
vector<GeneralizedCubicBezierSpline> sections(1,path);
vector<ContactFormation> stances(1,myContactFormation);

//create the time scaling
ContactTimeScaling scaling(&myDynamicsEquation);
//initialize the path and all constraints
scaling.Init(sections,colocationPoints,stances,
             tmin,tmax,amin,amax,vmin,vmax);
scaling.SetStartStop();
...


** III.E. Performance issues **

The limiting step in high-D problems is usually manifold interpolation.
The main way to optimize interpolation speed is to improve the speed of
the Newton-Raphson solver used for solving the constraint C(x)=0.  There
are two ways of doing this:
1. Passing a SparseVectorFunction to the SmoothConstrainedInterpolator and
   setting the flag 'SmoothConstrainedInterpolator.solver.sparse = true'.
   This will interpret the Jacobian as a sparse matrix and use sparse
   routines when solving for each linearized projection step.  See
   include/mintos/math/sparsefunction.h
2. Subclassing SmoothConstrainedInterpolator and overriding the Project
   and ProjectVelocity methods with your own custom routines.  For example, 
   if only a robot's feet are constrained, its arm and torso DOFs can be
   ignored.

** III.F. timeopt utility **

Spline files are a list of bezier curve segments.  The format for an
n-segment spline is:

  n
  duration_1   P0[1]   P1[1]   P2[1]  P3[1]
   ...
  duration_n   P0[n]   P1[n]   P2[n]  P3[n]

where each of the Pi[j] are control points, specified as N-D vectors of
the form  'N v1 ... vN'.  Typically paths have uniform durations, either
1 or 1/n. 

Limits files are four vectors

  vel_min
  vel_max
  acc_min
  acc_max

Where each entry is an N-D vector.

Currently the timeopt program does not process the grid resolution
correctly.  This will be fixed in a future version.


**** IV. Contact Information ****

Author: Kris Hauser
Mail:
   School of Informatics and Computing
   919 E. 10th St #257
   Bloomington, IN 47408
Office: (812) 856-7496
Fax: (812) 855-4829
Email: hauserk@indiana.edu
Web: http://cs.indiana.edu/~hauserk


**** V. Version History ****

v1.1 - 11/11/2013  - Improved derivative bounding mechanism, hooks for torque constraints, and routines for force/torque constraints.
v1.0 - 5/3/2013  - Initial release.


**** VI. License ****

Copyright (c) 2013, the Trustees of Indiana University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Indiana University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

