#ifndef MATH_DIFFERENTIATION_H
#define MATH_DIFFERENTIATION_H

#include "function.h"

/** @file math/differentiation.h"
 * @ingroup Math
 * @brief Numerical differentiation routines.
 */

namespace Math {
/** @addtogroup Math */
/*@{*/

/// centered differences
/// f'(x) ~= 1/2h*(f(x+h) - f(x-h)) + O(h^2)
Real dfCenteredDifference(RealFunction& f, Real x, Real h);
void dfCenteredDifference(VectorFunction& f, Real x, Real h, Vector& df);

/// adaptive centered differences with h determined dynamically
/// starting from h0 such that the (approximate) error is no greater than tol
Real dfCenteredDifferenceAdaptive(RealFunction& f, Real x, Real h0, Real tol);
void dfCenteredDifferenceAdaptive(VectorFunction& f, Real x, Real h0, Real tol, Vector& df);

/// centered differences for 2nd derivative
/// f''(x) ~= 1/h^2*(f(x+h) - 2f(x) + f(x-h)) + O(h^2)
Real ddfCenteredDifference(RealFunction& f, Real x, Real h);
void ddfCenteredDifference(VectorFunction& f, Real x, Real h, Vector& ddf);

/// x is provided as a temporary, it's restored to its original value later
void GradientForwardDifference(ScalarFieldFunction& f,Vector& x,Real h,Vector& g);
void JacobianForwardDifference(VectorFieldFunction& f,Vector& x,Real h,Matrix& J);
void HessianForwardDifference(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H);
void HessianForwardDifference_Grad(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H);

void GradientCenteredDifference(ScalarFieldFunction& f,Vector& x,Real h,Vector& g);
void JacobianCenteredDifference(VectorFieldFunction& f,Vector& x,Real h,Matrix& J);
void HessianCenteredDifference(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H);
void HessianCenteredDifference_Grad(ScalarFieldFunction& f,Vector& x,Real h,Matrix& H);

/// adaptive centered differences with h determined dynamically
/// starting from h0 such that the (approximate) error is no greater than tol
void GradientCenteredDifferenceAdaptive(ScalarFieldFunction& f,Vector& x,Real h0,Real tol,Vector& g);

/// specified hi in each direction xi
void GradientForwardDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Vector& g);
void JacobianForwardDifference(VectorFieldFunction& f,Vector& x,const Vector& h,Matrix& J);
void HessianForwardDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H);
void HessianForwardDifference_Grad(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H);

void GradientCenteredDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Vector& g);
void JacobianCenteredDifference(VectorFieldFunction& f,Vector& x,const Vector& h,Matrix& J);
void HessianCenteredDifference(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H);
void HessianCenteredDifference_Grad(ScalarFieldFunction& f,Vector& x,const Vector& h,Matrix& H);

/*@}*/
} //namespace Math

#endif
