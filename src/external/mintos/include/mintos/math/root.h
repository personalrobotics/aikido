/*****************************************************************************
 *
 * Copyright (c) 2013, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ***************************************************************************/

#ifndef MATH_ROOT_H
#define MATH_ROOT_H

#include "function.h"

/** @file root.h
 * @ingroup Math
 * @brief Numerical root-solving routines.
 */

namespace Math {
  /** @addtogroup Math */
  /*@{*/

enum ConvergenceResult { ConvergenceX, ConvergenceF, Divergence, LocalMinimum, MaxItersReached, ConvergenceError };

std::ostream& operator<<(std::ostream&, ConvergenceResult r);

/** @brief Uses bisection to bracket f's root.
 *
 * [a,b] must be a range such that f(a) and f(b) have opposing signs.
 * [a,b] is returned as a refined bracket.
 * Linear convergence.
 * The number of iterations used is ceil(log2(b-a)-log2(eps)).
 */
ConvergenceResult Root_Bisection(RealFunction& f,Real& a, Real& b, Real& x, Real tol);

/** @brief Uses the secant method to search for f's root.
 *
 * Order of convergence ~1.6. The # of iterations is returned in maxIters.
 */
ConvergenceResult Root_Secant(RealFunction& f,Real x0, Real& x1, int& maxIters, Real tolx, Real tolf);

///Same as above, but the interval [a,b] must be known to contain a root. 
///[a,b] is returned as a refined bracket.
ConvergenceResult Root_SecantBracket(RealFunction& f,Real& a, Real& b, Real& x, int& maxIters, Real tolx, Real tolf);

/** @brief Performs Newton's method, which uses the derivative of f to 
 * search for it's root.
 *
 * f.Deriv() must be defined.  Quadratic convergence.  The # of iterations 
 * is returned in maxIters.
 */
ConvergenceResult Root_Newton(RealFunction& f,Real& x, int& maxIters, Real tolx, Real tolf);

/// Same as above, but the interval [a,b] must be known to contain a root.
ConvergenceResult Root_NewtonBracket(RealFunction& f,Real a, Real b, Real& x, int& maxIters, Real tolx, Real tolf);

/// Same as Root_Newton(), but takes a scalar field function as input
ConvergenceResult Root_Newton(ScalarFieldFunction& f,const Vector& x0, Vector& x, int& maxIters, Real tolx, Real tolf);


/// Same as Root_Newton(), but takes a vector field function as input
ConvergenceResult Root_Newton(VectorFieldFunction& f,const Vector& x0, Vector& x, int& maxIters, Real tolx, Real tolf);

  /*@}*/
} // namespace Math

#endif
