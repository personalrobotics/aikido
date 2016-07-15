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

#ifndef MATH_FUNCTION_H
#define MATH_FUNCTION_H

#include "math.h"
#include "vector.h"
#include "matrix.h"
#include <mintos/misc/errors.h>
#include <string>

/** @ingroup Math
 * @file math/function.h
 * @brief Abstract base classes for function interfaces.
 *
 * Function specializations are located in realfunction.h and vectorfunction.h
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

/** @brief A function from R to R.
 *
 * Subclasses of RealFunction must overload Eval(), and optionally
 * overload Deriv() and Deriv2() (first and second derivatives).
 *
 * NOTE: PreEval(x) MUST be called before evaluating the function value
 * or derivatives using Eval(x), Deriv(x), or Deriv2(x).
 *
 * PreEval() allows reducing the number of computations when 
 * Eval() and Deriv() share some intermediate lengthy computation `Z'.  `Z'
 * can be computed and cached from PreEval(), and then reused when Eval()
 * or Deriv.
 *
 * y=f(x) can be used as a shorthand for f.PreEval(x); y=f.Eval(x).
 */
class RealFunction
{
public:
  virtual ~RealFunction() {}
  virtual std::string Label() const { return "<unknown R->R>"; }
  virtual std::string VariableLabel() const { return "x"; }
  virtual Real operator ()(Real t) { PreEval(t); return Eval(t); }
  virtual void PreEval(Real t) {}
  virtual Real Eval(Real t) =0;
  virtual Real Deriv(Real t) { FatalError("Deriv not defined in subclass of RealFunction"); return 0; }
  virtual Real Deriv2(Real t) { FatalError("Deriv2 not defined in subclass of RealFunction"); return 0; }
};

class RealFunction2
{
public:
  virtual ~RealFunction2() {}
  virtual std::string Label() const { return "<unknown R2->R>"; }
  virtual std::string VariableLabel(int i) const { return (i==0?"t":"x"); }
  virtual Real operator ()(Real t, Real x) { PreEval(t,x); return Eval(t,x); }
  virtual void PreEval(Real t, Real x) {}
  virtual Real Eval(Real t, Real x) =0;
};

/** @brief A function from R to R^n, identical to RealFunction except
 * a vector is returned.
 */
class VectorFunction
{
public:
  virtual ~VectorFunction() {}
  virtual std::string Label() const { return "<unknown R->Rn>"; }
  virtual std::string Label(int i) const;
  virtual std::string VariableLabel() const { return "t"; }
  virtual int NumDimensions() const { FatalError("NumDimensions() not defined in subclass of VectorFunction"); return -1; }
  virtual void operator()(Real t,Vector& x);// { PreEval(t); Eval(t,x); }
  virtual void PreEval(Real t) {}
  virtual void Eval(Real t,Vector& x) =0;
  virtual void Deriv(Real t,Vector& dx) { FatalError("Deriv not defined in subclass of VectorFunction"); }
  virtual void Deriv2(Real t,Vector& ddx) { FatalError("Deriv2 not defined in subclass of VectorFunction"); }
};

/** @brief A function from R^n to R.
 *
 * Takes a vector as input and returns a real value.  Subclasses will overload
 * Eval(), and optionally the derivative evaluation methods.
 *
 * Follows the same calling methodology as RealFunction, that is, PreEval(x)
 * must be called before Eval(x) or any derivative functions.
 *
 * Gradient_i() returns the ith component of the gradient vector.
 * Hessian_ij() returns the (i,j)th component of the hessian matrix.
 * DirectionalDeriv() returns the derivative in the direction h.
 * DirectionalDeriv2() returns the 2nd derivative in the direction h.
 */
class ScalarFieldFunction
{
public:
  virtual ~ScalarFieldFunction() {}
  virtual std::string Label() const { return "<unknown Rn->R>"; }
  virtual std::string VariableLabel(int i) const;
  virtual Real operator ()(const Vector& x) { PreEval(x); return Eval(x); }
  virtual void PreEval(const Vector& x) {}  //called before evaluations at x
  virtual Real Eval(const Vector& x) =0;
  virtual void Gradient(const Vector& x,Vector& grad) { FatalError("Gradient not defined in subclass of ScalarFieldFunction"); }
  virtual Real Gradient_i(const Vector& x,int i) { FatalError("Gradient_i not defined in subclass of ScalarFieldFunction"); return 0; }
  virtual Real DirectionalDeriv(const Vector& x,const Vector& h);
  virtual void Hessian(const Vector& x,Matrix& H) { FatalError("Hessian not defined in subclass of ScalarFieldFunction"); }
  virtual Real Hessian_ij(const Vector& x,int i,int j) { FatalError("Hessian_ij not defined in subclass of ScalarFieldFunction"); return 0; }
  virtual Real DirectionalDeriv2(const Vector& x,const Vector& h);
};


/** @brief A function from R^n to R^m.
 *
 * Takes a vector as input and returns a vector value.  Subclasses will
 * overload NumDimensions() (which should return m), Eval(), and 
 * optionally the derivative evaluation methods.
 *
 * Follows the same calling methodology as RealFunction, that is, PreEval(x)
 * must be called before Eval(x) or any derivative functions.
 *
 * Eval_i() returns the ith component of Eval(x)
 * Jacobian() returns the Jacobian matrix, where Jij = dfi / dxj.
 * Jacobian_i() returns the ith row of the Jacobian matrix.
 * Jacobian_j() returns the jth column of the Jacobian matrix.
 * Jacobian_ij() returns the (i,j)th component of the Jacobian matrix.
 * DirectionalDeriv() returns the derivative in the direction h (i.e. J*h).
 * Divergence() returns the trace of the Jacobian.  Only valid for m=n.
 * Hessian_i() returns the Hessian matrix of the ith component function.
 * Hessian_ijk() returns the (j,k)th entry of the ith Hessian matrix.
 */
class VectorFieldFunction
{
public:
  virtual ~VectorFieldFunction() {}
  virtual std::string Label() const;
  virtual std::string Label(int i) const;
  virtual std::string VariableLabel(int i) const;
  virtual int NumDimensions() const { FatalError("NumDimensions() not defined in subclass of VectorFieldFunction"); return -1; }
  virtual void operator ()(const Vector& x,Vector& v) { PreEval(x);Eval(x,v); }
  virtual void PreEval(const Vector& x) {}  //called before evaluations at x
  virtual void Eval(const Vector& x,Vector& v) =0;
  virtual Real Eval_i(const Vector& x,int i);
  virtual Real Jacobian_ij(const Vector& x,int i,int j) { FatalError("Jacobian_ij() not defined in subclass of VectorFieldFunction"); return 0; }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji);
  virtual void Jacobian_j(const Vector& x,int j,Vector& Jj);
  virtual void Jacobian(const Vector& x,Matrix& J);
  virtual void DirectionalDeriv(const Vector& x,const Vector& h,Vector& v);
  virtual Real Divergence(const Vector& x);

  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) { FatalError("Hessian_i() not defined in subclass of VectorFieldFunction"); }
  virtual Real Hessian_ijk(const Vector& x,int i,int j,int k) { FatalError("Hessian_ijk() not defined in subclass of VectorFieldFunction"); return 0; }
};


} //namespace Math

#endif
