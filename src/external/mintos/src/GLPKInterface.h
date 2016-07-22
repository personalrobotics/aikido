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
#ifndef OPTIMIZATION_GLPK_INTERFACE_H
#define OPTIMIZATION_GLPK_INTERFACE_H

#include "LinearProgram.h"
#if HAVE_GLPK

#include <glpk.h>

#else
#define glp_prob void
#endif

namespace Optimization {

/** @ingroup Optimization
 * @brief An interface to the GLPK linear program solver.  Activated with the
 * HAVE_GLPK preprocessor define.
 */
struct GLPKInterface
{
  GLPKInterface();
  ~GLPKInterface();
  //easiest interface
  void Set(const LinearProgram& LP);
  void Set(const LinearProgram_Sparse& LP);
  LinearProgram::Result Solve(Vector& xopt);
  //low level commands
  void Create(int m,int n);
  void Clear();
  void SetObjective(const Vector& c,bool minimize=true);
  void SetRow(int i,const Vector& Ai);
  void SetRowBounds(int i,Real low,Real high);		    
  void SetVariableBounds(int j,Real low,Real high);

  //warm starting the solver
  void SetRowBasic(int i);  //inactive
  void SetRowNonBasic(int i,bool upper=false);  //active
  void SetVariableBasic(int i);  //inactive
  void SetVariableNonBasic(int i,bool upper=false);  //active
  bool GetRowBasic(int i);  //inactive
  bool GetVariableBasic(int i);  //inactive
  double GetRowDual(int i);
  double GetVariableDual(int j);

  static bool Enabled();
  static void SelfTest();

  glp_prob* lp;
};

} //namespace Optimization

#endif
