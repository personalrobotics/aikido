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
#include <mintos/math/function.h>
#include <mintos/misc/errors.h>
using namespace Math;
using namespace std;


Real ScalarFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h)
{
  Vector grad;
  Gradient(x,grad);
  return grad.dot(h);
}

Real ScalarFieldFunction::DirectionalDeriv2(const Vector& x,const Vector& h)
{
  cerr<<"ScalarFieldFunction::DirectionalDeriv2: Warning, possibly inefficient evaluation"<<endl;
  Matrix H(x.n,x.n);
  Hessian(x,H);
  //calc h^t H h
  Real d=Zero;
  for(int i=0;i<x.n;i++) {
    d += h(i)*H.dotRow(i,h);
  }
  return d;
}


std::string VectorFunction::Label(int i) const
{
  std::string str = Label();
  char buf[32];
  sprintf(buf,"[%d]",i);
  str += buf;
  return str;
}

void VectorFunction::operator()(Real t,Vector& x) { PreEval(t); Eval(t,x); }

std::string ScalarFieldFunction::VariableLabel(int i) const
{
  char buf[32];
  sprintf(buf,"x[%d]",i);
  return buf;
}

std::string VectorFieldFunction::Label() const
{
  return "<unknown Rm->Rn>";
}

std::string VectorFieldFunction::Label(int i) const
{
  std::string str = Label();
  char buf[32];
  sprintf(buf,"[%d]",i);
  str += buf;
  return str;
}

std::string VectorFieldFunction::VariableLabel(int i) const
{
  char buf[32];
  sprintf(buf,"x[%d]",i);
  return buf;
}

Real VectorFieldFunction::Eval_i(const Vector& x,int i)
{
  cout<<"Warning: really inefficient Eval_i"<<endl;
  Vector v(NumDimensions());
  Eval(x,v);
  return v(i);
}

void VectorFieldFunction::Jacobian_i(const Vector& x,int i,Vector& Ji)
{
  Ji.resize(x.n);
  for(int j=0;j<Ji.n;j++)
    Ji(j) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::Jacobian_j(const Vector& x,int j,Vector& Jj)
{
  Jj.resize(NumDimensions());
  for(int i=0;i<Jj.n;i++)
    Jj(i) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::Jacobian(const Vector& x,Matrix& J)
{
  J.resize(NumDimensions(),x.n);
  for(int i=0;i<J.m;i++)
    for(int j=0;j<J.n;j++)
      J(i,j) = Jacobian_ij(x,i,j);
}

void VectorFieldFunction::DirectionalDeriv(const Vector& x,const Vector& h,Vector& v)
{
  Matrix J;
  Jacobian(x,J);
  J.mul(h,v);
}

Real VectorFieldFunction::Divergence(const Vector& x)
{
  Assert(x.n==NumDimensions());
  Real sum=Zero;
  for(int i=0;i<x.n;i++) sum+=Jacobian_ij(x,i,i);
  return sum;
}





