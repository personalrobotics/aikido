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
#include "VectorPrinter.h"
#include "complex.h"
#include "stringutils.h"
#include "ASCIIShade.h"
using namespace std;

namespace Math {

VectorPrinter::VectorPrinter(const fVector& v,Mode _mode)
  :fv(&v),dv(NULL),cv(NULL),delim(' '),bracket('['),mode(_mode)
{}

VectorPrinter::VectorPrinter(const dVector& v,Mode _mode)
  :fv(NULL),dv(&v),cv(NULL),delim(' '),bracket('['),mode(_mode)
{}

VectorPrinter::VectorPrinter(const cVector& v,Mode _mode)
  :fv(NULL),dv(NULL),cv(&v),delim(' '),bracket('['),mode(_mode)
{}

template<class T>
void PrintVector(const VectorTemplate<T>& x,ostream& out,char delim,char bracket)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  VectorIterator<T> v=x.begin();
  for(int i=0; i<x.n; i++,v++)
    out<<*v<<delim;
  if(bracket) out<<closebracket;
}

template <class T>
void OutputPlusMinus(ostream& out,const VectorTemplate<T>& x,T zeroTolerance=Epsilon)
{
  for(int i=0;i<x.n;i++) {
    if(x(i) < -zeroTolerance) out<<'-';
    else if(x(i) > zeroTolerance) out<<'+';
    else out<<'0';
  }
}

void VectorPrinter::Print(ostream& out) const
{
  switch(mode) {
  case Normal:
    if(fv) PrintVector(*fv,out,delim,bracket);
    else if(dv) PrintVector(*dv,out,delim,bracket);
    else if(cv) PrintVector(*cv,out,delim,bracket);
    break;
  case AsciiShade:
    if(fv) OutputASCIIShade(out,*fv);
    else if(dv) OutputASCIIShade(out,*dv);
    else if(cv) { cerr<<"Unable to output an ASCII-shaded complex matrix"<<endl; }
    break;
  case PlusMinus:
    if(fv) OutputPlusMinus(out,*fv);
    else if(dv) OutputPlusMinus(out,*dv);
    else if(cv) { cerr<<"Unable to output an +/- shaded complex matrix"<<endl; }
    break;
  }
}

ostream& operator << (ostream& out,const VectorPrinter& vp)
{
  vp.Print(out);
  return out;
}

} //namespace Math
