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
#include "MatrixPrinter.h"
#include "complex.h"
#include "ASCIIShade.h"
#include "stringutils.h"
using namespace std;

namespace Math {

MatrixPrinter::MatrixPrinter(const fMatrix& m,Mode _mode)
  :fm(&m),dm(NULL),cm(NULL),delim(' '),bracket('['),mode(_mode)
{}

MatrixPrinter::MatrixPrinter(const dMatrix& m,Mode _mode)
  :fm(NULL),dm(&m),cm(NULL),delim(' '),bracket('['),mode(_mode)
{}

MatrixPrinter::MatrixPrinter(const cMatrix& m,Mode _mode)
  :fm(NULL),dm(NULL),cm(&m),delim(' '),bracket('['),mode(_mode)
{}

template<class T>
void PrintMatrix(const MatrixTemplate<T>& x,ostream& out,char delim,char bracket)
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  MatrixIterator<T> v=x.begin();
  for(int i=0;i<x.m;i++,v.nextRow()) {
    if(bracket) out<<bracket;
    for(int j=0;j<x.n;j++,v.nextCol())
      out<<*v<<delim;
    if(bracket) out<<closebracket;
    if(i+1 != x.m) out<<endl;
  }
  if(bracket) out<<closebracket;
}

template<class T>
void OutputPlusMinus(ostream& out,const MatrixTemplate<T>& x,T zeroTolerance=Epsilon)
{
  MatrixIterator<T> v=x.begin();
  for(int i=0;i<x.m;i++,v.nextRow()) {
    for(int j=0;j<x.n;j++,v.nextCol()) {
      if(*v < -zeroTolerance) out<<'-';
      else if(*v > zeroTolerance) out<<'+';
      else out<<'0';
    }
    if(i+1 != x.m) out<<endl;
  }
}

void MatrixPrinter::Print(ostream& out) const
{
  switch(mode) {
  case Normal:
    if(fm) PrintMatrix(*fm,out,delim,bracket);
    else if(dm) PrintMatrix(*dm,out,delim,bracket);
    else if(cm) PrintMatrix(*cm,out,delim,bracket);
    break;
  case AsciiShade:
    if(fm) OutputASCIIShade(out,*fm);
    else if(dm) OutputASCIIShade(out,*dm);
    else if(cm) { cerr<<"Unable to output an ASCII-shaded complex matrix"<<endl; }
    break;
  case PlusMinus:
    if(fm) OutputPlusMinus(out,*fm);
    else if(dm) OutputPlusMinus(out,*dm);
    else if(cm) { cerr<<"Unable to output an +/- shaded complex matrix"<<endl; }
    break;
  }
}

ostream& operator << (ostream& out,const MatrixPrinter& mp)
{
  mp.Print(out);
  return out;
}

} //namespace Math
