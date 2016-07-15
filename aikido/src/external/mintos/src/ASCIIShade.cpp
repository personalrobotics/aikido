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
#include "ASCIIShade.h"
using namespace std;

namespace Math {

const static int kNumAsciiShades = 15;
const static char kAsciiShades[15] = {
  'W',
  'w',
  '#',
  '%',
  '&',
  '*',
  '+',
  ' ',  //0 is element 7
  '.',
  ':',
  'o',
  '0',
  '8',
  'O',
  '@',
};
  

char ASCIIShade(double x)
{
  if(IsNaN(x)) return 'E';
  if(IsInf(x)==1) return 'I';
  else if(IsInf(x)==-1) return 'i';
  int index = (int)Trunc(x*8) + 7;
  if(index < 0) index=0;
  if(index >= kNumAsciiShades) index=kNumAsciiShades-1;
  if(index == 7) {
    if(x > 0) return kAsciiShades[8];
    else if(x < 0) return kAsciiShades[6];
    else return kAsciiShades[7];
  }
  return kAsciiShades[index];
}

void OutputASCIIShade(ostream& out,double x)
{
  out<<ASCIIShade(x);
}

void OutputASCIIShade(ostream& out,const fVector& x,float scale)
{
  if(scale == 0) scale = x.maxAbsElement();
  out<<scale<<" x ";
  if(scale == 0) scale = 1;
  out<<'[';
  for(int i=0;i<x.n;i++)
    out<<ASCIIShade(x(i)/scale);
  out<<']';
}

void OutputASCIIShade(ostream& out,const dVector& x,double scale)
{
  if(scale == 0) scale = x.maxAbsElement();
  out<<scale<<" x ";
  if(scale == 0) scale = 1;
  out<<'[';
  for(int i=0;i<x.n;i++)
    out<<ASCIIShade(x(i)/scale);
  out<<']';
}

void OutputASCIIShade(ostream& out,const fMatrix& A,float scale)
{
  if(scale == 0) scale = A.maxAbsElement();
  out<<scale<<" x"<<endl;
  if(scale == 0) scale = 1;
  for(int i=0;i<A.m;i++) {
    out<<'[';
    for(int j=0;j<A.n;j++) 
      out<<ASCIIShade(A(i,j)/scale);
    out<<']';
    if(i+1 < A.m)
      out<<endl;
  }
}


void OutputASCIIShade(ostream& out,const dMatrix& A,double scale)
{
  if(scale == 0) scale = A.maxAbsElement();
  out<<scale<<" x"<<endl;
  if(scale == 0) scale = 1;
  for(int i=0;i<A.m;i++) {
    out<<'[';
    for(int j=0;j<A.n;j++) 
      out<<ASCIIShade(A(i,j)/scale);
    out<<']';
    if(i+1 < A.m)
      out<<endl;
  }
}


} //namespace Math
