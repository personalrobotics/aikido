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

#ifndef MINTOS_ERRORS_H
#define MINTOS_ERRORS_H

#include <assert.h>
#include <stdio.h>
#include <stdarg.h>


//error reporting

#define Assert assert
#define Abort abort

inline void RaiseErrorFmt(const char* func, const char* file, int line, const char* fmt, ...)
{
  fprintf(stderr,"Error in %s (%s:%d): ", func,file,line); 
  va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
  fprintf(stderr,"\n");
  Abort();
}

inline void RaiseErrorFmt(const char* fmt,...)
{
  fprintf(stderr,"Error (unknown function): ");
  va_list args;
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
  fprintf(stderr,"\n");
  Abort();
}


inline void RaiseError(const char* func, const char* file, int line, const char* text)
{
  fprintf(stderr,"Error in %s (%s:%d): %s\n", func,file,line,text); 
  Abort();
}



//the following is bending over backwards to support MS's lack of 
//variable argument macro support

#ifdef HAVE_PRETTY_FUNCTION
#define WHERE_AM_I __PRETTY_FUNCTION__, __FILE__, __LINE__
#else
#define WHERE_AM_I __FUNCTION__, __FILE__, __LINE__
#endif

//Error1 is guaranteed to print line numbers with a single-argument error
#define FatalError1(text) RaiseError(WHERE_AM_I,text)

#if HAVE_VARARGS_MACROS
#define FatalError(fmt,...) RaiseErrorFmt(WHERE_AM_I,fmt,__VA_ARGS__)
#else
//if no variable arguments, can't get any line info 
#define FatalError RaiseErrorFmt
#endif

#define AssertNotReached() RaiseError(WHERE_AM_I,"Code should not be reached")

#endif

