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


#include "Timer.h"
#include <stdlib.h>

#ifdef WIN32
#define GETCURRENTTIME(x) x=timeGetTime()
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
#define GETCURRENTTIME(x) clock_gettime(CLOCK_MONOTONIC,&x)
#else
#define GETCURRENTTIME(x) gettimeofday(&x,NULL)
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32

// Sadly, timersub isn't defined in Solaris. :(
// So we use this instead. (added by Ryan)

#if defined (__SVR4) && defined (__sun)
#include "timersub.h"
#endif

Timer::Timer()
{
  Reset();
}

void Timer::Reset()
{
  GETCURRENTTIME(start);
  current=start;
}

long long Timer::ElapsedTicks()
{
  GETCURRENTTIME(current);
  return LastElapsedTicks();
}

long long Timer::LastElapsedTicks() const
{
#ifdef WIN32
  return current-start;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  long long ticks = (current.tv_sec-start.tv_sec)*1000 + (current.tv_nsec-start.tv_nsec)/1000000;
  return ticks;
#else
  timeval delta;
  timersub(&current,&start,&delta);
  long long ticks = delta.tv_sec*1000 + delta.tv_usec/1000;
  return ticks;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32
}
    
double Timer::ElapsedTime()
{
  GETCURRENTTIME(current);
  return LastElapsedTime();
}

double Timer::LastElapsedTime() const
{
#ifdef WIN32
  return double(current-start)/1000.0;
#else
#ifdef  _POSIX_MONOTONIC_CLOCK
  double secs=double(current.tv_sec-start.tv_sec);
  secs += double(current.tv_nsec-start.tv_nsec)/1000000000.0;
  return secs;
#else
  timeval delta;
  timersub(&current,&start,&delta);
  double secs=double(delta.tv_sec);
  secs += double(delta.tv_usec)/1000000.0;
  return secs;
#endif //_POSIX_MONOTONIC_CLOCK
#endif //WIN32
}

/*
clock_t Timer::ElapsedTicks()
{
  current = clock();
  return (current-start);
}

double Timer::ElapsedTime()
{
  current = clock();
  return double(current-start)/CLOCKS_PER_SEC;
}

clock_t Timer::LastElapsedTicks() const
{
  return current-start;
}

double Timer::LastElapsedTime() const
{
  return double(current-start)/CLOCKS_PER_SEC;
}
*/
