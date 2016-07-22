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

#ifndef BASIC_UTILS_H
#define BASIC_UTILS_H

/** @file utils.h
 * @brief Utilities commonly used throughout a program.  
 *
 * Many of these are here just to provide a cross-platform interface
 * to non-standard build environments (e.g. windows)
 */

/** @def Min
 * @brief Operates exactly like std::min, but works on windows platforms
 * that include windows.h.
 */

/** @def Max
 * @brief Operates exactly like std::max, but works on windows platforms
 * that include windows.h.
 */

/** @def SafeDelete
 * @brief Delete a non-NULL pointer and set it to NULL.
 */

/** @def SafeArrayDelete
 * @brief Delete a non-NULL pointer to an array and set it to NULL.
 */

/** @def SafeDeleteProc
 * @brief Delete a non-NULL pointer, using the provided function,
 *  and set it to NULL.
 */

/** @def ARRAYSIZE
 * @brief Returns the size of the array x[].
 *
 * Returns n for constant-sized arrays declared as type x[n].  Does not
 * work for dynamic arrays allocated on the heap.
 */

#ifdef  _MSC_VER
template<class type>
inline type Max(const type& a,const type& b) { return (a>b?a:b); }
template<class type>
inline type Min(const type& a,const type& b) { return (a<b?a:b); }
template<class type>
inline void Swap(type& a,type& b) { type temp=a; a=b; b=temp; }
#else
#include <algorithm>
template<class type>
inline type Max(const type& a,const type& b) { return std::max(a,b); }
template<class type>
inline type Min(const type& a,const type& b) { return std::min(a,b); }
template<class type>
inline void Swap(type& a,type& b) { std::swap(a,b); }
#endif //_MSC_VER

template <class type>
inline type Max(const type& a,const type& b,const type& c)
{
  return Max(Max(a,b),c);
}

template <class type>
inline type Max(const type& a,const type& b,const type& c,const type& d)
{
  return Max(Max(a,b),Max(c,d));
}

template <class type>
inline type Min(const type& a,const type& b,const type& c)
{
  return Min(Min(a,b),c);
}

template <class type>
inline type Min(const type& a,const type& b,const type& c,const type& d)
{
  return Min(Min(a,b),Min(c,d));
}



inline void toggle(bool& bit)
{
  bit=!bit;
}

#define SafeDelete(x) { if (x) delete x; x=NULL; }
#define SafeArrayDelete(x) { if (x) delete [] x; x=NULL; }
#define SafeDeleteProc(x,proc) { if (x) proc(x); x=NULL; }

#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))
#endif

#endif

