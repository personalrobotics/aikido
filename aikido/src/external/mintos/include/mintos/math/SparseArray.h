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

#ifndef SPARSE_ARRAY_H
#define SPARSE_ARRAY_H

#include <map>
#include <assert.h>

/** @brief A sparse 1D array class.
 *
 * Acts just like a map of integers to values.
 */
template <class T>
class SparseArray
{
public:
  typedef std::map<int,T> Storage;
  typedef typename Storage::iterator iterator;
  typedef typename Storage::const_iterator const_iterator;

  SparseArray() :n(0) {}
  SparseArray(size_t _n) :n(_n) {}
  inline void resize(size_t _n) { n=_n; assert(isValid()); }
  inline void clear() { entries.clear(); n=0; }
  inline bool empty() const { return n==0; }
  inline size_t size() const { return n; }
  inline size_t numEntries() const { return entries.size(); }
  inline iterator begin() { return entries.begin(); }
  inline iterator end() { return entries.end(); }
  inline const_iterator begin() const { return entries.begin(); }
  inline const_iterator end() const { return entries.end(); }
  inline iterator insert(int i,const T& t) {
    std::pair<int,T> p;
    p.first=i;
    iterator res=entries.insert(p).first;
    res->second=t;
    return res;
  }
  inline iterator push_back(int i,const T& t) {
    std::pair<int,T> p;
    p.first=i;
    iterator res=entries.insert(end(),p);
    res->second=t;
    return res;
  }
  inline iterator find(int i) { return entries.find(i); }
  inline const_iterator find(int i) const { return entries.find(i); }
  inline bool erase(int i) { return entries.erase(i)!=0; }
  inline void erase(const iterator& it) { entries.erase(it); }
  inline bool isValidIndex(int i) const { return i>=0&&i<(int)n; }
  inline bool isValid() const 
  {
    for(const_iterator i=entries.begin();i!=entries.end();i++)
      if(i->first < 0 || i->first >= (int)n) return false;
    return true;
  }

  Storage entries;
  size_t n;
};


#endif
