#ifndef ARRAY_UTILS_H
#define ARRAY_UTILS_H

#include <algorithm>
#include <functional>
#include <vector>
#if defined (__GNUC__) && (__GNUC__ > 2)
#include <ext/algorithm>
namespace std {
  using __gnu_cxx::copy_n;
}
#endif
#include <assert.h>


/** @file utils/arrayutils.h
 * @ingroup Utils
 * @brief Convenience routines for C arrays and STL vectors.
 */

/** @ingroup Utils
 * @brief Convenience routines for C arrays and STL vectors
 */
namespace ArrayUtils {

/** @addtoGroup Utils */
/*@{*/

template <typename T>
inline void copy(const T& a, T* out, int n)
{
  std::fill(out,out+n,a);
}

template <typename T>
inline void copy(const T* a, T* out, int n)
{
  std::copy_n(a,n,out);
}

template <typename T,typename ftype>
inline void foreach(T* a, ftype f,int n)
{
  std::for_each(a,a+n,f);
}

template <typename T>
inline void reverse (T *a, int n)
{
  std::reverse(a,a+n);
}


template <typename t1,typename t2,typename ftype>
inline void transform(const t1* a, t2* out, ftype f,int n)
{
  std::transform(a,a+n,out,f);
}

template <typename t1,typename t2,typename t3,typename ftype>
inline void binary_transform(const t1* a, const t2* b, t3* out, ftype f,int n)
{
  std::transform(a,a+n,b,out,f);
}

template <typename T>
inline void add(const T* a, const T* b, T* out, int n)
{
  binary_transform(a,b,out,std::plus<T>(),n);
}

template <typename T>
inline void sub(const T* a, const T* b, T* out, int n)
{
  binary_transform(a,b,out,std::minus<T>(),n);
}

template <typename T>
inline void mul(const T* a, const T* b, T* out, int n)
{
  binary_transform(a,b,out,std::multiplies<T>(),n);
}

template <typename T>
inline void div(const T* a, const T* b, T* out, int n)
{
  binary_transform(a,b,out,std::divides<T>(),n);
}

///returns the nth largest element in the array a
template <typename T>
inline T nth_element (const std::vector<T>& S, size_t n)
{
  assert(n < S.size());
  size_t i=rand()%S.size();
  const T& m=S[i];
  std::vector<T>S1,S2;
  S1.reserve(n);
  S2.reserve(n);
  for(i=0;i<S.size();i++) {
    if(S[i] < m) S1.push_back(S[i]);
    else if(m < S[i]) S2.push_back(S[i]);
  }
  if(S1.size() > n) return nth_element(S1,n,std::less<T>());
  else if(S.size()-S2.size()>=n) return m;
  else return nth_element(S2,n-(S.size()-S2.size()),std::less<T>());
}

///returns the nth largest element in the array a
template <typename T,typename fless>
inline T nth_element (const std::vector<T>& S, size_t n, fless less)
{
  assert(n < S.size());
  size_t i=rand()%S.size();
  const T& m=S[i];
  std::vector<T>S1,S2;
  S1.reserve(n);
  S2.reserve(n);
  for(i=0;i<S.size();i++) {
    if(less(S[i],m)) S1.push_back(S[i]);
    else if(less(m,S[i])) S2.push_back(S[i]);
  }
  if(S1.size() > n) return nth_element(S1,n,less);
  else if(S.size()-S2.size()>=n) return m;
  else return nth_element(S2,n-(S.size()-S2.size()),less);
}

template <typename T>
inline bool is_sorted(T* a, int n)
{
  for(int i=1;i<n;i++)
    if(a[i]<a[i-1]) return false;
  return true;
  //return std::is_sorted(a,a+n);
}

template <typename T,typename fless>
inline bool is_sorted(T* a, int n, fless f)
{
  for(int i=1;i<n;i++)
    if(f(a[i],a[i-1])) return false;
  return true;
  //return std::is_sorted(a,a+n,f);
}

template <typename T>
inline void quicksort(T* a, int p, int r)
{
  if(p < r) {
    T x = a[p];
    T temp;
    int i = p;
    int j = p + 1;
    while (j <= r) {
      if (a[j] < x) {
        i++;
        temp=a[j];a[j]=a[i];a[i]=temp;
      }
      j++;
    }
    temp=a[p];a[p]=a[i];a[i]=temp;

    quicksort(a,p,i-1);
    quicksort(a,i+1,r);
  }
}

template <typename T,typename fless>
inline void quicksort(T* a, int p, int r,fless f)
{
  if(p < r) {
    T x = a[p];
    T temp;
    int i = p;
    int j = p + 1;
    while (j <= r) {
      if (f(a[j],x)) {
        i++;
        temp=a[j];a[j]=a[i];a[i]=temp;
      }
      j++;
    }
    temp=a[p];a[p]=a[i];a[i]=temp;

    quicksort(a,p,i-1,f);
    quicksort(a,i+1,r,f);
  }
}

template <typename T>
inline void sort(T* a, int n)
{
  quicksort(a,0,n-1);
}

template <typename T,typename fless>
inline void sort(T* a, int n, fless f)
{
  quicksort(a,0,n-1,f);
}

///Concatenates b onto the end of a
template <class T>
inline void concat(std::vector<T>& a,const std::vector<T>& b)
{
  size_t aorig=a.size();
  a.resize(a.size()+b.size());
  copy(b.begin(),b.end(),a.begin()+aorig);
}

/*@}*/

} //namespace ArrayUtils

#endif
